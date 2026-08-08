"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const sceneApi = require("../../monitor/web/radar_scene.js");

function makeGrid(forwardCells = 2, lateralCells = 3) {
  const bytes = Uint8Array.from({ length: forwardCells * lateralCells }, () => 0);
  return {
    bytes,
    payload: {
      encoding: "occupancy-u8-base64",
      layout: "forward-major_lateral-minor",
      unknown_value: 0,
      resolution_m: 0.05,
      forward_cells: forwardCells,
      lateral_cells: lateralCells,
      origin_forward_cell: 0,
      origin_lateral_cell: Math.floor(lateralCells / 2),
      data_base64: Buffer.from(bytes).toString("base64"),
    },
  };
}

function makeSnapshot(overrides = {}) {
  const grid = makeGrid(60, 60).payload;
  const snapshot = {
    status: "live",
    scene: {
      schema_version: 1,
      pose_mode: "robot_relative",
      calibration_status: "ok",
      grid,
      tracks: [],
      hazard: {
        level: "NORMAL",
        threshold_m: 0.1,
        reason: "confirmed_point_outside_threshold",
      },
    },
  };
  return {
    ...snapshot,
    ...overrides,
    scene: { ...snapshot.scene, ...(overrides.scene || {}) },
  };
}

test("decodes forward-major occupancy and preserves zero as UNKNOWN", () => {
  const grid = makeGrid(60, 60);
  grid.bytes[59 * 60 + 30] = 255;
  grid.payload.data_base64 = Buffer.from(grid.bytes).toString("base64");

  const decoded = sceneApi.decodeOccupancyGrid(grid.payload);

  assert.equal(decoded.length, 3600);
  assert.equal(decoded[0], 0);
  assert.equal(decoded[59 * 60 + 30], 255);
});

test("rejects non-canonical base64 before decoding", () => {
  const grid = makeGrid(1, 1);
  grid.payload.data_base64 = "AP==";

  assert.throws(() => sceneApi.decodeOccupancyGrid(grid.payload), /base64/);
});

test("rejects a grid whose decoded length does not match dimensions", () => {
  const grid = makeGrid(2, 2);
  grid.payload.data_base64 = Buffer.from([0, 1, 2]).toString("base64");

  assert.throws(() => sceneApi.decodeOccupancyGrid(grid.payload), /length/);
});

test("rejects grid metadata with an origin outside its declared dimensions", () => {
  const grid = makeGrid(2, 2);
  grid.payload.origin_lateral_cell = 2;

  assert.throws(() => sceneApi.decodeOccupancyGrid(grid.payload), /grid contract/);
});

test("drops a track at the exact 300ms freshness boundary", () => {
  const tracks = [
    { track_id: 1, age_ms: 299 },
    { track_id: 2, age_ms: 300 },
    { track_id: 3, age_ms: -1 },
  ];

  assert.deepEqual(sceneApi.filterFreshTracks(tracks), [{ track_id: 1, age_ms: 299 }]);
});

test("separates confirmed obstacles from reflection evidence", () => {
  const obstacle = {
    track_id: 1,
    age_ms: 0,
    source: "point",
    point_confirmed: true,
    classification: "confirmed_obstacle",
    distance_m: 0.4,
    forward_m: 0.4,
    lateral_m: 0,
  };
  const reflection = {
    track_id: 2,
    age_ms: 0,
    source: "heatmap",
    point_confirmed: false,
    classification: "reflection_evidence",
    distance_m: 0.6,
    forward_m: 0.6,
    lateral_m: 0.1,
  };
  const parsed = sceneApi.parseRadarScene(makeSnapshot({
    scene: {
      tracks: [obstacle, reflection],
      obstacle_tracks: [obstacle],
      reference_tracks: [reflection],
    },
  }));

  assert.deepEqual(parsed.tracks.map((track) => track.track_id), [1]);
  assert.deepEqual(parsed.referenceTracks.map((track) => track.track_id), [2]);
});

test("preserves the one-centimeter planned path and motion estimate", () => {
  const snapshot = makeSnapshot({
    scene: {
      planned_path: {
        status: "limited",
        resolution_m: 0.01,
        length_m: 0.02,
        points: [[0, 0], [0.01, 0], [0.02, 0.001]],
      },
      motion_compensation: {
        status: "ok",
        applied: true,
        matched_points: 5,
      },
    },
  });

  const parsed = sceneApi.parseRadarScene(snapshot);

  assert.equal(parsed.plannedPath.resolution_m, 0.01);
  assert.equal(parsed.plannedPath.points.length, 3);
  assert.equal(parsed.motionCompensation.applied, true);
  assert.equal(parsed.motionCompensation.matched_points, 5);
});

test("rejects a planned path that is not sampled at one centimeter", () => {
  assert.throws(
    () => sceneApi.parseRadarScene(makeSnapshot({
      scene: {
        planned_path: {
          status: "clear",
          resolution_m: 0.05,
          length_m: 1,
          points: [[0, 0], [0.05, 0]],
        },
      },
    })),
    /planned path contract/,
  );
});

test("preserves five-second accumulated strong obstacles", () => {
  const obstacle = {
    obstacle_id: 3,
    forward_m: 0.7,
    lateral_m: -0.2,
    confidence: 0.9,
    age_ms: 1200,
    remaining_ms: 3800,
  };
  const parsed = sceneApi.parseRadarScene(makeSnapshot({
    scene: {
      accumulated_obstacles: [obstacle],
      obstacle_memory_window_ms: 5000,
    },
  }));

  assert.deepEqual(parsed.accumulatedObstacles, [obstacle]);
  assert.equal(parsed.obstacleMemoryWindowMs, 5000);
});

test("prefers motion-relative persistent near obstacles for display", () => {
  const persistent = {
    forward_m: 0.35,
    lateral_m: 0.1,
    confidence: 0.91,
    age_ms: 900,
    remaining_ms: 4100,
  };
  const parsed = sceneApi.parseRadarScene(makeSnapshot({
    scene: {
      accumulated_obstacles: [],
      persistent_near_obstacles: [persistent],
      persistent_near_obstacle_memory: { window_ms: 5000 },
    },
  }));

  assert.deepEqual(parsed.accumulatedObstacles, [persistent]);
  assert.equal(parsed.obstacleMemoryWindowMs, 5000);
});

test("preserves the robot-centred fifty-centimeter rolling map", () => {
  const behindRobot = {
    forward_m: -0.20,
    lateral_m: 0.10,
    distance_m: Math.hypot(0.20, 0.10),
    confidence: 0.95,
    hits: 4,
    age_ms: 60_000,
  };
  const parsed = sceneApi.parseRadarScene(makeSnapshot({
    scene: {
      rolling_local_map: {
        radius_m: 0.50,
        resolution_m: 0.005,
        minimum_hits: 2,
        motion_relative: true,
        retention: "until_outside_radius_or_scene_reset",
        cells: [behindRobot],
      },
    },
  }));

  assert.deepEqual(parsed.rollingMapObstacles, [behindRobot]);
  assert.equal(parsed.rollingMap.radiusM, 0.50);
  assert.equal(parsed.rollingMap.motionRelative, true);
});

test("preserves the radar-motion travel trail ending at robot center", () => {
  const parsed = sceneApi.parseRadarScene(makeSnapshot({
    scene: {
      travel_trail: {
        radius_m: 0.50,
        resolution_m: 0.01,
        motion_relative: true,
        source: "radar_ego_motion",
        points: [[-0.20, 0.04], [-0.10, 0.02], [0, 0]],
      },
    },
  }));

  assert.deepEqual(
    parsed.travelTrail.points,
    [[-0.20, 0.04], [-0.10, 0.02], [0, 0]],
  );
  assert.equal(parsed.travelTrail.source, "radar_ego_motion");
});

test("normal preserves uncertainty without safe or free surface claims", () => {
  const parsed = sceneApi.parseRadarScene(makeSnapshot());

  assert.equal(parsed.hazardCopy, "10cm 이내 확인 장애물 없음 · 미관측 영역 존재");
  assert.equal(parsed.safe, undefined);
  assert.equal(parsed.free, undefined);
  assert.equal(parsed.inferredWall, undefined);
  assert.equal(parsed.inferredSurface, undefined);
});

test("rejects an unsupported scene schema", () => {
  assert.throws(
    () => sceneApi.parseRadarScene(makeSnapshot({ scene: { schema_version: 2 } })),
    /unsupported scene schema/,
  );
});

test("requires the robot-relative pose contract and returns validated pose", () => {
  const parsed = sceneApi.parseRadarScene(makeSnapshot());
  assert.equal(parsed.poseMode, "robot_relative");

  for (const poseMode of [undefined, null, "motion_compensated"]) {
    assert.throws(
      () => sceneApi.parseRadarScene(makeSnapshot({
        scene: { pose_mode: poseMode },
      })),
      /pose contract/,
      String(poseMode),
    );
  }
});

test("accepts the short-memory pose enum during additive map rollout", () => {
  const snapshot = makeSnapshot();
  snapshot.scene.pose_mode = "robot_relative_short_memory";
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.poseMode, "robot_relative_short_memory");
});

test("rejects each scene-v1 grid metadata mismatch independently", async (t) => {
  const mismatches = [
    ["resolution_m", 0.1],
    ["forward_cells", 59],
    ["lateral_cells", 61],
    ["origin_forward_cell", 1],
    ["origin_lateral_cell", 29],
    ["encoding", "occupancy-u8"],
    ["layout", "lateral-major_forward-minor"],
    ["unknown_value", 255],
  ];
  for (const [field, value] of mismatches) {
    await t.test(field, () => {
      const snapshot = makeSnapshot();
      snapshot.scene.grid = { ...snapshot.scene.grid, [field]: value };
      assert.throws(
        () => sceneApi.parseRadarScene(snapshot),
        /scene grid contract/,
      );
    });
  }
});

test("blocks waiting, stale, fault, and replay-ended sources", () => {
  for (const status of ["waiting", "stale", "fault", "replay_end"]) {
    const parsed = sceneApi.parseRadarScene(makeSnapshot({ status }));
    assert.equal(parsed.blocked, true, status);
    assert.equal(parsed.reason, status);
  }
});

test("renders only live and degraded source statuses", () => {
  for (const status of ["live", "degraded"]) {
    assert.equal(sceneApi.parseRadarScene(makeSnapshot({ status })).blocked, false, status);
  }
  for (const status of [undefined, "ready", "future_status"]) {
    const parsed = sceneApi.parseRadarScene(makeSnapshot({ status }));
    assert.equal(parsed.blocked, true, String(status));
    assert.equal(parsed.reason, status);
  }
});

test("blocks all non-renderable calibration states", () => {
  for (const calibrationStatus of [
    "calibration_required",
    "calibration_unavailable",
    "profile_mismatch",
  ]) {
    const parsed = sceneApi.parseRadarScene(
      makeSnapshot({ scene: { calibration_status: calibrationStatus } }),
    );
    assert.equal(parsed.blocked, true, calibrationStatus);
    assert.equal(parsed.reason, calibrationStatus);
  }
});

test("blocks a trusted live scene whose hazard reports SENSOR_FAULT", () => {
  const parsed = sceneApi.parseRadarScene(makeSnapshot({
    status: "live",
    scene: {
      calibration_status: "ok",
      hazard: {
        level: "SENSOR_FAULT",
        threshold_m: 0.1,
        reason: "source_fault",
      },
    },
  }));

  assert.equal(parsed.blocked, true);
  assert.equal(parsed.reason, "sensor_fault");
});

test("rejects hazard levels outside the scene vocabulary", () => {
  for (const level of ["SAFE", "FREE", "unknown"]) {
    assert.throws(
      () => sceneApi.parseRadarScene(makeSnapshot({
        scene: { hazard: { level, threshold_m: 0.1, reason: "forged" } },
      })),
      /hazard contract/,
      level,
    );
  }
});

test("rejects DANGER when only a forged heatmap confirmation is inside threshold", () => {
  const snapshot = makeSnapshot({
    scene: {
      tracks: [{
        track_id: 7,
        age_ms: 0,
        source: "heatmap",
        point_confirmed: true,
        distance_m: 0.09,
        forward_m: 0.09,
        lateral_m: 0,
      }],
      hazard: { level: "DANGER", threshold_m: 0.1, reason: "confirmed" },
    },
  });

  assert.throws(() => sceneApi.parseRadarScene(snapshot), /DANGER contract/);
});

test("rejects DANGER backed by a negative point distance", () => {
  const snapshot = makeSnapshot({
    scene: {
      tracks: [{
        track_id: 10,
        age_ms: 0,
        source: "point",
        point_confirmed: true,
        distance_m: -1,
        forward_m: 0,
        lateral_m: 0,
      }],
      hazard: { level: "DANGER", threshold_m: 0.1, reason: "forged" },
    },
  });

  assert.throws(() => sceneApi.parseRadarScene(snapshot), /DANGER contract/);
});

test("rejects a fresh track without finite map coordinates", () => {
  const snapshot = makeSnapshot({
    scene: {
      tracks: [{
        track_id: 11,
        age_ms: 0,
        source: "heatmap",
        point_confirmed: false,
        distance_m: 0.4,
        forward_m: 0.4,
        lateral_m: Infinity,
      }],
    },
  });

  assert.throws(() => sceneApi.parseRadarScene(snapshot), /track contract/);
});

test("renders a fresh confirmed point DANGER without mutating the snapshot", () => {
  const snapshot = makeSnapshot({
    scene: {
      tracks: [{
        track_id: 8,
        age_ms: 0,
        source: "point",
        point_confirmed: true,
        distance_m: 0.1,
        forward_m: 0.1,
        lateral_m: 0,
      }],
      hazard: { level: "DANGER", threshold_m: 0.1, reason: "confirmed" },
    },
  });
  const before = structuredClone(snapshot);

  const parsed = sceneApi.parseRadarScene(snapshot);

  assert.equal(parsed.blocked, false);
  assert.equal(parsed.tracks[0].track_id, 8);
  assert.deepEqual(snapshot, before);
  assert.notStrictEqual(parsed.tracks[0], snapshot.scene.tracks[0]);
});

test("rejects a DANGER threshold that is not finite numeric metadata", () => {
  const snapshot = makeSnapshot({
    scene: {
      tracks: [{
        track_id: 9,
        age_ms: 0,
        source: "point",
        point_confirmed: true,
        distance_m: 0.09,
      }],
      hazard: { level: "DANGER", threshold_m: "0.1", reason: "confirmed" },
    },
  });

  assert.throws(() => sceneApi.parseRadarScene(snapshot), /hazard contract/);
});

test("projects forward and lateral meters into a centered forward map", () => {
  const transform = sceneApi.makeMapTransform(200, 100, 5, 2);

  assert.deepEqual(transform, { originX: 100, originY: 100, scale: 20 });
  assert.deepEqual(sceneApi.projectMapPoint(transform, 1.5, -2), { x: 60, y: 70 });
});

test("exposes its UMD API and decodes through the browser atob branch", () => {
  const browserGlobal = {
    atob: (text) => Buffer.from(text, "base64").toString("binary"),
    btoa: (text) => Buffer.from(text, "binary").toString("base64"),
  };
  const source = fs.readFileSync(
    require.resolve("../../monitor/web/radar_scene.js"),
    "utf8",
  );

  vm.runInNewContext(source, browserGlobal);

  const bytes = browserGlobal.HanselRadarScene.decodeOccupancyGrid({
    encoding: "occupancy-u8-base64",
    layout: "forward-major_lateral-minor",
    unknown_value: 0,
    resolution_m: 0.05,
    forward_cells: 1,
    lateral_cells: 1,
    origin_forward_cell: 0,
    origin_lateral_cell: 0,
    data_base64: "AA==",
  });
  assert.equal(browserGlobal.HanselRadarScene.TRACK_MAX_AGE_MS, 300);
  assert.equal(bytes[0], 0);
});

test("panel diagnostics reads pose only from the validated presentation", () => {
  const elements = new Map();
  const document = {
    querySelector(selector) {
      if (!elements.has(selector)) {
        elements.set(selector, {
          dataset: {},
          hidden: false,
          textContent: "",
        });
      }
      return elements.get(selector);
    },
  };
  const browserGlobal = {
    document,
    window: {
      HanselRadarScene: sceneApi,
    },
  };
  const source = fs.readFileSync(
    require.resolve("../../monitor/web/radar_panel.js"),
    "utf8",
  );
  vm.runInNewContext(source, browserGlobal);
  const panel = Object.create(browserGlobal.window.HanselRadarPanel.prototype);
  panel.snapshot = {
    axes: {
      forward_axis: "y",
      forward_sign: 1,
      lateral_axis: "x",
      lateral_sign: 1,
    },
    frame: null,
    scene: {
      pose_mode: "motion_compensated",
      calibration_status: "ok",
      diagnostics: {},
    },
    counters: {},
  };

  panel.updateDiagnostics({
    blocked: false,
    poseMode: "robot_relative",
    tracks: [],
    grid: new Uint8Array(3600),
    hazard: { level: "UNKNOWN" },
  });

  assert.equal(
    elements.get("#pose-mode-value").textContent,
    "ROBOT RELATIVE",
  );
});
