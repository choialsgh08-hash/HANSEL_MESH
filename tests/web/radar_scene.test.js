"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const sceneApi = require("../../monitor/web/radar_scene.js");

const ADDITIVE_DIAGNOSTIC_KEYS = [
  "raw_point_count",
  "self_clutter_rejected_count",
  "quality_gate_rejected_count",
  "tentative_point_track_count",
  "confirmed_point_track_count",
  "heatmap_reflection_count",
  "confirmed_heatmap_track_count",
  "probability_active_cell_count",
  "probability_occupied_cell_count",
  "probability_decayed_cell_count",
  "probability_ttl_expired_cell_count",
  "path_switch_pending_count",
  "path_switch_accepted_count",
  "scene_epoch",
];

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

function makePointTrack(overrides = {}) {
  return {
    track_id: 1,
    age_ms: 0,
    source: "point",
    classification: "confirmed_obstacle",
    point_confirmed: true,
    map_confirmed: true,
    forward_m: 0.40,
    lateral_m: 0.0,
    distance_m: 0.40,
    height_m: 0.0,
    height_band: "sensor_level",
    height_reference: "sensor_relative_uncalibrated",
    confidence: 0.9,
    range_uncertainty_m: 0.049,
    lateral_uncertainty_m: 0.05,
    ...overrides,
  };
}

function makeHeatmapTrack(overrides = {}) {
  return {
    track_id: 2,
    age_ms: 0,
    source: "heatmap",
    classification: "reflection_evidence",
    heatmap_confirmed: false,
    forward_m: 0.35,
    lateral_m: -0.03,
    distance_m: 0.351,
    height_m: null,
    height_band: "unknown",
    height_reference: "unavailable",
    confidence: 0.5,
    range_uncertainty_m: 0.098,
    lateral_uncertainty_m: 0.06,
    ...overrides,
  };
}

function makeProbabilityCell(overrides = {}) {
  return {
    forward_m: 0.30,
    lateral_m: 0.05,
    probability: 0.8,
    confidence: 0.9,
    occupied: true,
    hits: 3,
    age_ms: 125,
    range_uncertainty_m: 0.049,
    lateral_uncertainty_m: 0.06,
    source: "point",
    ...overrides,
  };
}

function makeSnapshot(overrides = {}) {
  const grid = makeGrid(60, 60).payload;
  const diagnostics = Object.fromEntries(
    ADDITIVE_DIAGNOSTIC_KEYS.map((key) => [key, 0]),
  );
  const snapshot = {
    status: "live",
    scene: {
      schema_version: 1,
      pose_mode: "robot_relative_short_memory",
      calibration_status: "ok",
      grid,
      tracks: [],
      obstacle_tracks: [],
      reference_tracks: [],
      probabilistic_local_map: {
        radius_m: 0.50,
        resolution_m: 0.05,
        occupied_threshold: 0.75,
        decay_half_life_ms: 1000,
        ttl_ms: 2500,
        retention: "probability_decay_with_ttl",
        pose_mode: "robot_relative_short_memory",
        cells: [],
      },
      rolling_local_map: {
        radius_m: 0.50,
        resolution_m: 0.05,
        minimum_hits: 1,
        motion_relative: true,
        retention: "probability_decay_with_ttl",
        cells: [],
      },
      planned_path: {
        status: "clear",
        stability_status: "replanned",
        resolution_m: 0.01,
        length_m: 0.02,
        curvature_per_m: 0.0,
        points: [[0.0, 0.0], [0.01, 0.0], [0.02, 0.0]],
        robot_width_m: 0.15,
        safety_margin_m: 0.03,
        minimum_passage_width_m: 0.21,
        minimum_hold_ms: 0,
        switch_confirm_frames: 3,
        pending_switch_frames: 0,
        switch_accepted: false,
        replan_every_frame: true,
        blocked_previous_path: null,
      },
      diagnostics,
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

function makeLegacySnapshot(overrides = {}) {
  const snapshot = makeSnapshot(overrides);
  delete snapshot.scene.probabilistic_local_map;
  snapshot.scene.pose_mode = "robot_relative";
  for (const key of ADDITIVE_DIAGNOSTIC_KEYS) {
    delete snapshot.scene.diagnostics[key];
  }
  snapshot.scene.rolling_local_map = {
    radius_m: 0.50,
    resolution_m: 0.005,
    minimum_hits: 2,
    motion_relative: true,
    retention: "until_outside_radius_or_scene_reset",
    cells: [{
      forward_m: -0.20,
      lateral_m: 0.10,
      confidence: 0.95,
      hits: 4,
      age_ms: 60_000,
    }],
  };
  snapshot.scene.planned_path = {
    status: "limited",
    resolution_m: 0.01,
    length_m: 0.02,
    points: [[0, 0], [0.01, 0], [0.02, 0.001]],
  };
  return snapshot;
}

function setPath(snapshot, overrides) {
  snapshot.scene.planned_path = {
    ...snapshot.scene.planned_path,
    ...overrides,
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

test("rejects malformed occupancy payloads", async (t) => {
  await t.test("non-canonical base64", () => {
    const grid = makeGrid(1, 1).payload;
    grid.data_base64 = "AP==";
    assert.throws(() => sceneApi.decodeOccupancyGrid(grid), /base64/);
  });
  await t.test("length mismatch", () => {
    const grid = makeGrid(2, 2).payload;
    grid.data_base64 = Buffer.from([0, 1, 2]).toString("base64");
    assert.throws(() => sceneApi.decodeOccupancyGrid(grid), /length/);
  });
  await t.test("out-of-range origin", () => {
    const grid = makeGrid(2, 2).payload;
    grid.origin_lateral_cell = 2;
    assert.throws(() => sceneApi.decodeOccupancyGrid(grid), /grid contract/);
  });
});

test("drops a track at the exact 300ms freshness boundary", () => {
  const tracks = [
    { track_id: 1, age_ms: 299 },
    { track_id: 2, age_ms: 300 },
    { track_id: 3, age_ms: -1 },
  ];
  assert.deepEqual(sceneApi.filterFreshTracks(tracks), [{ track_id: 1, age_ms: 299 }]);
});

test("prefers probability cells over the rolling compatibility alias", () => {
  const probabilityCell = makeProbabilityCell();
  const snapshot = makeSnapshot();
  snapshot.scene.probabilistic_local_map.cells = [probabilityCell];
  snapshot.scene.rolling_local_map.cells = [
    { ...probabilityCell, forward_m: 0.45 },
  ];
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.probabilisticLocalMap.sourceField, "probabilistic_local_map");
  assert.deepEqual(parsed.probabilisticLocalMap.cells, [probabilityCell]);
  assert.equal(parsed.probabilisticLocalMap.radiusM, 0.50);
  assert.equal(parsed.probabilisticLocalMap.resolutionM, 0.05);
  assert.equal(parsed.probabilisticLocalMap.occupiedThreshold, 0.75);
  assert.equal(parsed.probabilisticLocalMap.decayHalfLifeMs, 1000);
  assert.equal(parsed.probabilisticLocalMap.ttlMs, 2500);
  assert.equal(parsed.probabilisticLocalMap.retention, "probability_decay_with_ttl");
  assert.equal(parsed.probabilisticLocalMap.poseMode, "robot_relative_short_memory");
});

test("presence selects new mode and invalid present data never falls back", async (t) => {
  for (const [name, value] of [
    ["null", null],
    ["string", "invalid"],
    ["array", []],
    ["number", 1],
  ]) {
    await t.test(name, () => {
      const snapshot = makeSnapshot();
      snapshot.scene.probabilistic_local_map = value;
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /probabilistic map contract/);
    });
  }
  await t.test("invalid object with valid rolling alias", () => {
    const snapshot = makeSnapshot();
    snapshot.scene.probabilistic_local_map.radius_m = 0.49;
    assert.throws(() => sceneApi.parseRadarScene(snapshot), /probabilistic map contract/);
  });
});

test("rejects every strict probability-map metadata mismatch", async (t) => {
  const cases = [
    ["radius_m", 0.49],
    ["resolution_m", 0.10],
    ["occupied_threshold", 0.70],
    ["decay_half_life_ms", 999],
    ["ttl_ms", 2499],
    ["retention", "until_reset"],
    ["pose_mode", "robot_relative"],
    ["cells", null],
  ];
  for (const [field, invalid] of cases) {
    await t.test(field, () => {
      const snapshot = makeSnapshot();
      snapshot.scene.probabilistic_local_map[field] = invalid;
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /probabilistic map contract/);
    });
  }
});

test("rejects every strict probability cell violation", async (t) => {
  const cases = [
    ["missing forward_m", (cell) => delete cell.forward_m],
    ["non-finite forward_m", (cell) => { cell.forward_m = Infinity; }],
    ["non-finite lateral_m", (cell) => { cell.lateral_m = NaN; }],
    ["outside radial boundary", (cell) => { cell.forward_m = 0.40; cell.lateral_m = 0.40; }],
    ["probability at prior", (cell) => { cell.probability = 0.50; cell.occupied = false; }],
    ["probability above cap", (cell) => { cell.probability = 0.98; }],
    ["non-finite probability", (cell) => { cell.probability = NaN; }],
    ["negative confidence", (cell) => { cell.confidence = -0.01; }],
    ["confidence above one", (cell) => { cell.confidence = 1.01; }],
    ["non-finite confidence", (cell) => { cell.confidence = Infinity; }],
    ["non-boolean occupied", (cell) => { cell.occupied = 1; }],
    ["zero hits", (cell) => { cell.hits = 0; }],
    ["negative hits", (cell) => { cell.hits = -1; }],
    ["fractional hits", (cell) => { cell.hits = 1.5; }],
    ["missing hits", (cell) => delete cell.hits],
    ["negative age", (cell) => { cell.age_ms = -1; }],
    ["age at ttl", (cell) => { cell.age_ms = 2500; }],
    ["non-finite age", (cell) => { cell.age_ms = Infinity; }],
    ["missing range uncertainty", (cell) => delete cell.range_uncertainty_m],
    ["negative range uncertainty", (cell) => { cell.range_uncertainty_m = -0.01; }],
    ["non-finite range uncertainty", (cell) => { cell.range_uncertainty_m = NaN; }],
    ["missing lateral uncertainty", (cell) => delete cell.lateral_uncertainty_m],
    ["negative lateral uncertainty", (cell) => { cell.lateral_uncertainty_m = -0.01; }],
    ["non-finite lateral uncertainty", (cell) => { cell.lateral_uncertainty_m = Infinity; }],
    ["invalid source", (cell) => { cell.source = "camera"; }],
    ["occupied below threshold", (cell) => { cell.probability = 0.70; cell.occupied = true; }],
    ["unoccupied at threshold", (cell) => { cell.probability = 0.75; cell.occupied = false; }],
  ];
  for (const [name, mutate] of cases) {
    await t.test(name, () => {
      const snapshot = makeSnapshot();
      const cell = makeProbabilityCell();
      mutate(cell);
      snapshot.scene.probabilistic_local_map.cells = [cell];
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /probabilistic map contract/);
    });
  }
});

test("validates and preserves each supported height band/reference pairing", async (t) => {
  const cases = [
    ["below sensor", makePointTrack({ height_m: -0.11, height_band: "below_sensor" })],
    ["sensor level", makePointTrack({ height_m: 0.0, height_band: "sensor_level" })],
    ["above sensor", makePointTrack({ height_m: 0.11, height_band: "above_sensor" })],
    ["heatmap unavailable", makeHeatmapTrack()],
  ];
  for (const [name, track] of cases) {
    await t.test(name, () => {
      const snapshot = makeSnapshot();
      snapshot.scene.tracks = [track];
      const parsed = sceneApi.parseRadarScene(snapshot);
      assert.deepEqual(parsed.filteredTracks, [track]);
    });
  }
});

test("rejects every strict new-mode track violation before freshness filtering", async (t) => {
  const cases = [
    ["missing identity", () => makePointTrack({ track_id: undefined })],
    ["zero identity", () => makePointTrack({ track_id: 0 })],
    ["negative identity", () => makePointTrack({ track_id: -1 })],
    ["fractional identity", () => makePointTrack({ track_id: 1.5 })],
    ["missing age", () => makePointTrack({ age_ms: undefined })],
    ["negative age", () => makePointTrack({ age_ms: -1 })],
    ["fractional stale age", () => makePointTrack({ age_ms: 300.5 })],
    ["non-finite age", () => makePointTrack({ age_ms: Infinity })],
    ["invalid source", () => makePointTrack({ source: "camera" })],
    ["invalid classification", () => makePointTrack({ classification: "wall" })],
    ["missing forward", () => makePointTrack({ forward_m: undefined })],
    ["non-finite forward", () => makePointTrack({ forward_m: Infinity })],
    ["non-finite lateral", () => makePointTrack({ lateral_m: NaN })],
    ["non-finite distance", () => makePointTrack({ distance_m: Infinity })],
    ["distance/hypot mismatch", () => makePointTrack({ distance_m: 0.402 })],
    ["negative confidence", () => makePointTrack({ confidence: -0.01 })],
    ["confidence above one", () => makePointTrack({ confidence: 1.01 })],
    ["missing range uncertainty", () => makePointTrack({ range_uncertainty_m: undefined })],
    ["negative range uncertainty", () => makePointTrack({ range_uncertainty_m: -0.01 })],
    ["non-finite range uncertainty", () => makePointTrack({ range_uncertainty_m: NaN })],
    ["missing lateral uncertainty", () => makePointTrack({ lateral_uncertainty_m: undefined })],
    ["negative lateral uncertainty", () => makePointTrack({ lateral_uncertainty_m: -0.01 })],
    ["non-finite lateral uncertainty", () => makePointTrack({ lateral_uncertainty_m: Infinity })],
    ["point classification mismatch", () => makePointTrack({ point_confirmed: false })],
    ["missing point confirmation", () => makePointTrack({ point_confirmed: undefined })],
    ["missing map confirmation", () => makePointTrack({ map_confirmed: undefined })],
    ["non-boolean map confirmation", () => makePointTrack({ map_confirmed: 1 })],
    ["point wrong height band", () => makePointTrack({ height_m: 0.11, height_band: "sensor_level" })],
    ["point unavailable reference", () => makePointTrack({ height_reference: "unavailable" })],
    ["point missing finite height", () => makePointTrack({ height_m: null, height_band: "unknown" })],
    ["heatmap classification mismatch", () => makeHeatmapTrack({ heatmap_confirmed: true })],
    ["missing heatmap confirmation", () => makeHeatmapTrack({ heatmap_confirmed: undefined })],
    ["heatmap invented height", () => makeHeatmapTrack({ height_m: 0, height_band: "sensor_level", height_reference: "sensor_relative_uncalibrated" })],
    ["heatmap invented reference", () => makeHeatmapTrack({ height_reference: "sensor_relative_uncalibrated" })],
  ];
  for (const [name, makeTrack] of cases) {
    await t.test(name, () => {
      const snapshot = makeSnapshot();
      snapshot.scene.tracks = [makeTrack()];
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /track contract/);
    });
  }
});

test("filtered tracks retain tentative point and heatmap reflection evidence", () => {
  const tracks = [
    makePointTrack({
      track_id: 11,
      classification: "point_candidate",
      point_confirmed: false,
      map_confirmed: false,
      forward_m: 0.30,
      lateral_m: 0.02,
      distance_m: 0.301,
      confidence: 0.6,
    }),
    makeHeatmapTrack({ track_id: 12 }),
  ];
  const snapshot = makeSnapshot();
  snapshot.scene.tracks = tracks;
  snapshot.scene.obstacle_tracks = [];
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.deepEqual(parsed.filteredTracks.map((track) => track.track_id), [11, 12]);
  assert.deepEqual(parsed.tracks, []);
});

test("separates confirmed obstacles from validated reflection evidence", () => {
  const obstacle = makePointTrack();
  const reflection = makeHeatmapTrack();
  const snapshot = makeSnapshot();
  snapshot.scene.tracks = [obstacle, reflection];
  snapshot.scene.obstacle_tracks = [obstacle];
  snapshot.scene.reference_tracks = [reflection];
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.deepEqual(parsed.tracks.map((track) => track.track_id), [1]);
  assert.deepEqual(parsed.referenceTracks.map((track) => track.track_id), [2]);
});

test("requires every additive diagnostic to be a non-negative integer", async (t) => {
  const invalidValues = [
    ["missing", Symbol("delete")],
    ["null", null],
    ["negative", -1],
    ["fractional", 0.5],
    ["not finite", NaN],
  ];
  for (const key of ADDITIVE_DIAGNOSTIC_KEYS) {
    for (const [label, invalid] of invalidValues) {
      await t.test(`${key} ${label}`, () => {
        const snapshot = makeSnapshot();
        if (typeof invalid === "symbol") {
          delete snapshot.scene.diagnostics[key];
        } else {
          snapshot.scene.diagnostics[key] = invalid;
        }
        assert.throws(() => sceneApi.parseRadarScene(snapshot), /diagnostics contract/);
      });
    }
  }
});

test("returns validated diagnostics and recursively detached presentation data", () => {
  const snapshot = makeSnapshot();
  const cell = makeProbabilityCell();
  const point = makePointTrack();
  snapshot.scene.probabilistic_local_map.cells = [cell];
  snapshot.scene.tracks = [point];
  snapshot.scene.obstacle_tracks = [point];
  snapshot.scene.diagnostics.raw_point_count = 7;
  setPath(snapshot, {
    stability_status: "route_changed_blocked",
    switch_accepted: true,
    blocked_previous_path: {
      points: [[0, 0], [0.01, 0]],
      segments: [{ start_index: 1, end_index: 1, reason: "occupied" }],
    },
  });
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.diagnostics.raw_point_count, 7);

  parsed.diagnostics.nested = { changed: true };
  parsed.probabilisticLocalMap.cells[0].source = "changed";
  parsed.filteredTracks[0].height_band = "changed";
  parsed.tracks[0].height_reference = "changed";
  parsed.plannedPath.points[1][0] = 99;
  parsed.plannedPath.blocked_previous_path.points[0][0] = 99;
  parsed.plannedPath.blocked_previous_path.segments[0].reason = "changed";

  assert.equal(snapshot.scene.diagnostics.nested, undefined);
  assert.equal(cell.source, "point");
  assert.equal(point.height_band, "sensor_level");
  assert.equal(point.height_reference, "sensor_relative_uncalibrated");
  assert.equal(snapshot.scene.planned_path.points[1][0], 0.01);
  assert.equal(snapshot.scene.planned_path.blocked_previous_path.points[0][0], 0);
  assert.equal(snapshot.scene.planned_path.blocked_previous_path.segments[0].reason, "occupied");
});

test("rejects malformed or missing physical and selector path fields", async (t) => {
  const cases = [
    ["missing status", "status", undefined],
    ["invalid status", "status", "safe"],
    ["missing stability", "stability_status", undefined],
    ["invalid stability", "stability_status", "quiet"],
    ["resolution", "resolution_m", 0.02],
    ["non-finite length", "length_m", Infinity],
    ["negative length", "length_m", -0.01],
    ["non-finite curvature", "curvature_per_m", NaN],
    ["points", "points", null],
    ["robot width missing", "robot_width_m", undefined],
    ["robot width", "robot_width_m", 0.16],
    ["safety margin missing", "safety_margin_m", undefined],
    ["safety margin", "safety_margin_m", 0.04],
    ["passage width missing", "minimum_passage_width_m", undefined],
    ["passage width", "minimum_passage_width_m", 0.20],
    ["hold missing", "minimum_hold_ms", undefined],
    ["hold", "minimum_hold_ms", 1],
    ["confirmation missing", "switch_confirm_frames", undefined],
    ["confirmation", "switch_confirm_frames", 2],
    ["pending missing", "pending_switch_frames", undefined],
    ["pending negative", "pending_switch_frames", -1],
    ["pending fractional", "pending_switch_frames", 0.5],
    ["pending at limit", "pending_switch_frames", 3],
    ["accepted missing", "switch_accepted", undefined],
    ["accepted non-boolean", "switch_accepted", 0],
    ["replan missing", "replan_every_frame", undefined],
    ["replan false", "replan_every_frame", false],
    ["blocked overlay missing", "blocked_previous_path", undefined],
  ];
  for (const [name, field, invalid] of cases) {
    await t.test(name, () => {
      const snapshot = makeSnapshot();
      if (invalid === undefined) {
        delete snapshot.scene.planned_path[field];
      } else {
        snapshot.scene.planned_path[field] = invalid;
      }
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /planned path contract/);
    });
  }
});

test("rejects invalid new-mode path geometry and status combinations", async (t) => {
  const cases = [
    ["non-pair point", { points: [[0, 0], [0.01]] }],
    ["non-finite point", { points: [[0, 0], [Infinity, 0]] }],
    ["route does not start at origin", { points: [[0.01, 0], [0.02, 0], [0.03, 0]] }],
    ["route is not sampled at one centimeter", { points: [[0, 0], [0.02, 0]], length_m: 0.02 }],
    ["length disagrees with geometry", { length_m: 0.03 }],
    ["neutral status with route stability", { status: "initializing" }],
    ["route status with neutral stability", { stability_status: "reset" }],
    ["accepted without route change", { switch_accepted: true }],
    ["route change not accepted", { stability_status: "route_changed_confirmed" }],
  ];
  for (const [name, overrides] of cases) {
    await t.test(name, () => {
      const snapshot = makeSnapshot();
      setPath(snapshot, overrides);
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /planned path contract/);
    });
  }
});

test("requires pending path selector count to match diagnostics", () => {
  const snapshot = makeSnapshot();
  setPath(snapshot, { stability_status: "holding_candidate", pending_switch_frames: 1 });
  assert.throws(() => sceneApi.parseRadarScene(snapshot), /planned path contract/);
});

test("accepts neutral initializing and reset paths without inventing a route", async (t) => {
  for (const status of ["initializing", "reset"]) {
    await t.test(status, () => {
      const snapshot = makeSnapshot();
      setPath(snapshot, {
        status,
        stability_status: status,
        length_m: 0,
        points: [[0, 0]],
      });
      const parsed = sceneApi.parseRadarScene(snapshot);
      assert.equal(parsed.plannedPath.status, status);
      assert.equal(parsed.plannedPath.stability_status, status);
      assert.equal(parsed.plannedPath.length_m, 0);
      assert.deepEqual(parsed.plannedPath.points, [[0, 0]]);
    });
  }
});

test("preserves a NORMAL blocked-near-field stop", () => {
  const snapshot = makeSnapshot();
  setPath(snapshot, {
    status: "blocked_near_field",
    stability_status: "route_changed_blocked",
    length_m: 0,
    points: [[0, 0]],
    switch_accepted: true,
  });
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.hazard.level, "NORMAL");
  assert.equal(parsed.plannedPath.status, "blocked_near_field");
  assert.equal(parsed.plannedPath.stability_status, "route_changed_blocked");
});

test("validates every blocked previous path point and segment field", async (t) => {
  const validOverlay = () => ({
    points: [[0, 0], [0.01, 0], [0.02, 0]],
    segments: [{ start_index: 1, end_index: 2, reason: "occupied" }],
  });
  const cases = [
    ["null points", (overlay) => { overlay.points = null; }],
    ["non-pair point", (overlay) => { overlay.points[1] = [0.01]; }],
    ["non-finite point", (overlay) => { overlay.points[1] = [NaN, 0]; }],
    ["null segments", (overlay) => { overlay.segments = null; }],
    ["non-object segment", (overlay) => { overlay.segments[0] = null; }],
    ["fractional start", (overlay) => { overlay.segments[0].start_index = 0.5; }],
    ["negative start", (overlay) => { overlay.segments[0].start_index = -1; }],
    ["end outside points", (overlay) => { overlay.segments[0].end_index = 3; }],
    ["reversed inclusive indices", (overlay) => { overlay.segments[0].start_index = 2; overlay.segments[0].end_index = 1; }],
    ["empty reason", (overlay) => { overlay.segments[0].reason = ""; }],
    ["whitespace reason", (overlay) => { overlay.segments[0].reason = "   "; }],
    ["non-string reason", (overlay) => { overlay.segments[0].reason = 1; }],
  ];
  for (const [name, mutate] of cases) {
    await t.test(name, () => {
      const snapshot = makeSnapshot();
      const overlay = validOverlay();
      mutate(overlay);
      setPath(snapshot, {
        stability_status: "route_changed_blocked",
        switch_accepted: true,
        blocked_previous_path: overlay,
      });
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /planned path contract/);
    });
  }
});

test("preserves a valid blocked previous path overlay", () => {
  const snapshot = makeSnapshot();
  setPath(snapshot, {
    stability_status: "route_changed_blocked",
    switch_accepted: true,
    blocked_previous_path: {
      points: [[0, 0], [0.01, 0], [0.02, 0]],
      segments: [{ start_index: 1, end_index: 2, reason: "occupied" }],
    },
  });
  assert.deepEqual(
    sceneApi.parseRadarScene(snapshot).plannedPath.blocked_previous_path,
    snapshot.scene.planned_path.blocked_previous_path,
  );
});

test("preserves a validated blocked overlay after the changed route stabilizes", () => {
  const snapshot = makeSnapshot();
  setPath(snapshot, {
    stability_status: "stable",
    blocked_previous_path: {
      points: [[0, 0], [0.01, 0], [0.02, 0]],
      segments: [{ start_index: 1, end_index: 2, reason: "occupied" }],
    },
  });
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.plannedPath.stability_status, "stable");
  assert.equal(
    parsed.plannedPath.blocked_previous_path.segments[0].reason,
    "occupied",
  );
});

test("preserves the one-centimeter planned path and motion estimate", () => {
  const snapshot = makeSnapshot({
    scene: {
      motion_compensation: { status: "ok", applied: true, matched_points: 5 },
    },
  });
  setPath(snapshot, {
    status: "limited",
    points: [[0, 0], [0.01, 0], [0.02, 0.001]],
  });
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.plannedPath.resolution_m, 0.01);
  assert.equal(parsed.plannedPath.points.length, 3);
  assert.equal(parsed.motionCompensation.applied, true);
  assert.equal(parsed.motionCompensation.matched_points, 5);
});

test("true legacy absence preserves decoded grid and rolling circles", () => {
  const legacyTrack = {
    track_id: 41,
    age_ms: 0,
    source: "point",
    point_confirmed: false,
    forward_m: 0.40,
    lateral_m: 0,
  };
  const snapshot = makeLegacySnapshot();
  snapshot.scene.tracks = [legacyTrack];
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.equal(parsed.poseMode, "robot_relative");
  assert.equal(parsed.probabilisticLocalMap, null);
  assert.equal(parsed.grid.length, 3600);
  assert.deepEqual(parsed.rollingMapObstacles, snapshot.scene.rolling_local_map.cells);
  assert.deepEqual(parsed.filteredTracks, [legacyTrack]);
  assert.notStrictEqual(parsed.filteredTracks[0], legacyTrack);
  for (const key of ADDITIVE_DIAGNOSTIC_KEYS) {
    assert.equal(parsed.diagnostics[key], null, key);
  }
});

test("new and true legacy modes require their exact pose values", () => {
  assert.equal(sceneApi.parseRadarScene(makeSnapshot()).poseMode, "robot_relative_short_memory");
  assert.equal(sceneApi.parseRadarScene(makeLegacySnapshot()).poseMode, "robot_relative");

  const wrongNew = makeSnapshot();
  wrongNew.scene.pose_mode = "robot_relative";
  assert.throws(() => sceneApi.parseRadarScene(wrongNew), /pose contract/);

  const wrongLegacy = makeLegacySnapshot();
  wrongLegacy.scene.pose_mode = "robot_relative_short_memory";
  assert.throws(() => sceneApi.parseRadarScene(wrongLegacy), /pose contract/);
});

test("preserves legacy accumulated obstacle, travel trail, and rolling metadata", () => {
  const snapshot = makeLegacySnapshot();
  snapshot.scene.accumulated_obstacles = [{
    obstacle_id: 3,
    forward_m: 0.7,
    lateral_m: -0.2,
    confidence: 0.9,
    age_ms: 1200,
    remaining_ms: 3800,
  }];
  snapshot.scene.obstacle_memory_window_ms = 5000;
  snapshot.scene.travel_trail = {
    radius_m: 0.50,
    resolution_m: 0.01,
    motion_relative: true,
    source: "radar_ego_motion",
    points: [[-0.20, 0.04], [-0.10, 0.02], [0, 0]],
  };
  const parsed = sceneApi.parseRadarScene(snapshot);
  assert.deepEqual(parsed.accumulatedObstacles, snapshot.scene.accumulated_obstacles);
  assert.equal(parsed.obstacleMemoryWindowMs, 5000);
  assert.deepEqual(parsed.travelTrail.points, snapshot.scene.travel_trail.points);
  assert.equal(parsed.rollingMap.radiusM, 0.50);
  assert.equal(parsed.rollingMap.motionRelative, true);
});

test("normal preserves uncertainty without safe or free surface claims", () => {
  const parsed = sceneApi.parseRadarScene(makeSnapshot());
  assert.equal(parsed.safe, undefined);
  assert.equal(parsed.free, undefined);
  assert.equal(parsed.inferredWall, undefined);
  assert.equal(parsed.inferredSurface, undefined);
});

test("rejects unsupported schema and every scene-v1 grid mismatch", async (t) => {
  assert.throws(
    () => sceneApi.parseRadarScene(makeSnapshot({ scene: { schema_version: 2 } })),
    /unsupported scene schema/,
  );
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
      assert.throws(() => sceneApi.parseRadarScene(snapshot), /scene grid contract/);
    });
  }
});

test("preserves all blocked-source, calibration, and sensor-fault rules", () => {
  for (const status of ["waiting", "stale", "fault", "replay_end"]) {
    const parsed = sceneApi.parseRadarScene(makeSnapshot({ status }));
    assert.deepEqual(parsed, { blocked: true, reason: status });
  }
  for (const calibrationStatus of [
    "calibration_required",
    "calibration_unavailable",
    "calibration_mismatch",
    "profile_mismatch",
  ]) {
    const parsed = sceneApi.parseRadarScene(
      makeSnapshot({ scene: { calibration_status: calibrationStatus } }),
    );
    assert.deepEqual(parsed, { blocked: true, reason: calibrationStatus });
  }
  const fault = makeSnapshot({
    scene: {
      hazard: { level: "SENSOR_FAULT", threshold_m: 0.1, reason: "source_fault" },
    },
  });
  assert.deepEqual(sceneApi.parseRadarScene(fault), { blocked: true, reason: "sensor_fault" });
});

test("renders only live and degraded sources", () => {
  for (const status of ["live", "degraded"]) {
    assert.equal(sceneApi.parseRadarScene(makeSnapshot({ status })).blocked, false);
  }
});

test("preserves strict hazard and DANGER consistency rules", async (t) => {
  for (const level of ["SAFE", "FREE", "unknown"]) {
    await t.test(`invalid level ${level}`, () => {
      assert.throws(
        () => sceneApi.parseRadarScene(makeSnapshot({
          scene: { hazard: { level, threshold_m: 0.1, reason: "forged" } },
        })),
        /hazard contract/,
      );
    });
  }
  await t.test("non-finite numeric threshold", () => {
    const snapshot = makeSnapshot();
    snapshot.scene.hazard.threshold_m = "0.1";
    assert.throws(() => sceneApi.parseRadarScene(snapshot), /hazard contract/);
  });
  await t.test("DANGER without a confirmed point", () => {
    const snapshot = makeSnapshot();
    snapshot.scene.tracks = [makeHeatmapTrack({
      classification: "confirmed_obstacle",
      heatmap_confirmed: true,
      forward_m: 0.09,
      lateral_m: 0,
      distance_m: 0.09,
    })];
    snapshot.scene.obstacle_tracks = [...snapshot.scene.tracks];
    snapshot.scene.hazard = { level: "DANGER", threshold_m: 0.1, reason: "forged" };
    assert.throws(() => sceneApi.parseRadarScene(snapshot), /DANGER contract/);
  });
  await t.test("DANGER with a fresh confirmed point", () => {
    const snapshot = makeSnapshot();
    const point = makePointTrack({ forward_m: 0.10, distance_m: 0.10 });
    snapshot.scene.tracks = [point];
    snapshot.scene.obstacle_tracks = [point];
    snapshot.scene.hazard = { level: "DANGER", threshold_m: 0.1, reason: "confirmed" };
    const parsed = sceneApi.parseRadarScene(snapshot);
    assert.equal(parsed.hazard.level, "DANGER");
    assert.equal(parsed.tracks[0].track_id, 1);
  });
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
        elements.set(selector, { dataset: {}, hidden: false, textContent: "" });
      }
      return elements.get(selector);
    },
  };
  const browserGlobal = { document, window: { HanselRadarScene: sceneApi } };
  const source = fs.readFileSync(
    require.resolve("../../monitor/web/radar_panel.js"),
    "utf8",
  );
  vm.runInNewContext(source, browserGlobal);
  const panel = Object.create(browserGlobal.window.HanselRadarPanel.prototype);
  panel.snapshot = {
    axes: { forward_axis: "y", forward_sign: 1, lateral_axis: "x", lateral_sign: 1 },
    frame: null,
    scene: { pose_mode: "motion_compensated", calibration_status: "ok", diagnostics: {} },
    counters: {},
  };
  panel.updateDiagnostics({
    blocked: false,
    poseMode: "robot_relative",
    tracks: [],
    grid: new Uint8Array(3600),
    hazard: { level: "UNKNOWN" },
  });
  assert.equal(elements.get("#pose-mode-value").textContent, "ROBOT RELATIVE");
});
