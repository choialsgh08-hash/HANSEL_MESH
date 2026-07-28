"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
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
  const grid = makeGrid(2, 2).payload;
  const snapshot = {
    status: "live",
    scene: {
      schema_version: 1,
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

test("blocks waiting, stale, fault, and replay-ended sources", () => {
  for (const status of ["waiting", "stale", "fault", "replay_end"]) {
    const parsed = sceneApi.parseRadarScene(makeSnapshot({ status }));
    assert.equal(parsed.blocked, true, status);
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

test("rejects DANGER when only a forged heatmap confirmation is inside threshold", () => {
  const snapshot = makeSnapshot({
    scene: {
      tracks: [{
        track_id: 7,
        age_ms: 0,
        source: "heatmap",
        point_confirmed: true,
        distance_m: 0.09,
      }],
      hazard: { level: "DANGER", threshold_m: 0.1, reason: "confirmed" },
    },
  });

  assert.throws(() => sceneApi.parseRadarScene(snapshot), /DANGER contract/);
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
