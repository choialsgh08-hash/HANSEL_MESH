"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

const PANEL_PATH = require.resolve("../../monitor/web/radar_panel.js");
const FRONT_PATH = require.resolve("../../monitor/web/radar_front.html");
const BLOCKED_CASES = [
  [
    "waiting",
    "RADAR STARTING · DRIVE STOP",
    "레이더 준비 중 · 주행을 정지하세요",
  ],
  [
    "stale",
    "RADAR RECONNECTING · DRIVE STOP",
    "레이더 재연결 중 · 주행을 정지하세요",
  ],
  [
    "fault",
    "RADAR RECONNECTING · DRIVE STOP",
    "레이더 재연결 중 · 주행을 정지하세요",
  ],
  [
    "sensor_fault",
    "RADAR RECONNECTING · DRIVE STOP",
    "레이더 재연결 중 · 주행을 정지하세요",
  ],
  [
    "http_lost",
    "RADAR RECONNECTING · DRIVE STOP",
    "레이더 재연결 중 · 주행을 정지하세요",
  ],
  [
    "replay_end",
    "REPLAY END",
    "마지막 프레임을 지도처럼 사용하지 마세요",
  ],
  [
    "calibration_required",
    "CALIBRATION REQUIRED",
    "빈 장면 self-clutter 보정 파일이 필요합니다",
  ],
  [
    "calibration_unavailable",
    "CALIBRATION UNAVAILABLE",
    "보정되지 않은 반사를 점유로 표시하지 않습니다",
  ],
  [
    "profile_mismatch",
    "PROFILE MISMATCH",
    "보정 파일과 현재 레이더 프로필이 다릅니다",
  ],
  [
    "invalid_scene",
    "SCENE CONTRACT ERROR",
    "검증되지 않은 장면은 표시하지 않습니다",
  ],
];

function makeElement(children = {}) {
  return {
    dataset: {},
    hidden: false,
    textContent: "",
    checked: false,
    addEventListener() {},
    querySelector(selector) {
      return children[selector] || null;
    },
  };
}

function makeMetric() {
  return makeElement({
    strong: makeElement(),
    span: makeElement(),
  });
}

function makeContext() {
  let currentPath = [];
  let globalAlpha = 1;
  const context = {
    clearRectCalls: [],
    fillRectCalls: [],
    fillRectStyles: [],
    fillTextCalls: [],
    fillTextStyles: [],
    strokeRectCalls: [],
    strokeRectStyles: [],
    arcCalls: [],
    ellipseCalls: [],
    strokeCalls: [],
    fillCalls: [],
    globalAlphaValues: [],
    shadowBlur: 0,
    setTransform() {},
    save() {},
    restore() {},
    closePath() {},
    rect() {},
    clip() {},
    translate() {},
    setLineDash() {},
    beginPath() {
      currentPath = [];
    },
    arc(...args) {
      this.arcCalls.push(args);
      currentPath.push(["arc", ...args]);
    },
    ellipse(...args) {
      this.ellipseCalls.push(args);
      currentPath.push(["ellipse", ...args]);
    },
    fill() {
      this.fillCalls.push({
        fillStyle: this.fillStyle,
        path: currentPath.map((command) => [...command]),
      });
    },
    stroke() {
      this.strokeCalls.push({
        strokeStyle: this.strokeStyle,
        lineWidth: this.lineWidth,
        path: currentPath.map((command) => [...command]),
      });
    },
    moveTo(...args) {
      currentPath.push(["moveTo", ...args]);
    },
    lineTo(...args) {
      currentPath.push(["lineTo", ...args]);
    },
    measureText(text) {
      return { width: String(text).length * 7 };
    },
    clearRect(...args) {
      this.clearRectCalls.push(args);
    },
    fillRect(...args) {
      this.fillRectCalls.push(args);
      this.fillRectStyles.push({
        fillStyle: this.fillStyle,
        globalAlpha: this.globalAlpha,
        args,
      });
    },
    strokeRect(...args) {
      this.strokeRectCalls.push(args);
      this.strokeRectStyles.push(this.strokeStyle);
    },
    fillText(text, ...args) {
      this.fillTextCalls.push([text, ...args]);
      this.fillTextStyles.push(this.fillStyle);
    },
  };
  Object.defineProperty(context, "globalAlpha", {
    get() {
      return globalAlpha;
    },
    set(value) {
      globalAlpha = value;
      if (typeof value === "number") {
        context.globalAlphaValues.push(value);
      }
    },
  });
  return context;
}

function makeCanvas(context) {
  return {
    width: 0,
    height: 0,
    getContext() {
      return context;
    },
    addEventListener() {},
    setPointerCapture() {},
    getBoundingClientRect() {
      return { width: 640, height: 360 };
    },
  };
}

function makeHarness() {
  const mainContext = makeContext();
  const collisionContext = makeContext();
  const pointCloudContext = makeContext();
  const mainCanvas = makeCanvas(mainContext);
  const collisionCanvas = makeCanvas(collisionContext);
  const pointCloudCanvas = makeCanvas(pointCloudContext);
  const elements = new Map();
  const metrics = [
    "metric-hazard",
    "metric-nearest",
    "metric-fps",
    "metric-age",
  ];
  for (const id of metrics) {
    elements.set(`#${id}`, makeMetric());
  }
  for (const id of [
    "fullscreen-button",
    "raw-toggle",
    "radar-status",
    "warning-strip",
    "warning-text",
    "radar-mode",
    "collision-inset",
    "collision-distance",
    "pointcloud-3d-status",
    "axis-value",
    "profile-value",
    "calibration-value",
    "pose-mode-value",
    "frame-value",
    "gap-value",
    "raw-point-value",
    "ground-rejected-value",
    "reflectivity-cell-value",
    "ground-fringe-rejected-value",
    "confirmed-track-value",
    "clutter-rejected-value",
    "heatmap-candidate-value",
    "heatmap-cluster-value",
    "reflection-value",
    "quality-rejected-value",
    "probability-active-value",
    "probability-occupied-value",
    "probability-decayed-value",
    "probability-expired-value",
    "path-pending-value",
    "path-accepted-value",
    "obstacle-memory-value",
    "stable-cell-value",
    "motion-status-value",
    "path-status-value",
    "path-confidence-value",
    "grid-status-value",
    "scene-hazard-value",
    "axis-warning",
    "ground-filter-tag",
  ]) {
    if (!elements.has(`#${id}`)) {
      elements.set(`#${id}`, makeElement());
    }
  }
  for (let index = 0; index < 5; index += 1) {
    elements.set(
      `#sector-${index}`,
      makeElement({ b: makeElement(), span: makeElement() }),
    );
  }

  const document = {
    fullscreenElement: null,
    addEventListener() {},
    querySelector(selector) {
      return elements.get(selector) || null;
    },
  };
  const root = {
    addEventListener() {},
    querySelector(selector) {
      if (selector === "#radar-main-canvas") return mainCanvas;
      if (selector === "#collision-canvas") return collisionCanvas;
      if (selector === "#pointcloud-3d-canvas") return pointCloudCanvas;
      return null;
    },
  };
  const window = {
    HanselRadarScene: {
      makeMapTransform() {
        return { scale: 1 };
      },
      projectMapPoint(transform, forwardM, lateralM) {
        return {
          x: transform.originX + lateralM * transform.scale,
          y: transform.originY - forwardM * transform.scale,
        };
      },
    },
    addEventListener() {},
    devicePixelRatio: 1,
    location: { reload() {} },
  };
  window.window = window;
  const context = vm.createContext({
    window,
    document,
    console,
    Number,
    Math,
    String,
    Boolean,
    Array,
    Object,
    Infinity,
  });
  vm.runInContext(fs.readFileSync(PANEL_PATH, "utf8"), context, {
    filename: PANEL_PATH,
  });

  const panel = new window.HanselRadarPanel(root);
  panel.snapshot = {
    status: "live",
    warning: null,
    fps: 30,
    age_ms: 5,
    axes: {
      forward_axis: "y",
      forward_sign: 1,
      lateral_axis: "x",
      lateral_sign: 1,
    },
    frame: {
      number: 41,
      display_point_count: 1,
      source_point_count: 1,
      profile_id: "stale-profile",
    },
    counters: {
      frame_gaps_total: 0,
      sensor_sequence_gaps_total: 0,
      writer_drops_total: 0,
    },
    scene: {
      calibration_status: "ok",
      hazard: { level: "NORMAL", threshold_m: 0.1 },
      diagnostics: {
        scene_point_count: 1,
        clutter_points_rejected: 0,
        heatmap_cells_rejected: 0,
      },
      tracks: [{
        source: "point",
        point_confirmed: true,
        distance_m: 0.05,
        forward_m: 0.05,
        lateral_m: 0,
      }],
    },
  };
  for (const drawContext of [
    mainContext,
    collisionContext,
    pointCloudContext,
  ]) {
    for (const key of [
      "clearRectCalls",
      "fillRectCalls",
      "fillRectStyles",
      "fillTextCalls",
      "fillTextStyles",
      "strokeRectCalls",
      "strokeRectStyles",
      "arcCalls",
      "ellipseCalls",
      "strokeCalls",
      "fillCalls",
      "globalAlphaValues",
    ]) {
      drawContext[key].length = 0;
    }
  }
  return {
    panel,
    elements,
    contexts: [mainContext, collisionContext, pointCloudContext],
  };
}

function livePresentationWithProbabilityMap() {
  return {
    blocked: false,
    reason: null,
    hazard: { level: "NORMAL", threshold_m: 0.10 },
    poseMode: "robot_relative_short_memory",
    groundFilter: {
      sensorHeightM: 0.065,
      sensorPitchDeg: 0.0,
      groundRejectionHeightM: 0.035,
      filterRangeM: 0.60,
      pitchConvention: "positive_up",
      status: "configured",
    },
    reflectivitySurface: {
      radiusM: 0.50,
      resolutionM: 0.025,
      historyFrames: 5,
      requiredHits: 3,
      retention: "three_of_five_complete_frames",
      poseMode: "sensor_relative_short_memory",
      source: "calibrated_heatmap_residual",
      cells: [],
    },
    probabilisticLocalMap: {
      sourceField: "probabilistic_local_map",
      radiusM: 0.50,
      resolutionM: 0.05,
      occupiedThreshold: 0.75,
      cells: [],
    },
    grid: { cells: [] },
    rollingMapObstacles: [
      { forward_m: 0.25, lateral_m: 0.05, confidence: 0.8, age_ms: 0 },
    ],
    travelTrail: { points: [] },
    plannedPath: {
      status: "clear",
      stability_status: "stable",
      length_m: 0.01,
      points: [[0.00, 0.00], [0.01, 0.00]],
      minimum_passage_width_m: 0.21,
      blocked_previous_path: null,
    },
    referenceTracks: [],
    tracks: [],
    filteredTracks: [],
    diagnostics: {},
  };
}

test("main rolling-map transform keeps the radar at circle center", () => {
  const harness = makeHarness();
  const transform = harness.panel.makeCircularViewportTransform(
    600,
    500,
    0.5,
    { top: 50, right: 25, bottom: 50, left: 25 },
  );

  assert.equal(transform.originX, 300);
  assert.equal(transform.originY, 250);
  assert.equal(transform.scale, 400);
  assert.equal(transform.centeredCircle, true);
});

test("2.5d point cloud uses filtered height and filters beyond fifty centimeters", () => {
  const { panel, contexts } = makeHarness();
  const context = contexts[2];

  panel.drawPointCloud25d(context, 360, 260, [
    { source: "point", classification: "confirmed_obstacle",
      point_confirmed: true, forward_m: 0.30, lateral_m: 0.10,
      distance_m: 0.316, height_m: 0.10, height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
    { source: "point", classification: "confirmed_obstacle",
      point_confirmed: true, forward_m: 0.60, lateral_m: 0,
      distance_m: 0.60, height_m: 0, height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
  ], [], false);

  assert.ok(context.fillTextCalls.some(
    (entry) => entry[0] === "FILTERED 1 · RAW OFF · Z SENSOR-RELATIVE",
  ));
});

test("probability cells fade as probability approaches unknown", () => {
  const { panel, contexts } = makeHarness();
  const context = contexts[0];
  const transform = panel.makeCircularViewportTransform(
    500, 500, 0.5, { top: 0, right: 0, bottom: 0, left: 0 },
  );
  panel.drawProbabilisticLocalMap(context, transform, {
    resolutionM: 0.05,
    cells: [
      { forward_m: 0.20, lateral_m: 0, probability: 0.80, occupied: true },
      { forward_m: 0.30, lateral_m: 0, probability: 0.60, occupied: false },
    ],
  }, { clipShape: "circle" });
  const alphas = context.globalAlphaValues.filter(
    (value) => value > 0 && value < 1,
  ).slice(-2);
  assert.ok(alphas[0] > alphas[1]);
  assert.deepEqual(context.fillRectCalls[0], [237.5, 137.5, 25, 25]);
  assert.deepEqual(
    context.fillRectStyles.slice(-2).map((call) => call.fillStyle),
    ["rgb(255, 126, 54)", "rgb(92, 228, 208)"],
  );
  assert.equal(context.globalAlphaValues.at(-1), 1);
});

test("probability evidence is nearly invisible as it reaches unknown", () => {
  const { panel, contexts } = makeHarness();
  const transform = panel.makeCircularViewportTransform(
    500, 500, 0.5, { top: 0, right: 0, bottom: 0, left: 0 },
  );
  panel.drawProbabilisticLocalMap(contexts[0], transform, {
    resolutionM: 0.05,
    cells: [{
      forward_m: 0.20, lateral_m: 0, probability: 0.5001, occupied: false,
    }],
  }, { clipShape: "circle" });
  assert.ok(contexts[0].globalAlphaValues.some(
    (value) => value > 0 && value < 0.01,
  ));
});

test("new probability map suppresses both legacy layers in both map views", () => {
  const { panel } = makeHarness();
  let legacyGridCalls = 0;
  let legacyCircleCalls = 0;
  panel.drawEvidenceGrid = () => { legacyGridCalls += 1; };
  panel.drawAccumulatedObstacles = () => { legacyCircleCalls += 1; };
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  const presentation = livePresentationWithProbabilityMap();
  panel.drawLidarTopView(makeContext(), transform, presentation);
  panel.drawCollisionInset(makeContext(), transform, presentation);
  assert.equal(legacyGridCalls, 0);
  assert.equal(legacyCircleCalls, 0);
});

test("null probability map preserves both legacy layers in both map views", () => {
  const { panel } = makeHarness();
  let legacyGridCalls = 0;
  let legacyCircleCalls = 0;
  panel.drawEvidenceGrid = () => { legacyGridCalls += 1; };
  panel.drawAccumulatedObstacles = () => { legacyCircleCalls += 1; };
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  const presentation = livePresentationWithProbabilityMap();
  presentation.probabilisticLocalMap = null;
  panel.drawLidarTopView(makeContext(), transform, presentation);
  panel.drawCollisionInset(makeContext(), transform, presentation);
  assert.equal(legacyGridCalls, 2);
  assert.equal(legacyCircleCalls, 2);
});

test("both map views invoke reflectivity probability and robot-footprint renderers", () => {
  const { panel, contexts } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  let probabilityCalls = 0;
  let reflectivityCalls = 0;
  let footprintCalls = 0;
  panel.drawReflectivitySurface = () => { reflectivityCalls += 1; };
  panel.drawProbabilisticLocalMap = () => { probabilityCalls += 1; };
  panel.drawRobotFootprint = () => { footprintCalls += 1; };
  panel.drawLidarTopView(contexts[0], transform, presentation);
  panel.drawCollisionInset(contexts[1], transform, presentation);
  assert.equal(reflectivityCalls, 2);
  assert.equal(probabilityCalls, 2);
  assert.equal(footprintCalls, 2);
});

test("both map views draw reflectivity then probability before every track layer", () => {
  const { panel, contexts } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  for (const [context, render] of [
    [contexts[0], () => panel.drawLidarTopView(contexts[0], transform, presentation)],
    [contexts[1], () => panel.drawCollisionInset(contexts[1], transform, presentation)],
  ]) {
    const order = [];
    panel.drawReflectivitySurface = () => { order.push("reflectivity"); };
    panel.drawProbabilisticLocalMap = () => { order.push("probability"); };
    panel.drawTracks = () => { order.push("tracks"); };
    render();
    assert.deepEqual(order.slice(0, 2), ["reflectivity", "probability"]);
    assert.ok(order.includes("tracks"));
    assert.ok(order.slice(2).every((entry) => entry === "tracks"));
    assert.ok(context);
  }
});

test("reflectivity surface fills cells and outlines only exposed edges", () => {
  const { panel, contexts } = makeHarness();
  const context = contexts[0];
  const transform = {
    originX: 250, originY: 250, scale: 400,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  panel.drawReflectivitySurface(context, transform, {
    resolutionM: 0.025,
    cells: [
      { forward_m: 0.20, lateral_m: 0.0, intensity: 0.9, persistence: 1.0 },
      { forward_m: 0.20, lateral_m: 0.025, intensity: 0.8, persistence: 0.8 },
      { forward_m: 0.30, lateral_m: 0.10, intensity: 0.7, persistence: 0.6 },
    ],
  }, { clipShape: "circle" });

  assert.equal(context.fillRectCalls.length, 3);
  assert.deepEqual(context.fillRectCalls[0], [245, 165, 10, 10]);
  const boundarySegments = context.strokeCalls.flatMap(
    (call) => call.path.filter((command) => command[0] === "lineTo"),
  );
  assert.equal(boundarySegments.length, 10);
  assert.ok(context.fillRectStyles.every(
    (call) => call.fillStyle === "rgb(224, 232, 231)" &&
      call.globalAlpha > 0 && call.globalAlpha < 1,
  ));
  assert.equal(context.shadowBlur, 0);
  assert.equal(context.globalAlphaValues.at(-1), 1);
});

test("display-only reflectivity cells never enter track drawing", () => {
  const { panel, contexts } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  presentation.reflectivitySurface.cells = [{
    forward_m: 0.40,
    lateral_m: 0.05,
    intensity: 0.9,
    persistence: 1.0,
    hits: 5,
    age_ms: 0,
  }];
  const transform = {
    originX: 250, originY: 250, scale: 500,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  const trackInputs = [];
  panel.drawTracks = (_ctx, _transform, tracks) => { trackInputs.push(tracks); };

  panel.drawLidarTopView(contexts[0], transform, presentation);
  panel.drawCollisionInset(contexts[1], transform, presentation);

  assert.ok(trackInputs.length > 0);
  assert.ok(trackInputs.every((tracks) => Array.isArray(tracks) && tracks.length === 0));
});

test("live point tracks render their distance label without a rendering error", () => {
  const { panel, contexts } = makeHarness();
  const context = contexts[0];
  const transform = {
    originX: 320,
    originY: 340,
    scale: 100,
    width: 640,
    height: 360,
    forwardMaxM: 3,
    halfWidthM: 1.5,
  };
  const track = {
    source: "point",
    point_confirmed: true,
    age_ms: 0,
    distance_m: 0.25,
    forward_m: 0.25,
    lateral_m: 0,
    height_m: 0.1,
  };

  assert.doesNotThrow(() => {
    panel.drawTracks(context, transform, [track], {
      scene: {
        hazard: {
          level: "NORMAL",
          threshold_m: 0.1,
        },
      },
      labelLimit: 1,
      maxRangeM: 3,
      clipShape: "rectangular",
    });
  });
  assert.equal(
    context.fillTextCalls.some(([text]) => String(text).includes("0.25m")),
    true,
  );
});

test("2.5d view renders filtered height bands and unknown heatmap height", () => {
  const { panel, contexts } = makeHarness();
  const context = contexts[2];
  panel.drawPointCloud25d(context, 420, 300, [
    { source: "point", classification: "confirmed_obstacle", forward_m: 0.30,
      lateral_m: 0.05, distance_m: 0.304, height_m: 0.12,
      height_band: "above_sensor",
      height_reference: "sensor_relative_uncalibrated",
      confidence: 0.8, age_ms: 0 },
    { source: "heatmap", classification: "confirmed_obstacle",
      heatmap_confirmed: true, forward_m: 0.35,
      lateral_m: -0.05, distance_m: 0.354, height_m: null,
      height_band: "unknown", height_reference: "unavailable",
      confidence: 0.5, age_ms: 0 },
  ], [], false);
  assert.ok(context.fillTextCalls.some(([text]) => String(text).includes("FILTERED 2")));
  assert.ok(context.fillTextCalls.some(([text]) => String(text).includes("Z UNKNOWN")));
  assert.ok(context.fillTextCalls.some(([text]) => String(text).includes("0.30m")));
});

test("full draw never evaluates frame points while raw debug is off", () => {
  const { panel } = makeHarness();
  panel.presentation = livePresentationWithProbabilityMap();
  panel.snapshot = { frame: {} };
  Object.defineProperty(panel.snapshot.frame, "points", {
    get() { throw new Error("raw accessed"); },
  });
  panel.rawToggle.checked = false;
  assert.doesNotThrow(() => panel.draw());
});

test("raw debug reads points only when checked without changing guidance", () => {
  const { panel } = makeHarness();
  panel.presentation = livePresentationWithProbabilityMap();
  const before = JSON.stringify(panel.presentation);
  let reads = 0;
  panel.snapshot = { frame: {} };
  Object.defineProperty(panel.snapshot.frame, "points", {
    get() { reads += 1; return []; },
  });
  panel.rawToggle.checked = true;
  panel.draw();
  assert.equal(reads, 1);
  assert.equal(JSON.stringify(panel.presentation), before);
});

test("robot footprint uses fifteen centimeters plus three centimeter margins", () => {
  const { panel, contexts } = makeHarness();
  panel.drawRobotFootprint(contexts[0], { originX: 250, originY: 250, scale: 400 });
  const radii = contexts[0].arcCalls.map((args) => args[2]);
  assert.ok(radii.some((radius) => Math.abs(radius - 30) < 0.001));
  assert.ok(radii.some((radius) => Math.abs(radius - 42) < 0.001));
});

test("range and lateral uncertainty render as a thin metric outline", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    forwardMaxM: 0.5, halfWidthM: 0.5,
  };
  panel.drawUncertaintyOutline(contexts[0], transform, {
    forward_m: 0.30, lateral_m: 0.10,
    range_uncertainty_m: 0.05, lateral_uncertainty_m: 0.08,
  });
  const [x, y, lateralRadius, rangeRadius, rotation] =
    contexts[0].ellipseCalls[0];
  assert.deepEqual([x, y], [290, 130]);
  assert.equal(lateralRadius, 32);
  assert.equal(rangeRadius, 20);
  assert.ok(Math.abs(rotation - Math.atan2(0.10, 0.30)) < 1e-9);
  assert.equal(contexts[0].fillCalls.length, 0);
  assert.equal(contexts[0].shadowBlur, 0);
  assert.ok(contexts[0].strokeCalls.some(
    (call) => call.strokeStyle === "rgba(223, 252, 255, 0.35)" &&
      call.lineWidth === 1,
  ));
});

test("normal map rendering invokes uncertainty outlines for map and track", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    width: 500, height: 500, forwardMaxM: 0.5,
    halfWidthM: 0.5, centeredCircle: true,
  };
  const presentation = livePresentationWithProbabilityMap();
  presentation.probabilisticLocalMap.cells = [{
    forward_m: 0.20, lateral_m: 0.0, probability: 0.8, occupied: true,
    range_uncertainty_m: 0.05, lateral_uncertainty_m: 0.06,
    source: "point",
  }];
  presentation.filteredTracks = [{
    forward_m: 0.30, lateral_m: 0.02,
    range_uncertainty_m: 0.04, lateral_uncertainty_m: 0.07,
    source: "heatmap", heatmap_confirmed: true, age_ms: 0,
  }];
  const outlined = [];
  panel.drawUncertaintyOutline = (_ctx, _transform, evidence) => {
    outlined.push(evidence.source);
  };
  panel.drawLidarTopView(contexts[0], transform, presentation);
  assert.deepEqual(outlined.sort(), ["heatmap", "point"]);
});

test("planned path underlay uses the physical minimum passage width", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    forwardMaxM: 0.5, halfWidthM: 0.5,
  };
  panel.drawPlannedPath(contexts[0], transform, {
    points: [[0.0, 0.0], [0.10, 0.0], [0.20, 0.0]],
    minimum_passage_width_m: 0.21,
    blocked_previous_path: null,
  });
  assert.ok(contexts[0].strokeCalls.some(
    (call) => Math.abs(call.lineWidth - 84) < 0.001,
  ));
});

test("blocked path segments use original inclusive point indices", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    forwardMaxM: 0.5, halfWidthM: 0.5,
  };
  panel.drawPlannedPath(contexts[0], transform, {
    points: [[0.0, 0.0], [0.10, -0.02]],
    minimum_passage_width_m: 0.21,
    blocked_previous_path: {
      points: [[-0.01, 0.0], [0.10, 0.0], [0.20, 0.05], [0.30, 0.05]],
      segments: [{ start_index: 1, end_index: 2, reason: "confirmed_obstacle" }],
    },
  });
  const blocked = contexts[0].strokeCalls.find(
    (call) => call.strokeStyle === "rgba(255, 81, 81, 0.92)",
  );
  assert.deepEqual(blocked.path, [
    ["moveTo", 250, 210], ["lineTo", 270, 170],
  ]);
});

test("singleton blocked path segment renders a visible red point", () => {
  const { panel, contexts } = makeHarness();
  const transform = {
    originX: 250, originY: 250, scale: 400,
    forwardMaxM: 0.5, halfWidthM: 0.5,
  };
  panel.drawPlannedPath(contexts[0], transform, {
    points: [[0.0, 0.0], [0.10, 0.0]],
    minimum_passage_width_m: 0.21,
    blocked_previous_path: {
      points: [[0.10, 0.0], [0.20, 0.05], [0.30, 0.05]],
      segments: [{ start_index: 1, end_index: 1, reason: "confirmed_obstacle" }],
    },
  });
  const blockedPoint = contexts[0].fillCalls.find(
    (call) => call.fillStyle === "rgba(255, 81, 81, 0.92)",
  );
  assert.ok(blockedPoint, "singleton blocked segment must remain visible");
  assert.deepEqual(blockedPoint.path, [
    ["arc", 270, 170, 3.2, 0, Math.PI * 2],
  ]);
});

test("collision inset includes fifty centimeters and excludes fifty-one", () => {
  const { panel, contexts } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  presentation.tracks = [
    { source: "point", point_confirmed: true, classification: "confirmed_obstacle",
      age_ms: 0, distance_m: 0.50, forward_m: 0.50, lateral_m: 0,
      height_m: 0, height_band: "sensor_level" },
    { source: "point", point_confirmed: true, classification: "confirmed_obstacle",
      age_ms: 0, distance_m: 0.51, forward_m: 0.51, lateral_m: 0,
      height_m: 0, height_band: "sensor_level" },
  ];
  panel.presentation = presentation;
  panel.draw();
  const labels = contexts[1].fillTextCalls.map(([text]) => String(text));
  assert.ok(labels.includes("P 50cm"));
  assert.equal(labels.includes("P 51cm"), false);
});

test("collision nearest distance excludes evidence beyond fifty centimeters", () => {
  const { panel, elements } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  presentation.tracks = [{
    source: "point", point_confirmed: true, classification: "confirmed_obstacle",
    age_ms: 0, distance_m: 0.51, forward_m: 0.51, lateral_m: 0,
    height_m: 0, height_band: "sensor_level",
  }];
  panel.updateText(presentation);
  assert.equal(elements.get("#collision-distance").textContent, "-- cm");

  presentation.tracks[0].distance_m = 0.50;
  presentation.tracks[0].forward_m = 0.50;
  panel.updateText(presentation);
  assert.equal(elements.get("#collision-distance").textContent, "50 cm");
});

test("2.5d view excludes tracks beyond fifty centimeters or sixty degrees", () => {
  const { panel, contexts } = makeHarness();
  panel.drawPointCloud25d(contexts[2], 420, 300, [
    { source: "point", classification: "confirmed_obstacle",
      forward_m: 0.51, lateral_m: 0, distance_m: 0.51, height_m: 0,
      height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
    { source: "point", classification: "confirmed_obstacle",
      forward_m: 0.10, lateral_m: 0.30, distance_m: 0.316, height_m: 0,
      height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
  ], [], false);
  assert.ok(contexts[2].fillTextCalls.some(
    ([text]) => String(text).includes("FILTERED 0"),
  ));
});

test("2.5d markers distinguish confirmed tentative heatmap and raw geometry", () => {
  const { panel, contexts } = makeHarness();
  const tracks = [
    { source: "point", classification: "confirmed_obstacle",
      point_confirmed: true, forward_m: 0.20, lateral_m: -0.04,
      distance_m: 0.204, height_m: 0.0, height_band: "sensor_level",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.9, age_ms: 0 },
    { source: "point", classification: "point_candidate",
      point_confirmed: false, forward_m: 0.30, lateral_m: 0.04,
      distance_m: 0.303, height_m: 0.12, height_band: "above_sensor",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.6, age_ms: 0 },
    { source: "heatmap", classification: "confirmed_obstacle",
      heatmap_confirmed: true, forward_m: 0.40, lateral_m: 0.0,
      distance_m: 0.40, height_m: null, height_band: "unknown",
      height_reference: "unavailable", confidence: 0.5, age_ms: 0 },
    { source: "point", classification: "confirmed_obstacle",
      point_confirmed: true, forward_m: 0.25, lateral_m: 0.0,
      distance_m: 0.25, height_m: -0.12, height_band: "below_sensor",
      height_reference: "sensor_relative_uncalibrated", confidence: 0.8,
      age_ms: 0 },
  ];
  panel.drawPointCloud25d(
    contexts[2], 420, 300, tracks,
    [[0.25, 0.0, 0.0, 0.0, 24.0, 70.0]], true,
  );
  assert.ok(contexts[2].fillCalls.some(
    (call) => call.fillStyle === "#dffcff",
  ));
  assert.ok(contexts[2].fillCalls.some(
    (call) => call.fillStyle === "#5bbcff",
  ));
  assert.ok(contexts[2].strokeCalls.some(
    (call) => call.strokeStyle === "#ffb84d",
  ));
  assert.ok(contexts[2].fillCalls.some(
    (call) => call.fillStyle === "#a98cff" &&
      call.path.filter(([command]) => command === "lineTo").length >= 3,
  ));
  assert.ok(contexts[2].strokeCalls.some(
    (call) => call.strokeStyle === "rgba(170, 180, 180, 0.55)",
  ));
  assert.ok(contexts[2].fillTextCalls.some(
    ([text]) => String(text).includes("RAW DEBUG 1"),
  ));
  assert.equal(contexts[2].shadowBlur, 0);
  for (const label of ["10cm", "30cm", "50cm"]) {
    assert.ok(contexts[2].fillTextCalls.some(
      ([text]) => String(text).includes(label),
    ));
  }
});

test("2.5d labels only the nearest five points with signed z", () => {
  const { panel, contexts } = makeHarness();
  const tracks = Array.from({ length: 6 }, (_, index) => ({
    source: "point", classification: "confirmed_obstacle",
    point_confirmed: true, forward_m: 0.10 + index * 0.05, lateral_m: 0.0,
    distance_m: 0.10 + index * 0.05, height_m: (index + 1) * 0.01,
    height_band: "sensor_level",
    height_reference: "sensor_relative_uncalibrated",
    confidence: 0.9, age_ms: 0,
  }));
  panel.drawPointCloud25d(contexts[2], 420, 300, tracks, [], false);
  const metricLabels = contexts[2].fillTextCalls
    .map(([value]) => String(value))
    .filter((value) => /^0\.\d{2}m · Z [+-]0\.\d{2}m$/.test(value));
  assert.equal(metricLabels.length, 5);
  assert.ok(metricLabels.includes("0.10m · Z +0.01m"));
  assert.equal(metricLabels.includes("0.35m · Z +0.06m"), false);
});

test("2.5d raw marker is absent when raw debug is off", () => {
  const { panel, contexts } = makeHarness();
  panel.drawPointCloud25d(
    contexts[2], 420, 300, [],
    [[0.25, 0.0, 0.0, 0.0, 24.0, 70.0]], false,
  );
  assert.equal(contexts[2].strokeCalls.some(
    (call) => call.strokeStyle === "rgba(170, 180, 180, 0.55)"), false,
  );
});

test("validated diagnostics and blocked-near-field status drive existing rows", () => {
  const { panel, elements } = makeHarness();
  const presentation = livePresentationWithProbabilityMap();
  presentation.diagnostics = {
    raw_point_count: 7,
    ground_rejected_count: 13,
    ground_fringe_rejected_count: 6,
    reflectivity_surface_cell_count: 11,
    self_clutter_rejected_count: 2,
    quality_gate_rejected_count: 1,
    tentative_point_track_count: 3,
    confirmed_point_track_count: 4,
    heatmap_reflection_count: 5,
    confirmed_heatmap_track_count: 6,
    probability_active_cell_count: 8,
    probability_occupied_cell_count: 2,
    probability_decayed_cell_count: 9,
    probability_ttl_expired_cell_count: 10,
    path_switch_pending_count: 1,
    path_switch_accepted_count: 12,
  };
  presentation.plannedPath = {
    ...presentation.plannedPath,
    status: "blocked_near_field",
    stability_status: "route_changed_blocked",
    length_m: 0,
    points: [[0, 0]],
  };
  panel.updateDiagnostics(presentation);
  assert.equal(elements.get("#raw-point-value").textContent, "7");
  assert.equal(elements.get("#ground-rejected-value").textContent, "13");
  assert.equal(elements.get("#ground-fringe-rejected-value").textContent, "6");
  assert.equal(elements.get("#reflectivity-cell-value").textContent, "11");
  assert.equal(
    elements.get("#ground-filter-tag").textContent,
    "SENSOR 6.5cm / PITCH 0.0deg / GROUND HIDDEN",
  );
  assert.equal(elements.get("#clutter-rejected-value").textContent, "2");
  assert.equal(elements.get("#reflection-value").textContent, "5");
  assert.equal(elements.get("#confirmed-track-value").textContent, "10");
  assert.equal(elements.get("#quality-rejected-value").textContent, "1");
  assert.equal(elements.get("#probability-active-value").textContent, "8");
  assert.equal(elements.get("#probability-occupied-value").textContent, "2");
  assert.equal(elements.get("#probability-decayed-value").textContent, "9");
  assert.equal(elements.get("#probability-expired-value").textContent, "10");
  assert.equal(elements.get("#path-pending-value").textContent, "1");
  assert.equal(elements.get("#path-accepted-value").textContent, "12");
  assert.match(
    elements.get("#path-status-value").textContent,
    /BLOCKED_NEAR_FIELD.*ROUTE_CHANGED_BLOCKED/,
  );
});

function staleBlockedPresentation(reason) {
  return {
    blocked: true,
    reason,
    hazard: { level: "NORMAL", threshold_m: 0.1 },
    tracks: [{
      source: "point",
      point_confirmed: true,
      distance_m: 0.05,
      forward_m: 0.05,
      lateral_m: 0,
    }],
  };
}

test("every blocked reason clears stale maps and renders only its stop overlay", async (t) => {
  for (const [reason, heading, explanation] of BLOCKED_CASES) {
    await t.test(reason, () => {
      const { panel, contexts } = makeHarness();
      panel.presentation = staleBlockedPresentation(reason);
      let lidarDraws = 0;
      let collisionDraws = 0;
      const overlayCalls = [];
      const realOverlay = panel.drawBlockingOverlay;
      panel.drawLidarTopView = () => {
        lidarDraws += 1;
      };
      panel.drawCollisionInset = () => {
        collisionDraws += 1;
      };
      panel.drawBlockingOverlay = function (...args) {
        overlayCalls.push(args);
        return realOverlay.apply(this, args);
      };

      panel.draw();

      assert.equal(lidarDraws, 0, `${reason}: stale LiDAR scene rendered`);
      assert.equal(collisionDraws, 0, `${reason}: stale collision scene rendered`);
      assert.equal(overlayCalls.length, 3, `${reason}: overlay count`);
      assert.deepEqual(
        contexts.map((context) => context.clearRectCalls.length),
        [1, 1, 1],
        `${reason}: canvas clear count`,
      );
      for (const context of contexts) {
        assert.deepEqual(
          context.fillTextCalls.map(([text]) => text),
          [heading, explanation],
          `${reason}: reason-specific overlay copy`,
        );
        assert.deepEqual(
          context.strokeRectStyles,
          ["rgba(255, 81, 81, 0.92)"],
          `${reason}: red stop border`,
        );
        assert.equal(
          context.fillTextStyles[0],
          "#ff5151",
          `${reason}: red stop heading`,
        );
      }
    });
  }
});

test("every blocked reason replaces stale live metrics and sectors with sensor fault", async (t) => {
  for (const [reason] of BLOCKED_CASES) {
    await t.test(reason, () => {
      const { panel, elements } = makeHarness();
      const presentation = staleBlockedPresentation(reason);

      panel.updateText(presentation);
      panel.updateDiagnostics(presentation);
      panel.updateSectors(presentation);

      assert.equal(elements.get("#radar-mode").textContent, "MAP BLOCKED", reason);
      assert.equal(
        elements.get("#metric-nearest").querySelector("strong").textContent,
        "--",
        `${reason}: nearest distance`,
      );
      assert.equal(
        elements.get("#metric-hazard").querySelector("strong").textContent,
        "SENSOR_FAULT",
        `${reason}: hazard metric`,
      );
      assert.equal(
        elements.get("#metric-hazard").dataset.hazard,
        "SENSOR_FAULT",
        `${reason}: hazard metric style`,
      );
      assert.equal(
        elements.get("#collision-inset").dataset.hazard,
        "SENSOR_FAULT",
        `${reason}: collision hazard`,
      );
      assert.equal(
        elements.get("#scene-hazard-value").textContent,
        "SENSOR_FAULT",
        `${reason}: diagnostic hazard`,
      );
      assert.equal(
        elements.get("#radar-status").textContent === "LIVE",
        false,
        `${reason}: blocked badge`,
      );
      assert.equal(
        elements.get("#metric-fps").querySelector("strong").textContent,
        "--",
        `${reason}: fps`,
      );
      assert.equal(
        elements.get("#metric-age").querySelector("strong").textContent,
        "--",
        `${reason}: age`,
      );
      assert.equal(
        elements.get("#collision-distance").textContent,
        "-- cm",
        `${reason}: collision distance`,
      );
      for (const selector of [
        "#profile-value",
        "#calibration-value",
        "#pose-mode-value",
        "#frame-value",
        "#gap-value",
        "#raw-point-value",
        "#ground-rejected-value",
        "#confirmed-track-value",
        "#clutter-rejected-value",
        "#heatmap-candidate-value",
        "#heatmap-cluster-value",
        "#reflection-value",
        "#quality-rejected-value",
        "#probability-active-value",
        "#probability-occupied-value",
        "#probability-decayed-value",
        "#probability-expired-value",
        "#path-pending-value",
        "#path-accepted-value",
      ]) {
        assert.equal(
          elements.get(selector).textContent,
          "--",
          `${reason}: clears ${selector}`,
        );
      }
      for (let index = 0; index < 5; index += 1) {
        const sector = elements.get(`#sector-${index}`);
        assert.equal(sector.dataset.level, "invalid", `${reason}: sector ${index}`);
        assert.equal(
          sector.querySelector("span").textContent,
          "차단",
          `${reason}: sector ${index} label`,
        );
      }
    });
  }
});

test("blocked diagnostics clear all new and legacy count rows after live data", () => {
  const { panel, elements } = makeHarness();
  const live = livePresentationWithProbabilityMap();
  live.diagnostics = {
    raw_point_count: 11,
    ground_rejected_count: 13,
    self_clutter_rejected_count: 2,
    quality_gate_rejected_count: 3,
    confirmed_point_track_count: 4,
    confirmed_heatmap_track_count: 5,
    heatmap_reflection_count: 6,
    probability_active_cell_count: 7,
    probability_occupied_cell_count: 8,
    probability_decayed_cell_count: 9,
    probability_ttl_expired_cell_count: 10,
    path_switch_pending_count: 1,
    path_switch_accepted_count: 12,
  };
  panel.updateDiagnostics(live);
  panel.updateDiagnostics(staleBlockedPresentation("sensor_fault"));
  for (const selector of [
    "#raw-point-value",
    "#ground-rejected-value",
    "#clutter-rejected-value",
    "#reflection-value",
    "#confirmed-track-value",
    "#quality-rejected-value",
    "#probability-active-value",
    "#probability-occupied-value",
    "#probability-decayed-value",
    "#probability-expired-value",
    "#path-pending-value",
    "#path-accepted-value",
  ]) {
    assert.equal(elements.get(selector).textContent, "--", selector);
  }
  assert.equal(elements.get("#ground-filter-tag").textContent, "GROUND FILTER --");
});

test("operator views hide unconfirmed heatmap geometry but retain confirmed evidence", () => {
  const { panel, contexts } = makeHarness();
  const unconfirmed = {
    track_id: 1,
    source: "heatmap",
    heatmap_confirmed: false,
    forward_m: 0.20,
    lateral_m: 0.01,
    distance_m: 0.20,
    age_ms: 0,
  };
  const confirmed = {
    ...unconfirmed,
    track_id: 2,
    heatmap_confirmed: true,
    classification: "confirmed_obstacle",
  };
  const point = {
    track_id: 3,
    source: "point",
    point_confirmed: true,
    forward_m: 0.25,
    lateral_m: -0.01,
    distance_m: 0.25,
    age_ms: 0,
  };
  const scene = livePresentationWithProbabilityMap();
  scene.filteredTracks = [unconfirmed, confirmed, point];
  scene.referenceTracks = [unconfirmed];
  scene.tracks = [unconfirmed, confirmed, point];
  const outlines = [];
  const heatmaps = [];
  const points = [];
  panel.drawMetricGrid = () => {};
  panel.clipToMapBoundary = () => {};
  panel.drawProbabilisticLocalMap = () => {};
  panel.drawTravelTrail = () => {};
  panel.drawPlannedPath = () => {};
  panel.drawRobotFootprint = () => {};
  panel.drawRobot = () => {};
  panel.drawUncertaintyOutline = (_ctx, _transform, track) => {
    outlines.push(track.track_id);
  };
  panel.drawHeatmapUncertainty = (_ctx, _transform, track) => {
    heatmaps.push(track.track_id);
  };
  panel.drawPointMarker = (_ctx, _point, track) => {
    points.push(track.track_id);
  };
  panel.drawTrackLabel = () => {};
  const mainTransform = panel.makeCircularViewportTransform(
    640, 360, 0.50, { top: 0, right: 0, bottom: 0, left: 0 },
  );
  const collisionTransform = panel.makeViewportTransform(
    320, 180, 0.50, 0.50, { top: 0, right: 0, bottom: 0, left: 0 },
  );
  panel.drawLidarTopView(contexts[0], mainTransform, scene);
  panel.drawCollisionInset(contexts[1], collisionTransform, scene);
  assert.deepEqual(outlines, [2, 3, 2, 3]);
  assert.deepEqual(heatmaps, [2, 2]);
  assert.deepEqual(points, [3, 3]);
  assert.equal(outlines.includes(1), false);
  assert.equal(heatmaps.includes(1), false);
});

test("2.5d view hides unconfirmed heatmap reflection evidence", () => {
  const { panel, contexts, elements } = makeHarness();
  const tracks = [
    { source: "heatmap", heatmap_confirmed: false, forward_m: 0.20,
      lateral_m: 0.01, distance_m: 0.20, age_ms: 0 },
    { source: "heatmap", heatmap_confirmed: true, forward_m: 0.25,
      lateral_m: 0.02, distance_m: 0.25, age_ms: 0 },
    { source: "point", point_confirmed: true, forward_m: 0.30,
      lateral_m: -0.02, distance_m: 0.30, height_m: 0.01,
      height_band: "sensor_level", age_ms: 0 },
  ];
  panel.drawPointCloud25d(contexts[2], 420, 300, tracks, [], false);
  assert.equal(
    elements.get("#pointcloud-3d-status").textContent,
    "2 FILTERED · SENSOR-RELATIVE Z",
  );
  assert.equal(contexts[2].fillTextCalls.some(
    ([value]) => String(value).startsWith("0.20m"),
  ), false);
  assert.equal(contexts[2].fillTextCalls.some(
    ([value]) => String(value).startsWith("0.25m"),
  ), true);
  assert.equal(contexts[2].fillTextCalls.some(
    ([value]) => String(value).startsWith("0.30m"),
  ), true);
});

test("blocked draw clears short-memory pose and marks 2.5d sensor fault", () => {
  const { panel, elements } = makeHarness();
  const live = livePresentationWithProbabilityMap();
  panel.updateDiagnostics(live);
  panel.drawPointCloud25d(panel.pointCloud3dContext, 420, 300, [], [], false);
  assert.equal(
    elements.get("#pose-mode-value").textContent,
    "ROBOT RELATIVE · SHORT MEMORY",
  );
  assert.notEqual(elements.get("#pointcloud-3d-status").textContent, "SENSOR FAULT");

  const blocked = staleBlockedPresentation("sensor_fault");
  panel.presentation = blocked;
  panel.updateDiagnostics(blocked);
  panel.draw();

  assert.equal(elements.get("#pose-mode-value").textContent, "--");
  assert.equal(elements.get("#pointcloud-3d-status").textContent, "SENSOR FAULT");
});

test("every blocked reason replaces retained live warning with red stop copy", async (t) => {
  for (const [reason, heading, explanation] of BLOCKED_CASES) {
    await t.test(reason, () => {
      const { panel, elements } = makeHarness();
      panel.snapshot.warning = "STALE LIVE WARNING";

      panel.updateText(staleBlockedPresentation(reason));

      const warning = elements.get("#warning-text").textContent;
      assert.equal(
        warning,
        `${heading} · ${explanation}`,
        `${reason}: reason-derived stop warning`,
      );
      assert.doesNotMatch(
        warning,
        /STALE LIVE WARNING/,
        `${reason}: stale live warning`,
      );
      assert.equal(
        elements.get("#warning-strip").dataset.blocked,
        "true",
        `${reason}: red blocked warning state`,
      );
    });
  }

  const { panel, elements } = makeHarness();
  panel.snapshot.warning = "STALE LIVE WARNING";
  panel.updateText({
    blocked: false,
    hazard: { level: "NORMAL", threshold_m: 0.1 },
    hazardCopy: "LIVE NORMAL COPY",
    tracks: [],
  });
  assert.equal(elements.get("#warning-text").textContent, "LIVE NORMAL COPY");
  assert.equal(elements.get("#warning-strip").dataset.blocked, "false");
});

test("served warning strip has explicit red blocked styling", () => {
  const html = fs.readFileSync(FRONT_PATH, "utf8");
  assert.match(
    html,
    /\.warning\[data-blocked="true"\]\s*\{[^}]*rgba\(255,\s*81,\s*81,[^)]+\)[^}]*\}/s,
  );
  assert.match(
    html,
    /\.warning\[data-blocked="true"\]::before\s*\{[^}]*var\(--red\)[^}]*\}/s,
  );
});

test("live-input blocking badges follow the presentation reason, not stale snapshot status", () => {
  const expected = new Map([
    ["waiting", "WAITING"],
    ["stale", "STALE"],
    ["fault", "FAULT"],
    ["sensor_fault", "SENSOR FAULT"],
    ["http_lost", "HTTP LOST"],
  ]);
  for (const [reason, label] of expected) {
    const { panel, elements } = makeHarness();
    panel.updateText(staleBlockedPresentation(reason));
    const badge = elements.get("#radar-status");
    assert.equal(badge.dataset.status, reason, `${reason}: status token`);
    assert.equal(badge.textContent, label, `${reason}: badge copy`);
    assert.notEqual(badge.textContent, "LIVE", `${reason}: stale live badge`);
  }
});

test("blocked stale danger evidence cannot override sensor fault outputs", () => {
  const { panel, elements } = makeHarness();
  panel.snapshot.scene.hazard = {
    level: "DANGER",
    threshold_m: 0.1,
  };
  panel.snapshot.scene.tracks[0].distance_m = 0.05;
  const presentation = staleBlockedPresentation("sensor_fault");
  presentation.hazard = {
    level: "DANGER",
    threshold_m: 0.1,
  };

  panel.updateText(presentation);
  panel.updateDiagnostics(presentation);

  assert.equal(
    elements.get("#metric-hazard").querySelector("strong").textContent,
    "SENSOR_FAULT",
  );
  assert.equal(
    elements.get("#metric-hazard").dataset.hazard,
    "SENSOR_FAULT",
  );
  assert.equal(
    elements.get("#collision-inset").dataset.hazard,
    "SENSOR_FAULT",
  );
  assert.equal(
    elements.get("#scene-hazard-value").textContent,
    "SENSOR_FAULT",
  );
});
