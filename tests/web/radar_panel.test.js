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
  return {
    clearRectCalls: [],
    fillRectCalls: [],
    fillTextCalls: [],
    fillTextStyles: [],
    strokeRectStyles: [],
    setTransform() {},
    save() {},
    restore() {},
    beginPath() {},
    arc() {},
    fill() {},
    stroke() {},
    moveTo() {},
    lineTo() {},
    measureText(text) {
      return { width: String(text).length * 7 };
    },
    clearRect(...args) {
      this.clearRectCalls.push(args);
    },
    fillRect(...args) {
      this.fillRectCalls.push(args);
    },
    strokeRect() {
      this.strokeRectStyles.push(this.strokeStyle);
    },
    fillText(text, ...args) {
      this.fillTextCalls.push([text, ...args]);
      this.fillTextStyles.push(this.fillStyle);
    },
  };
}

function makeCanvas(context) {
  return {
    width: 0,
    height: 0,
    getContext() {
      return context;
    },
    getBoundingClientRect() {
      return { width: 640, height: 360 };
    },
  };
}

function makeHarness() {
  const mainContext = makeContext();
  const collisionContext = makeContext();
  const mainCanvas = makeCanvas(mainContext);
  const collisionCanvas = makeCanvas(collisionContext);
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
    "axis-value",
    "profile-value",
    "calibration-value",
    "pose-mode-value",
    "frame-value",
    "gap-value",
    "raw-point-value",
    "confirmed-track-value",
    "clutter-rejected-value",
    "heatmap-rejected-value",
    "grid-status-value",
    "scene-hazard-value",
    "axis-warning",
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
  mainContext.clearRectCalls.length = 0;
  mainContext.fillRectCalls.length = 0;
  mainContext.fillTextCalls.length = 0;
  mainContext.fillTextStyles.length = 0;
  mainContext.strokeRectStyles.length = 0;
  collisionContext.clearRectCalls.length = 0;
  collisionContext.fillRectCalls.length = 0;
  collisionContext.fillTextCalls.length = 0;
  collisionContext.fillTextStyles.length = 0;
  collisionContext.strokeRectStyles.length = 0;
  return {
    panel,
    elements,
    contexts: [mainContext, collisionContext],
  };
}

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
      assert.equal(overlayCalls.length, 2, `${reason}: overlay count`);
      assert.deepEqual(
        contexts.map((context) => context.clearRectCalls.length),
        [1, 1],
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
        "#confirmed-track-value",
        "#clutter-rejected-value",
        "#heatmap-rejected-value",
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
