(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) {
    module.exports = api;
  }
  if (root) {
    root.HanselRadarScene = api;
  }
})(typeof globalThis === "object" ? globalThis : this, function () {
  "use strict";
  const TRACK_MAX_AGE_MS = 300;
  const HAZARD_LEVELS = new Set(["DANGER", "NORMAL", "UNKNOWN", "SENSOR_FAULT"]);

  function decodeBase64(text) {
    if (typeof text !== "string" ||
        !/^(?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)?$/.test(text)) {
      throw new Error("invalid base64");
    }
    if (typeof Buffer === "function") {
      const bytes = Buffer.from(text, "base64");
      if (bytes.toString("base64") !== text) {
        throw new Error("invalid base64");
      }
      return Uint8Array.from(bytes);
    }
    const decoded = atob(text);
    if (btoa(decoded) !== text) {
      throw new Error("invalid base64");
    }
    return Uint8Array.from(decoded, (character) => character.charCodeAt(0));
  }

  function decodeOccupancyGrid(grid) {
    if (!grid || grid.encoding !== "occupancy-u8-base64" ||
        grid.layout !== "forward-major_lateral-minor" ||
        grid.unknown_value !== 0 ||
        !Number.isFinite(grid.resolution_m) || grid.resolution_m <= 0 ||
        !Number.isInteger(grid.forward_cells) || grid.forward_cells <= 0 ||
        !Number.isInteger(grid.lateral_cells) || grid.lateral_cells <= 0 ||
        !Number.isInteger(grid.origin_forward_cell) ||
        grid.origin_forward_cell < 0 ||
        grid.origin_forward_cell >= grid.forward_cells ||
        !Number.isInteger(grid.origin_lateral_cell) ||
        grid.origin_lateral_cell < 0 ||
        grid.origin_lateral_cell >= grid.lateral_cells) {
      throw new Error("invalid grid contract");
    }
    const bytes = decodeBase64(grid.data_base64);
    if (bytes.length !== grid.forward_cells * grid.lateral_cells) {
      throw new Error("grid length does not match dimensions");
    }
    return bytes;
  }

  function filterFreshTracks(tracks, maxAgeMs = TRACK_MAX_AGE_MS) {
    return (Array.isArray(tracks) ? tracks : []).filter(
      (track) => track && Number.isFinite(track.age_ms) &&
        track.age_ms >= 0 && track.age_ms < maxAgeMs,
    );
  }

  function parseRadarScene(snapshot) {
    const scene = snapshot && snapshot.scene;
    if (!scene || scene.schema_version !== 1) {
      throw new Error("unsupported scene schema");
    }
    if (!["live", "degraded"].includes(snapshot.status)) {
      return { blocked: true, reason: snapshot.status };
    }
    if (!["ok", "synthetic"].includes(scene.calibration_status)) {
      return { blocked: true, reason: scene.calibration_status };
    }
    if (!scene.hazard || !HAZARD_LEVELS.has(scene.hazard.level) ||
        !Number.isFinite(scene.hazard.threshold_m) ||
        scene.hazard.threshold_m <= 0) {
      throw new Error("invalid hazard contract");
    }
    const tracks = filterFreshTracks(scene.tracks);
    if (tracks.some((track) => !Number.isFinite(track.forward_m) ||
        !Number.isFinite(track.lateral_m))) {
      throw new Error("invalid track contract");
    }
    if (scene.hazard.level === "DANGER" && !tracks.some(
      (track) => track.source === "point" &&
        track.point_confirmed === true &&
        Number.isFinite(track.distance_m) && track.distance_m >= 0 &&
        track.distance_m <= scene.hazard.threshold_m,
    )) {
      throw new Error("DANGER contract is inconsistent");
    }
    return {
      blocked: false,
      grid: decodeOccupancyGrid(scene.grid),
      gridMeta: { ...scene.grid },
      tracks: tracks.map((track) => ({ ...track })),
      hazard: { ...scene.hazard },
      hazardCopy: scene.hazard.level === "NORMAL"
        ? "10cm 이내 확인 장애물 없음 · 미관측 영역 존재"
        : scene.hazard.reason,
    };
  }

  function makeMapTransform(width, height, forwardMaxM, halfWidthM) {
    const scale = Math.min(width / (halfWidthM * 2), height / forwardMaxM);
    return { originX: width / 2, originY: height, scale };
  }

  function projectMapPoint(transform, forwardM, lateralM) {
    return {
      x: transform.originX + lateralM * transform.scale,
      y: transform.originY - forwardM * transform.scale,
    };
  }

  return {
    TRACK_MAX_AGE_MS,
    decodeOccupancyGrid,
    filterFreshTracks,
    parseRadarScene,
    makeMapTransform,
    projectMapPoint,
  };
});
