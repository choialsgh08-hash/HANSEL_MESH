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
  const SCENE_GRID_V1 = Object.freeze({
    resolution_m: 0.05,
    forward_cells: 60,
    lateral_cells: 60,
    origin_forward_cell: 0,
    origin_lateral_cell: 30,
    encoding: "occupancy-u8-base64",
    layout: "forward-major_lateral-minor",
    unknown_value: 0,
  });

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

  function decodeSceneV1Grid(grid) {
    if (!grid || Object.entries(SCENE_GRID_V1).some(
      ([field, expected]) => grid[field] !== expected,
    )) {
      throw new Error("invalid scene grid contract");
    }
    return decodeOccupancyGrid(grid);
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
    if (scene.pose_mode !== "robot_relative") {
      throw new Error("invalid pose contract");
    }
    if (!scene.hazard || !HAZARD_LEVELS.has(scene.hazard.level) ||
        !Number.isFinite(scene.hazard.threshold_m) ||
        scene.hazard.threshold_m <= 0) {
      throw new Error("invalid hazard contract");
    }
    if (scene.hazard.level === "SENSOR_FAULT") {
      return { blocked: true, reason: "sensor_fault" };
    }
    const plannedPath = scene.planned_path || {
      status: "unavailable",
      resolution_m: 0.01,
      length_m: 0,
      points: [],
    };
    if (!Number.isFinite(plannedPath.resolution_m) ||
        Math.abs(plannedPath.resolution_m - 0.01) > 1e-9 ||
        !Number.isFinite(plannedPath.length_m) ||
        !Array.isArray(plannedPath.points) ||
        plannedPath.points.some((point) => !Array.isArray(point) ||
          point.length !== 2 || !Number.isFinite(point[0]) ||
          !Number.isFinite(point[1]))) {
      throw new Error("invalid planned path contract");
    }
    const accumulatedObstacles = Array.isArray(scene.accumulated_obstacles)
      ? scene.accumulated_obstacles
      : [];
    const hasPersistentNearObstacles = Array.isArray(
      scene.persistent_near_obstacles,
    );
    const displayedObstacles = hasPersistentNearObstacles
      ? scene.persistent_near_obstacles
      : accumulatedObstacles;
    if (displayedObstacles.some((obstacle) =>
      !obstacle || !Number.isFinite(obstacle.forward_m) ||
      !Number.isFinite(obstacle.lateral_m) ||
      !Number.isFinite(obstacle.confidence) ||
      !Number.isFinite(obstacle.age_ms) ||
      !Number.isFinite(obstacle.remaining_ms))) {
      throw new Error("invalid persistent obstacle contract");
    }
    const rollingLocalMap = scene.rolling_local_map || null;
    const rollingMapObstacles = rollingLocalMap && Array.isArray(
      rollingLocalMap.cells,
    ) ? rollingLocalMap.cells : displayedObstacles;
    if (rollingLocalMap && (
      !Number.isFinite(rollingLocalMap.radius_m) ||
      Math.abs(rollingLocalMap.radius_m - 0.50) > 1e-9 ||
      !Number.isFinite(rollingLocalMap.resolution_m) ||
      rollingLocalMap.motion_relative !== true ||
      rollingMapObstacles.some((obstacle) =>
        !obstacle || !Number.isFinite(obstacle.forward_m) ||
        !Number.isFinite(obstacle.lateral_m) ||
        !Number.isFinite(obstacle.confidence) ||
        !Number.isFinite(obstacle.age_ms) ||
        !Number.isFinite(obstacle.hits)))
    ) {
      throw new Error("invalid rolling local map contract");
    }
    const travelTrail = scene.travel_trail || {
      radius_m: 0.50,
      resolution_m: 0.01,
      motion_relative: true,
      source: "unavailable",
      points: [[0, 0]],
    };
    if (!Number.isFinite(travelTrail.radius_m) ||
        !Number.isFinite(travelTrail.resolution_m) ||
        travelTrail.motion_relative !== true ||
        !Array.isArray(travelTrail.points) ||
        travelTrail.points.some((point) => !Array.isArray(point) ||
          point.length !== 2 || !Number.isFinite(point[0]) ||
          !Number.isFinite(point[1]) ||
          Math.hypot(point[0], point[1]) > travelTrail.radius_m + 1e-9)) {
      throw new Error("invalid travel trail contract");
    }
    const allTracks = filterFreshTracks(scene.tracks);
    const tracks = filterFreshTracks(
      Array.isArray(scene.obstacle_tracks)
        ? scene.obstacle_tracks
        : scene.tracks,
    );
    const referenceTracks = filterFreshTracks(
      Array.isArray(scene.reference_tracks)
        ? scene.reference_tracks
        : allTracks.filter((track) => !tracks.some(
          (obstacle) => obstacle.track_id === track.track_id,
        )),
    );
    if ([...tracks, ...referenceTracks].some(
      (track) => !Number.isFinite(track.forward_m) ||
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
      poseMode: scene.pose_mode,
      grid: decodeSceneV1Grid(scene.grid),
      gridMeta: { ...scene.grid },
      tracks: tracks.map((track) => ({ ...track })),
      referenceTracks: referenceTracks.map((track) => ({ ...track })),
      plannedPath: {
        ...plannedPath,
        points: plannedPath.points.map((point) => [...point]),
      },
      motionCompensation: {
        status: "unavailable",
        applied: false,
        ...(scene.motion_compensation || {}),
      },
      accumulatedObstacles: displayedObstacles.map(
        (obstacle) => ({ ...obstacle }),
      ),
      rollingMapObstacles: rollingMapObstacles.map(
        (obstacle) => ({ ...obstacle }),
      ),
      rollingMap: rollingLocalMap ? {
        radiusM: rollingLocalMap.radius_m,
        resolutionM: rollingLocalMap.resolution_m,
        minimumHits: rollingLocalMap.minimum_hits,
        retention: rollingLocalMap.retention,
        motionRelative: rollingLocalMap.motion_relative,
      } : null,
      travelTrail: {
        radiusM: travelTrail.radius_m,
        resolutionM: travelTrail.resolution_m,
        source: travelTrail.source,
        points: travelTrail.points.map((point) => [...point]),
      },
      obstacleMemoryWindowMs: Number.isFinite(
        scene.persistent_near_obstacle_memory &&
          scene.persistent_near_obstacle_memory.window_ms,
      ) ? scene.persistent_near_obstacle_memory.window_ms
        : Number.isFinite(scene.obstacle_memory_window_ms)
          ? scene.obstacle_memory_window_ms : 5000,
      hazard: { ...scene.hazard },
      hazardCopy: scene.hazard.level === "NORMAL"
        ? "10cm 이내 확인 장애물 없음 · 미관측 영역 존재"
        : scene.hazard.level === "DANGER"
          ? "확인된 point 장애물 ≤10cm · 즉시 정지"
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
