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
  const HEIGHT_BANDS = new Set([
    "below_sensor", "sensor_level", "above_sensor", "unknown",
  ]);
  const ADDITIVE_DIAGNOSTIC_KEYS = Object.freeze([
    "raw_point_count",
    "ground_rejected_count",
    "ground_fringe_rejected_count",
    "reflectivity_surface_cell_count",
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
  ]);
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

  function isPlainObject(value) {
    if (!value || typeof value !== "object" || Array.isArray(value)) {
      return false;
    }
    const prototype = Object.getPrototypeOf(value);
    return prototype === Object.prototype || prototype === null;
  }

  function deepClone(value) {
    if (Array.isArray(value)) {
      return value.map((item) => deepClone(item));
    }
    if (value && typeof value === "object") {
      return Object.fromEntries(
        Object.entries(value).map(([key, item]) => [key, deepClone(item)]),
      );
    }
    return value;
  }

  function expectedHeightBand(heightM) {
    if (heightM === null) return "unknown";
    if (heightM < -0.10) return "below_sensor";
    if (heightM > 0.10) return "above_sensor";
    return "sensor_level";
  }

  function parseProbabilisticLocalMap(scene) {
    const map = scene.probabilistic_local_map;
    if (!isPlainObject(map) || map.radius_m !== 0.50 ||
        map.resolution_m !== 0.05 || map.occupied_threshold !== 0.75 ||
        map.decay_half_life_ms !== 1000 || map.ttl_ms !== 2500 ||
        map.retention !== "probability_decay_with_ttl" ||
        map.pose_mode !== "robot_relative_short_memory" ||
        map.pose_mode !== scene.pose_mode || !Array.isArray(map.cells)) {
      throw new Error("invalid probabilistic map contract");
    }
    const cells = map.cells.map((cell) => {
      if (!isPlainObject(cell) || !Number.isFinite(cell.forward_m) ||
          !Number.isFinite(cell.lateral_m) ||
          Math.hypot(cell.forward_m, cell.lateral_m) > map.radius_m + 1e-9 ||
          !Number.isFinite(cell.probability) || cell.probability <= 0.5 ||
          cell.probability > 0.97 || !Number.isFinite(cell.confidence) ||
          cell.confidence < 0 || cell.confidence > 1 ||
          typeof cell.occupied !== "boolean" ||
          cell.occupied !== (cell.probability >= map.occupied_threshold) ||
          !Number.isInteger(cell.hits) || cell.hits <= 0 ||
          !Number.isFinite(cell.age_ms) || cell.age_ms < 0 ||
          cell.age_ms >= map.ttl_ms ||
          !Number.isFinite(cell.range_uncertainty_m) ||
          cell.range_uncertainty_m < 0 ||
          !Number.isFinite(cell.lateral_uncertainty_m) ||
          cell.lateral_uncertainty_m < 0 ||
          !["point", "heatmap", "mixed"].includes(cell.source)) {
        throw new Error("invalid probabilistic map contract");
      }
      return deepClone(cell);
    });
    return {
      radiusM: map.radius_m,
      resolutionM: map.resolution_m,
      occupiedThreshold: map.occupied_threshold,
      decayHalfLifeMs: map.decay_half_life_ms,
      ttlMs: map.ttl_ms,
      retention: map.retention,
      poseMode: map.pose_mode,
      sourceField: "probabilistic_local_map",
      cells,
    };
  }

  function parseReflectivitySurface(scene) {
    const surface = scene.reflectivity_surface;
    if (!isPlainObject(surface) || surface.radius_m !== 0.50 ||
        surface.resolution_m !== 0.025 || surface.history_frames !== 5 ||
        surface.required_hits !== 3 ||
        surface.retention !== "three_of_five_complete_frames" ||
        surface.pose_mode !== "sensor_relative_short_memory" ||
        surface.source !== "calibrated_heatmap_residual" ||
        !Array.isArray(surface.cells)) {
      throw new Error("invalid reflectivity surface contract");
    }
    const occupiedKeys = new Set();
    const cells = surface.cells.map((cell) => {
      if (!isPlainObject(cell) || !Number.isFinite(cell.forward_m) ||
          !Number.isFinite(cell.lateral_m) || cell.forward_m < 0 ||
          Math.hypot(cell.forward_m, cell.lateral_m) > surface.radius_m + 1e-9 ||
          Math.abs(
            cell.forward_m / surface.resolution_m -
            Math.round(cell.forward_m / surface.resolution_m)
          ) > 1e-8 || Math.abs(
            cell.lateral_m / surface.resolution_m -
            Math.round(cell.lateral_m / surface.resolution_m)
          ) > 1e-8 || !Number.isFinite(cell.intensity) ||
          cell.intensity < 0 || cell.intensity > 1 ||
          !Number.isFinite(cell.persistence) || cell.persistence <= 0 ||
          cell.persistence > 1 || !Number.isInteger(cell.hits) ||
          cell.hits < surface.required_hits ||
          cell.hits > surface.history_frames ||
          Math.abs(
            cell.persistence - cell.hits / surface.history_frames
          ) > 1e-8 || !Number.isInteger(cell.age_ms) || cell.age_ms < 0) {
        throw new Error("invalid reflectivity surface contract");
      }
      const key = [
        Math.round(cell.forward_m / surface.resolution_m),
        Math.round(cell.lateral_m / surface.resolution_m),
      ].join(":");
      if (occupiedKeys.has(key)) {
        throw new Error("invalid reflectivity surface contract");
      }
      occupiedKeys.add(key);
      return deepClone(cell);
    });
    return {
      radiusM: surface.radius_m,
      resolutionM: surface.resolution_m,
      historyFrames: surface.history_frames,
      requiredHits: surface.required_hits,
      retention: surface.retention,
      poseMode: surface.pose_mode,
      source: surface.source,
      cells,
    };
  }

  function parseReflectivityCorridor(scene) {
    const corridor = scene.reflectivity_corridor;
    const invalid = () => {
      throw new Error("invalid reflectivity corridor contract");
    };
    if (!isPlainObject(corridor) ||
        !["unavailable", "candidate", "confirmed"].includes(corridor.status) ||
        corridor.method !== "deterministic_boundary_fit" ||
        ![null, "parallel", "converging"].includes(corridor.geometry) ||
        corridor.history_frames !== 5 || corridor.required_hits !== 3 ||
        !Number.isInteger(corridor.hits) || corridor.hits < 0 ||
        corridor.hits > corridor.history_frames ||
        !Number.isFinite(corridor.confidence) || corridor.confidence < 0 ||
        corridor.confidence > 1 || !Array.isArray(corridor.walls)) {
      return invalid();
    }
    if (corridor.status === "unavailable") {
      if (corridor.hits !== 0 || corridor.measured_heading_deg !== null ||
          corridor.guidance_heading_deg !== null || corridor.width_m !== null ||
          corridor.geometry !== null || corridor.confidence !== 0 ||
          corridor.walls.length !== 0) {
        return invalid();
      }
      return {
        status: corridor.status,
        method: corridor.method,
        geometry: null,
        historyFrames: corridor.history_frames,
        requiredHits: corridor.required_hits,
        hits: corridor.hits,
        measuredHeadingDeg: null,
        guidanceHeadingDeg: null,
        widthM: null,
        confidence: corridor.confidence,
        walls: [],
      };
    }
    if ((corridor.status === "candidate" &&
          (corridor.hits < 1 || corridor.hits >= corridor.required_hits)) ||
        (corridor.status === "confirmed" &&
          corridor.hits < corridor.required_hits) ||
        !Number.isFinite(corridor.measured_heading_deg) ||
        Math.abs(corridor.measured_heading_deg) > 75 ||
        !Number.isFinite(corridor.guidance_heading_deg) ||
        Math.abs(corridor.guidance_heading_deg) > 55 ||
        Math.abs(
          corridor.guidance_heading_deg - Math.max(
            -55, Math.min(55, corridor.measured_heading_deg),
          )
        ) > 1e-6 || !["parallel", "converging"].includes(corridor.geometry) ||
        !Number.isFinite(corridor.width_m) || corridor.width_m < 0 ||
        corridor.width_m > 0.55 ||
        (corridor.geometry === "parallel" && corridor.width_m < 0.06) ||
        corridor.walls.length !== 2) {
      return invalid();
    }
    const walls = corridor.walls.map((wall) => {
      if (!isPlainObject(wall) ||
          !["wall_1", "wall_2"].includes(wall.wall_id) ||
          !Number.isFinite(wall.heading_deg) ||
          Math.abs(wall.heading_deg) > 75 ||
          !Number.isFinite(wall.normal_offset_m) ||
          !Number.isInteger(wall.support_cells) || wall.support_cells < 2 ||
          !Number.isFinite(wall.span_m) || wall.span_m < 0.04 ||
          !Number.isFinite(wall.mean_intensity) ||
          wall.mean_intensity < 0.30 || wall.mean_intensity > 1 ||
          !validPointPair(wall.start) || !validPointPair(wall.end) ||
          wall.end[0] < wall.start[0] - 1e-9) {
        return invalid();
      }
      const deltaForward = wall.end[0] - wall.start[0];
      const deltaLateral = wall.end[1] - wall.start[1];
      const endpointSpan = Math.hypot(deltaForward, deltaLateral);
      const endpointHeading = Math.atan2(deltaLateral, deltaForward) * 180 / Math.PI;
      const headingRad = wall.heading_deg * Math.PI / 180;
      const endpointNormalOffset =
        -Math.sin(headingRad) * wall.start[0] +
        Math.cos(headingRad) * wall.start[1];
      if (endpointSpan + 2e-5 < wall.span_m ||
          Math.abs(endpointHeading - wall.heading_deg) > 2e-4 ||
          Math.abs(endpointNormalOffset - wall.normal_offset_m) > 2e-5) {
        return invalid();
      }
      return {
        wallId: wall.wall_id,
        headingDeg: wall.heading_deg,
        normalOffsetM: wall.normal_offset_m,
        supportCells: wall.support_cells,
        spanM: wall.span_m,
        meanIntensity: wall.mean_intensity,
        start: [...wall.start],
        end: [...wall.end],
      };
    });
    if (new Set(walls.map((wall) => wall.wallId)).size !== 2) {
      return invalid();
    }
    if (corridor.geometry === "parallel") {
      if (Math.abs(walls[0].headingDeg - walls[1].headingDeg) > 10 ||
          Math.abs(
            Math.abs(walls[0].normalOffsetM - walls[1].normalOffsetM) -
            corridor.width_m
          ) > 2e-5 || Math.abs(
            (walls[0].headingDeg + walls[1].headingDeg) / 2 -
            corridor.measured_heading_deg
          ) > 2e-5) {
        return invalid();
      }
    } else {
      const nearWidth = walls[1].start[1] - walls[0].start[1];
      const farWidth = walls[1].end[1] - walls[0].end[1];
      const nearCenter = [
        (walls[0].start[0] + walls[1].start[0]) / 2,
        (walls[0].start[1] + walls[1].start[1]) / 2,
      ];
      const farCenter = [
        (walls[0].end[0] + walls[1].end[0]) / 2,
        (walls[0].end[1] + walls[1].end[1]) / 2,
      ];
      const centerHeading = Math.atan2(
        farCenter[1] - nearCenter[1],
        farCenter[0] - nearCenter[0],
      ) * 180 / Math.PI;
      if (Math.abs(walls[0].start[0] - walls[1].start[0]) > 2e-5 ||
          Math.abs(walls[0].end[0] - walls[1].end[0]) > 2e-5 ||
          nearWidth <= 0 || farWidth < 0 ||
          nearWidth - farWidth < 0.075 - 2e-5 ||
          Math.abs(farWidth - corridor.width_m) > 2e-5 ||
          Math.abs(centerHeading - corridor.measured_heading_deg) > 2e-4) {
        return invalid();
      }
    }
    return {
      status: corridor.status,
      method: corridor.method,
      geometry: corridor.geometry,
      historyFrames: corridor.history_frames,
      requiredHits: corridor.required_hits,
      hits: corridor.hits,
      measuredHeadingDeg: corridor.measured_heading_deg,
      guidanceHeadingDeg: corridor.guidance_heading_deg,
      widthM: corridor.width_m,
      confidence: corridor.confidence,
      walls,
    };
  }

  function validateFilteredTrack(track) {
    if (!isPlainObject(track) || !Number.isInteger(track.track_id) ||
        track.track_id <= 0 || !Number.isInteger(track.age_ms) ||
        track.age_ms < 0 || !["point", "heatmap"].includes(track.source) ||
        !Number.isFinite(track.forward_m) ||
        !Number.isFinite(track.lateral_m) ||
        !Number.isFinite(track.distance_m) || track.distance_m < 0 ||
        Math.abs(
          track.distance_m - Math.hypot(track.forward_m, track.lateral_m)
        ) > 0.001 || !Number.isFinite(track.confidence) ||
        track.confidence < 0 || track.confidence > 1 ||
        !Number.isFinite(track.range_uncertainty_m) ||
        track.range_uncertainty_m < 0 ||
        !Number.isFinite(track.lateral_uncertainty_m) ||
        track.lateral_uncertainty_m < 0 ||
        !HEIGHT_BANDS.has(track.height_band)) {
      throw new Error("invalid track contract");
    }
    if (track.source === "point") {
      if (!Number.isFinite(track.hazard_distance_m) ||
          track.hazard_distance_m < 0 ||
          typeof track.point_confirmed !== "boolean" ||
          typeof track.map_confirmed !== "boolean" ||
          track.classification !== (track.point_confirmed
            ? "confirmed_obstacle" : "point_candidate") ||
          !Number.isFinite(track.height_m) ||
          track.height_band !== expectedHeightBand(track.height_m) ||
          track.height_band === "unknown" ||
          track.height_reference !== "sensor_relative_uncalibrated") {
        throw new Error("invalid track contract");
      }
    } else if (Object.prototype.hasOwnProperty.call(
      track, "hazard_distance_m"
    ) || typeof track.heatmap_confirmed !== "boolean" ||
        track.classification !== (track.heatmap_confirmed
          ? "confirmed_obstacle" : "reflection_evidence") ||
        track.height_m !== null || track.height_band !== "unknown" ||
        track.height_reference !== "unavailable") {
      throw new Error("invalid track contract");
    }
    return deepClone(track);
  }

  function parseDiagnostics(diagnostics, required) {
    if (required && !isPlainObject(diagnostics)) {
      throw new Error("invalid diagnostics contract");
    }
    if (diagnostics !== undefined && diagnostics !== null &&
        !isPlainObject(diagnostics)) {
      throw new Error("invalid diagnostics contract");
    }
    const parsed = deepClone(diagnostics || {});
    for (const key of ADDITIVE_DIAGNOSTIC_KEYS) {
      if (!Object.prototype.hasOwnProperty.call(parsed, key)) {
        if (required) {
          throw new Error("invalid diagnostics contract");
        }
        parsed[key] = null;
        continue;
      }
      if (!Number.isInteger(parsed[key]) || parsed[key] < 0) {
        throw new Error("invalid diagnostics contract");
      }
    }
    return parsed;
  }

  function parseGroundFilter(groundFilter, required) {
    if (!required && (groundFilter === undefined || groundFilter === null)) {
      return null;
    }
    if (!isPlainObject(groundFilter)) {
      throw new Error("invalid ground filter contract");
    }
    const numericKeys = [
      "sensor_height_m",
      "sensor_pitch_deg",
      "ground_rejection_height_m",
      "ground_uncertainty_m",
      "effective_rejection_height_m",
      "filter_range_m",
    ];
    if (numericKeys.some((key) =>
      typeof groundFilter[key] !== "number" ||
      !Number.isFinite(groundFilter[key])) ||
        groundFilter.sensor_height_m < 0 ||
        groundFilter.ground_rejection_height_m < 0 ||
        groundFilter.ground_uncertainty_m < 0 ||
        groundFilter.filter_range_m <= 0 ||
        groundFilter.effective_rejection_height_m >= groundFilter.sensor_height_m ||
        Math.abs(
          groundFilter.effective_rejection_height_m -
          groundFilter.ground_rejection_height_m -
          groundFilter.ground_uncertainty_m
        ) > 1e-9 ||
        groundFilter.pitch_convention !== "positive_up" ||
        groundFilter.status !== "configured") {
      throw new Error("invalid ground filter contract");
    }
    return {
      sensorHeightM: groundFilter.sensor_height_m,
      sensorPitchDeg: groundFilter.sensor_pitch_deg,
      groundRejectionHeightM: groundFilter.ground_rejection_height_m,
      groundUncertaintyM: groundFilter.ground_uncertainty_m,
      effectiveRejectionHeightM: groundFilter.effective_rejection_height_m,
      filterRangeM: groundFilter.filter_range_m,
      pitchConvention: groundFilter.pitch_convention,
      status: groundFilter.status,
    };
  }

  function validPointPair(point) {
    return Array.isArray(point) && point.length === 2 &&
      Number.isFinite(point[0]) && Number.isFinite(point[1]);
  }

  function parseBlockedPreviousPath(overlay) {
    if (overlay === null) return null;
    if (!isPlainObject(overlay) || !Array.isArray(overlay.points) ||
        overlay.points.length === 0 ||
        overlay.points.some((point) => !validPointPair(point)) ||
        !Array.isArray(overlay.segments) || overlay.segments.length === 0) {
      throw new Error("invalid planned path contract");
    }
    for (const segment of overlay.segments) {
      if (!isPlainObject(segment) ||
          !Number.isInteger(segment.start_index) ||
          !Number.isInteger(segment.end_index) ||
          segment.start_index < 0 ||
          segment.start_index > segment.end_index ||
          segment.end_index >= overlay.points.length ||
          typeof segment.reason !== "string" || segment.reason.trim() === "") {
        throw new Error("invalid planned path contract");
      }
    }
    return deepClone(overlay);
  }

  function parseNewPlannedPath(path, diagnostics) {
    const pathStatuses = new Set([
      "clear", "limited", "blocked_near_field", "initializing", "reset",
    ]);
    const stabilityStatuses = new Set([
      "replanned", "initialized", "stable", "holding_candidate",
      "route_changed_confirmed", "route_changed_blocked", "initializing",
      "reset",
    ]);
    if (!isPlainObject(path) || !pathStatuses.has(path.status) ||
        !stabilityStatuses.has(path.stability_status) ||
        path.resolution_m !== 0.01 || !Number.isFinite(path.length_m) ||
        path.length_m < 0 || !Number.isFinite(path.curvature_per_m) ||
        !Array.isArray(path.points) || path.points.length === 0 ||
        path.points.some((point) => !validPointPair(point)) ||
        path.robot_width_m !== 0.15 || path.safety_margin_m !== 0.03 ||
        path.minimum_passage_width_m !== 0.21 ||
        path.minimum_hold_ms !== 0 || path.switch_confirm_frames !== 3 ||
        !Number.isInteger(path.pending_switch_frames) ||
        path.pending_switch_frames < 0 || path.pending_switch_frames >= 3 ||
        typeof path.switch_accepted !== "boolean" ||
        path.replan_every_frame !== true ||
        !Object.prototype.hasOwnProperty.call(path, "blocked_previous_path")) {
      throw new Error("invalid planned path contract");
    }
    const neutral = path.status === "initializing" || path.status === "reset";
    if ((neutral && path.stability_status !== path.status) ||
        (!neutral && ["initializing", "reset"].includes(path.stability_status)) ||
        Math.abs(path.points[0][0]) > 1e-9 ||
        Math.abs(path.points[0][1]) > 1e-9 ||
        Math.abs(path.length_m - (path.points.length - 1) * 0.01) > 1e-9 ||
        path.points.slice(1).some((point, index) =>
          Math.abs(Math.hypot(
            point[0] - path.points[index][0],
            point[1] - path.points[index][1],
          ) - 0.01) > 0.001) ||
        (neutral && (path.length_m !== 0 || path.points.length !== 1)) ||
        (path.status === "blocked_near_field" &&
          (path.length_m !== 0 || path.points.length !== 1))) {
      throw new Error("invalid planned path contract");
    }
    const routeChanged = [
      "route_changed_confirmed", "route_changed_blocked",
    ].includes(path.stability_status);
    if (path.switch_accepted !== routeChanged ||
        path.pending_switch_frames !== diagnostics.path_switch_pending_count ||
        (path.stability_status === "holding_candidate"
          ? path.pending_switch_frames === 0
          : path.pending_switch_frames !== 0)) {
      throw new Error("invalid planned path contract");
    }
    return {
      ...deepClone(path),
      blocked_previous_path: parseBlockedPreviousPath(
        path.blocked_previous_path,
      ),
    };
  }

  function parseLegacyPlannedPath(path) {
    const plannedPath = path || {
      status: "unavailable",
      resolution_m: 0.01,
      length_m: 0,
      points: [],
    };
    if (!isPlainObject(plannedPath) ||
        !Number.isFinite(plannedPath.resolution_m) ||
        Math.abs(plannedPath.resolution_m - 0.01) > 1e-9 ||
        !Number.isFinite(plannedPath.length_m) ||
        !Array.isArray(plannedPath.points) ||
        plannedPath.points.some((point) => !validPointPair(point))) {
      throw new Error("invalid planned path contract");
    }
    return deepClone(plannedPath);
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
    const hasProbabilityMap = Object.prototype.hasOwnProperty.call(
      scene, "probabilistic_local_map",
    );
    if (scene.pose_mode !== (hasProbabilityMap
      ? "robot_relative_short_memory" : "robot_relative")) {
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
    const probabilisticLocalMap = hasProbabilityMap
      ? parseProbabilisticLocalMap(scene) : null;
    const reflectivitySurface = hasProbabilityMap
      ? parseReflectivitySurface(scene) : null;
    const reflectivityCorridor = hasProbabilityMap
      ? parseReflectivityCorridor(scene) : null;
    const groundFilter = parseGroundFilter(scene.ground_filter, hasProbabilityMap);
    const diagnostics = parseDiagnostics(scene.diagnostics, hasProbabilityMap);
    const plannedPath = hasProbabilityMap
      ? parseNewPlannedPath(scene.planned_path, diagnostics)
      : parseLegacyPlannedPath(scene.planned_path);
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
    const rollingLocalMap = hasProbabilityMap
      ? null : scene.rolling_local_map || null;
    const rollingMapObstacles = hasProbabilityMap
      ? probabilisticLocalMap.cells
      : rollingLocalMap && Array.isArray(rollingLocalMap.cells)
        ? rollingLocalMap.cells : displayedObstacles;
    if (!hasProbabilityMap && rollingLocalMap && (
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
    const validatedSceneTracks = hasProbabilityMap
      ? (Array.isArray(scene.tracks) ? scene.tracks : []).map(
        (track) => validateFilteredTrack(track),
      )
      : deepClone(Array.isArray(scene.tracks) ? scene.tracks : []);
    const filteredTracks = filterFreshTracks(validatedSceneTracks);
    const guidanceSource = Array.isArray(scene.obstacle_tracks)
      ? scene.obstacle_tracks : validatedSceneTracks;
    const tracks = filterFreshTracks(hasProbabilityMap
      ? guidanceSource.map((track) => validateFilteredTrack(track))
      : deepClone(guidanceSource));
    const referenceSource = Array.isArray(scene.reference_tracks)
      ? scene.reference_tracks
      : filteredTracks.filter((track) => !tracks.some(
        (obstacle) => obstacle.track_id === track.track_id,
      ));
    const referenceTracks = filterFreshTracks(hasProbabilityMap
      ? referenceSource.map((track) => validateFilteredTrack(track))
      : deepClone(referenceSource));
    if (!hasProbabilityMap && [
      ...filteredTracks, ...tracks, ...referenceTracks,
    ].some(
      (track) => !Number.isFinite(track.forward_m) ||
        !Number.isFinite(track.lateral_m))) {
      throw new Error("invalid track contract");
    }
    if (scene.hazard.level === "DANGER" && !tracks.some(
      (track) => track.source === "point" &&
        track.point_confirmed === true &&
        Number.isFinite(track.hazard_distance_m) &&
        track.hazard_distance_m >= 0 &&
        track.hazard_distance_m <= scene.hazard.threshold_m,
    )) {
      throw new Error("DANGER contract is inconsistent");
    }
    return {
      blocked: false,
      poseMode: scene.pose_mode,
      grid: decodeSceneV1Grid(scene.grid),
      gridMeta: deepClone(scene.grid),
      probabilisticLocalMap,
      reflectivitySurface,
      reflectivityCorridor,
      groundFilter,
      filteredTracks: deepClone(filteredTracks),
      tracks: deepClone(tracks),
      referenceTracks: deepClone(referenceTracks),
      diagnostics,
      plannedPath: deepClone(plannedPath),
      motionCompensation: {
        status: "unavailable",
        applied: false,
        ...deepClone(scene.motion_compensation || {}),
      },
      accumulatedObstacles: deepClone(displayedObstacles),
      rollingMapObstacles: deepClone(rollingMapObstacles),
      rollingMap: hasProbabilityMap ? {
        radiusM: probabilisticLocalMap.radiusM,
        resolutionM: probabilisticLocalMap.resolutionM,
        minimumHits: 1,
        retention: probabilisticLocalMap.retention,
        motionRelative: true,
      } : rollingLocalMap ? {
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
      hazard: deepClone(scene.hazard),
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
