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
    "heatmap_cells_observed",
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
          ![
            "point", "heatmap", "mixed", "adc_heatmap_fused",
          ].includes(cell.source)) {
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
    if (isPlainObject(surface) &&
        surface.source === "full_range_azimuth_heatmap") {
      if (surface.radius_m !== 0.50 ||
          Object.prototype.hasOwnProperty.call(surface, "resolution_m") ||
          (!Number.isFinite(surface.range_step_m) &&
            !(surface.range_step_m === null && surface.cells?.length === 0)) ||
          (Number.isFinite(surface.range_step_m) && surface.range_step_m <= 0) ||
          surface.angular_resolution_deg !== 29.0 ||
          surface.history_frames !== 5 || surface.required_hits !== 3 ||
          surface.retention !== "continuous_five_frame_evidence" ||
          surface.pose_mode !== "sensor_relative_short_memory" ||
          !(
            (surface.calibrated === true && surface.normalization ===
              "current_range_common_mode_physical_beam") ||
            (surface.calibrated === false && surface.normalization ===
              "per_frame_median_polish")
          ) ||
          surface.room_baseline_used !== false ||
          surface.valid_min_range_m !== 0.12 ||
          surface.independent_beam_count !== 5 ||
          !Array.isArray(surface.cells)) {
        throw new Error("invalid reflectivity surface contract");
      }
      const occupiedKeys = new Set();
      const cells = surface.cells.map((cell) => {
        if (!isPlainObject(cell) || !Number.isInteger(cell.range_index) ||
            cell.range_index < 1 || !Number.isInteger(cell.azimuth_index) ||
            cell.azimuth_index < 0 || !Number.isFinite(cell.range_m) ||
            cell.range_m <= 0 || cell.range_m > surface.radius_m + 1e-9 ||
            !Number.isFinite(cell.azimuth_deg) ||
            Math.abs(cell.azimuth_deg) > 60.0 + 1e-9 ||
            !Number.isFinite(cell.forward_m) || cell.forward_m < 0 ||
            !Number.isFinite(cell.lateral_m) ||
            Math.abs(Math.hypot(cell.forward_m, cell.lateral_m) -
              cell.range_m) > 2e-5 || !Number.isFinite(cell.intensity) ||
            cell.intensity < 0 || cell.intensity > 1 ||
            !Number.isFinite(cell.confidence) || cell.confidence < 0 ||
            cell.confidence > 1 || !Number.isFinite(cell.persistence) ||
            cell.persistence <= 0 || cell.persistence > 1 ||
            !Number.isInteger(cell.hits) || cell.hits < 1 ||
            cell.hits > surface.history_frames ||
            Math.abs(cell.persistence - cell.hits / surface.history_frames) >
              1e-8 || typeof cell.verified !== "boolean" ||
            cell.verified !== (cell.hits >= surface.required_hits) ||
            !Number.isInteger(cell.age_ms) || cell.age_ms < 0 ||
            !Number.isFinite(cell.relative_db) ||
            !Number.isFinite(cell.calibrated_residual_db) ||
            Math.abs(
              cell.calibrated_residual_db - cell.relative_db
            ) > 1e-9 ||
            !Number.isFinite(cell.range_uncertainty_m) ||
            cell.range_uncertainty_m <= 0 ||
            cell.angular_uncertainty_deg !== 14.5 ||
            !Number.isFinite(cell.scene_change_score) ||
            cell.scene_change_score < 0 || cell.scene_change_score > 1 ||
            (Object.prototype.hasOwnProperty.call(
              cell, "baseline_delta_db",
            ) && !Number.isFinite(cell.baseline_delta_db))) {
          throw new Error("invalid reflectivity surface contract");
        }
        const key = `${cell.range_index}:${cell.azimuth_index}`;
        if (occupiedKeys.has(key)) {
          throw new Error("invalid reflectivity surface contract");
        }
        occupiedKeys.add(key);
        return deepClone(cell);
      });
      return {
        radiusM: surface.radius_m,
        rangeStepM: surface.range_step_m,
        angularResolutionDeg: surface.angular_resolution_deg,
        historyFrames: surface.history_frames,
        requiredHits: surface.required_hits,
        retention: surface.retention,
        poseMode: surface.pose_mode,
        source: surface.source,
        normalization: surface.normalization,
        calibrated: surface.calibrated,
        roomBaselineUsed: surface.room_baseline_used,
        validMinRangeM: surface.valid_min_range_m,
        independentBeamCount: surface.independent_beam_count,
        cells,
      };
    }
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

  function rawHeatmapEvidenceSurface(snapshot) {
    const heatmap = snapshot && snapshot.frame && snapshot.frame.heatmap;
    if (!isPlainObject(heatmap) || heatmap.encoding !== "log-u8" ||
        !Number.isInteger(heatmap.range_bins) || heatmap.range_bins < 1 ||
        !Number.isInteger(heatmap.azimuth_bins) || heatmap.azimuth_bins < 1 ||
        !Number.isFinite(heatmap.range_step_m) || heatmap.range_step_m <= 0) {
      throw new Error("no usable current heatmap evidence");
    }
    const bytes = decodeBase64(heatmap.data_base64);
    if (bytes.length !== heatmap.range_bins * heatmap.azimuth_bins) {
      throw new Error("current heatmap evidence shape mismatch");
    }
    const cells = [];
    for (let rangeIndex = 1; rangeIndex < heatmap.range_bins; rangeIndex += 1) {
      const rangeM = rangeIndex * heatmap.range_step_m;
      if (rangeM < 0.07 || rangeM > 0.50 + 1e-9) continue;
      for (let azimuthIndex = 0; azimuthIndex < heatmap.azimuth_bins;
        azimuthIndex += 1) {
        const intensity = bytes[
          rangeIndex * heatmap.azimuth_bins + azimuthIndex
        ] / 255;
        const spatialFrequency = Math.max(-1, Math.min(
          1,
          2 * (azimuthIndex - heatmap.azimuth_bins / 2) /
            heatmap.azimuth_bins,
        ));
        const azimuthDeg = Math.asin(spatialFrequency) * 180 / Math.PI;
        if (Math.abs(azimuthDeg) > 60 + 1e-9) continue;
        const angleRad = azimuthDeg * Math.PI / 180;
        cells.push({
          range_index: rangeIndex,
          azimuth_index: azimuthIndex,
          range_m: rangeM,
          azimuth_deg: azimuthDeg,
          forward_m: rangeM * Math.cos(angleRad),
          lateral_m: rangeM * Math.sin(angleRad),
          intensity,
          confidence: intensity,
          persistence: 0.2,
          hits: 1,
          verified: false,
          age_ms: 0,
          relative_db: intensity,
          calibrated_residual_db: intensity,
          range_uncertainty_m: heatmap.range_step_m / 2,
          angular_uncertainty_deg: 14.5,
        });
      }
    }
    return {
      radiusM: 0.50,
      rangeStepM: heatmap.range_step_m,
      angularResolutionDeg: 29.0,
      historyFrames: 5,
      requiredHits: 3,
      retention: "current_frame_unverified_evidence",
      poseMode: "sensor_relative_current_frame",
      source: "full_range_azimuth_heatmap",
      normalization: "firmware_log_u8_current_frame",
      calibrated: false,
      roomBaselineUsed: false,
      validMinRangeM: 0.07,
      independentBeamCount: 5,
      cells,
    };
  }

  function parseAdcRangeAngleSurface(scene) {
    if (!Object.prototype.hasOwnProperty.call(
      scene, "adc_range_angle_surface",
    )) {
      return null;
    }
    const surface = scene.adc_range_angle_surface;
    const invalid = () => {
      throw new Error("invalid adc_range_angle_surface contract");
    };
    if (!isPlainObject(surface) ||
        surface.source !== "uart_last_chirp_pair_adc" ||
        surface.direction_semantics !== "range_azimuth" ||
        surface.processing !== "bpm_decode_hann_range_fft_four_element_das" ||
        !["evm_design_default", "measured"].includes(
          surface.phase_calibration,
        ) || surface.history_frames !== 5 || surface.required_hits !== 3 ||
        surface.retention !== "three_of_five_complete_frames" ||
        !Number.isFinite(surface.range_step_m) || surface.range_step_m <= 0 ||
        surface.angle_min_deg !== -60 || surface.angle_max_deg !== 60 ||
        surface.angle_step_deg !== 2 ||
        surface.angular_resolution_deg !== 29 ||
        surface.complete_frame_adc !== false ||
        !Array.isArray(surface.cells) || !Array.isArray(surface.peaks)) {
      return invalid();
    }
    const validatePosition = (item) => {
      const radians = item.azimuth_deg * Math.PI / 180;
      return Number.isFinite(item.range_m) && item.range_m > 0 &&
        item.range_m <= 0.30 + 1e-9 &&
        Number.isFinite(item.azimuth_deg) &&
        item.azimuth_deg >= surface.angle_min_deg - 1e-9 &&
        item.azimuth_deg <= surface.angle_max_deg + 1e-9 &&
        Number.isFinite(item.forward_m) && item.forward_m >= 0 &&
        Number.isFinite(item.lateral_m) &&
        Math.abs(item.forward_m - item.range_m * Math.cos(radians)) <= 2e-5 &&
        Math.abs(item.lateral_m - item.range_m * Math.sin(radians)) <= 2e-5;
    };
    const occupiedCells = new Set();
    const cells = surface.cells.map((cell) => {
      if (!isPlainObject(cell) || !Number.isInteger(cell.range_index) ||
          cell.range_index < 1 || !Number.isInteger(cell.azimuth_index) ||
          cell.azimuth_index < 0 || cell.azimuth_index > 60 ||
          !validatePosition(cell) || !Number.isFinite(cell.intensity) ||
          cell.intensity < 0 || cell.intensity > 1 ||
          !Number.isFinite(cell.relative_db) ||
          !Number.isFinite(cell.coherence) || cell.coherence < 0 ||
          cell.coherence > 1) {
        return invalid();
      }
      const key = `${cell.range_index}:${cell.azimuth_index}`;
      if (occupiedCells.has(key)) return invalid();
      occupiedCells.add(key);
      return deepClone(cell);
    });
    const peaks = surface.peaks.map((peak) => {
      if (!isPlainObject(peak) || !Number.isInteger(peak.range_index) ||
          peak.range_index < 3 || !validatePosition(peak) ||
          !Number.isFinite(peak.relative_db) ||
          !Number.isFinite(peak.coherence) || peak.coherence < 0 ||
          peak.coherence > 1 || !Number.isFinite(peak.confidence) ||
          peak.confidence < 0 || peak.confidence > 1 ||
          !Number.isInteger(peak.hits) || peak.hits < 1 ||
          peak.hits > surface.history_frames ||
          typeof peak.verified !== "boolean" ||
          peak.verified !== (peak.hits >= surface.required_hits) ||
          !Number.isInteger(peak.age_ms) || peak.age_ms < 0 ||
          peak.angular_uncertainty_deg !== 14.5) {
        return invalid();
      }
      return deepClone(peak);
    });
    return {
      source: surface.source,
      directionSemantics: surface.direction_semantics,
      processing: surface.processing,
      phaseCalibration: surface.phase_calibration,
      historyFrames: surface.history_frames,
      requiredHits: surface.required_hits,
      retention: surface.retention,
      rangeStepM: surface.range_step_m,
      angleMinDeg: surface.angle_min_deg,
      angleMaxDeg: surface.angle_max_deg,
      angleStepDeg: surface.angle_step_deg,
      angularResolutionDeg: surface.angular_resolution_deg,
      completeFrameAdc: surface.complete_frame_adc,
      cells,
      peaks,
    };
  }

  function parseRadarEvidenceFallback(snapshot, cause) {
    if (!snapshot || !["live", "degraded"].includes(snapshot.status)) {
      throw new Error("evidence fallback requires a live radar transport");
    }
    let surface;
    try {
      surface = parseReflectivitySurface(snapshot.scene || {});
    } catch (_surfaceError) {
      surface = rawHeatmapEvidenceSurface(snapshot);
    }
    if (!surface || !Array.isArray(surface.cells) || surface.cells.length < 1) {
      throw new Error("no current radar evidence for fallback");
    }
    let adcRangeAngleSurface = null;
    try {
      adcRangeAngleSurface = parseAdcRangeAngleSurface(snapshot.scene || {});
    } catch (_adcError) {
      // The display-only ADC overlay must never suppress valid heatmap evidence.
    }
    return {
      blocked: false,
      evidenceOnly: true,
      sceneContractFallback: true,
      poseMode: "sensor_relative_current_frame",
      grid: null,
      gridMeta: null,
      probabilisticLocalMap: null,
      reflectivitySurface: surface,
      adcRangeAngleSurface,
      reflectivityCorridor: { status: "unavailable", walls: [] },
      reflectivityBoundaries: { status: "unavailable", walls: [] },
      reflectivityContours: { status: "unavailable", segments: [] },
      sceneWarnings: ["scene_contract_evidence_fallback"],
      groundFilter: null,
      filteredTracks: [], tracks: [], referenceTracks: [],
      diagnostics: deepClone(snapshot.scene?.diagnostics || {}),
      plannedPath: {
        status: "unavailable", stability_status: "evidence_only",
        length_m: 0, points: [], minimum_passage_width_m: 0.21,
        blocked_previous_path: null,
      },
      motionCompensation: { status: "unavailable", applied: false },
      accumulatedObstacles: [], rollingMapObstacles: [], rollingMap: null,
      travelTrail: { radiusM: 0.50, resolutionM: 0.01,
        source: "unavailable", points: [[0, 0]] },
      obstacleMemoryWindowMs: 0,
      hazard: { level: "UNKNOWN", threshold_m: 0.10,
        nearest_confirmed_m: null, reason: "scene_contract_fallback" },
      hazardCopy: "CURRENT RADAR EVIDENCE ONLY · PATH/HAZARD UNVERIFIED",
    };
  }

  function parseReflectivityContours(scene) {
    const contours = scene.reflectivity_contours;
    const invalid = () => {
      throw new Error("invalid reflectivity contour contract");
    };
    if (!isPlainObject(contours) ||
        !["unavailable", "confirmed"].includes(contours.status) ||
        contours.method !== "calibrated_nearest_reflection_segments" ||
        contours.history_frames !== 5 || contours.required_hits !== 3 ||
        contours.max_range_m !== 0.30 || !Array.isArray(contours.segments)) {
      return invalid();
    }
    if (contours.status === "unavailable") {
      if (contours.segments.length !== 0) return invalid();
      return {
        status: contours.status,
        method: contours.method,
        historyFrames: contours.history_frames,
        requiredHits: contours.required_hits,
        maxRangeM: contours.max_range_m,
        segments: [],
      };
    }
    if (contours.segments.length < 1) return invalid();
    const segments = contours.segments.map((segment, index) => {
      if (!isPlainObject(segment) ||
          segment.segment_id !== `contour_${index + 1}` ||
          segment.evidence_kind !== "calibrated_heatmap_prominence" ||
          !Number.isInteger(segment.hits) ||
          segment.hits < contours.required_hits ||
          segment.hits > contours.history_frames ||
          !Number.isFinite(segment.confidence) || segment.confidence < 0 ||
          segment.confidence > 1 ||
          !Number.isFinite(segment.heading_deg) ||
          Math.abs(segment.heading_deg) > 90.0 + 1e-9 ||
          !Number.isInteger(segment.support_cells) ||
          segment.support_cells < 3 ||
          !Number.isFinite(segment.span_m) || segment.span_m < 0.030 ||
          segment.span_m > contours.max_range_m ||
          !Number.isFinite(segment.mean_prominence_db) ||
          segment.mean_prominence_db <= 0.0 ||
          !validPointPair(segment.start) || !validPointPair(segment.end) ||
          segment.start[0] < 0 || segment.end[0] < 0 ||
          Math.hypot(...segment.start) > contours.max_range_m + 1e-9 ||
          Math.hypot(...segment.end) > contours.max_range_m + 1e-9) {
        return invalid();
      }
      const deltaForward = segment.end[0] - segment.start[0];
      const deltaLateral = segment.end[1] - segment.start[1];
      const endpointSpan = Math.hypot(deltaForward, deltaLateral);
      const endpointHeading = Math.atan2(
        deltaLateral, deltaForward,
      ) * 180 / Math.PI;
      if (Math.abs(endpointSpan - segment.span_m) > 2e-5 ||
          Math.abs(endpointHeading - segment.heading_deg) > 5e-4) {
        return invalid();
      }
      return {
        segmentId: segment.segment_id,
        evidenceKind: segment.evidence_kind,
        hits: segment.hits,
        confidence: segment.confidence,
        headingDeg: segment.heading_deg,
        supportCells: segment.support_cells,
        spanM: segment.span_m,
        meanProminenceDb: segment.mean_prominence_db,
        start: [...segment.start],
        end: [...segment.end],
      };
    });
    return {
      status: contours.status,
      method: contours.method,
      historyFrames: contours.history_frames,
      requiredHits: contours.required_hits,
      maxRangeM: contours.max_range_m,
      segments,
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
          Math.abs(endpointHeading - wall.heading_deg) > 5e-4 ||
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
    const lateralAt = (wall, forward) => {
      const fraction = (
        (forward - wall.start[0]) /
        (wall.end[0] - wall.start[0])
      );
      return wall.start[1] + fraction * (
        wall.end[1] - wall.start[1]
      );
    };
    const overlapStart = Math.max(
      walls[0].start[0],
      walls[1].start[0],
    );
    const overlapEnd = Math.min(walls[0].end[0], walls[1].end[0]);
    const centerlineHeading = () => {
      const nearCenterLateral = walls.reduce(
        (sum, wall) => sum + lateralAt(wall, overlapStart),
        0,
      ) / 2;
      const farCenterLateral = walls.reduce(
        (sum, wall) => sum + lateralAt(wall, overlapEnd),
        0,
      ) / 2;
      return Math.atan2(
        farCenterLateral - nearCenterLateral,
        overlapEnd - overlapStart,
      ) * 180 / Math.PI;
    };
    if (corridor.geometry === "parallel") {
      if (Math.abs(walls[0].headingDeg - walls[1].headingDeg) > 10 ||
          Math.abs(
            Math.abs(walls[0].normalOffsetM - walls[1].normalOffsetM) -
            corridor.width_m
          ) > 2e-5 || overlapEnd <= overlapStart || Math.abs(
            centerlineHeading() - corridor.measured_heading_deg
          ) > 2e-4) {
        return invalid();
      }
    } else {
      const nearLaterals = walls.map(
        (wall) => lateralAt(wall, overlapStart),
      );
      const farLaterals = walls.map(
        (wall) => lateralAt(wall, overlapEnd),
      );
      const nearWidth = nearLaterals[1] - nearLaterals[0];
      const farWidth = farLaterals[1] - farLaterals[0];
      const centerHeading = centerlineHeading();
      // A steep but physically valid wall pair can span four independent
      // range rows while projecting to less than 7.5 cm on the forward axis.
      // The backend already verifies row count, wall span, residual and width
      // reduction, so only reject pairs with no common forward interval here.
      if (overlapEnd <= overlapStart ||
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

  function parseReflectivityBoundaries(scene) {
    const boundaries = scene.reflectivity_boundaries;
    const invalid = () => {
      throw new Error("invalid reflectivity boundary contract");
    };
    if (!isPlainObject(boundaries) ||
        !["unavailable", "confirmed"].includes(boundaries.status) ||
        boundaries.method !== "independent_boundary_fit" ||
        boundaries.history_frames !== 5 || boundaries.required_hits !== 3 ||
        !Array.isArray(boundaries.walls) || boundaries.walls.length > 2) {
      return invalid();
    }
    if (boundaries.status === "unavailable") {
      if (boundaries.walls.length !== 0) return invalid();
      return {
        status: boundaries.status,
        method: boundaries.method,
        historyFrames: boundaries.history_frames,
        requiredHits: boundaries.required_hits,
        walls: [],
      };
    }
    if (boundaries.walls.length < 1) return invalid();
    const walls = boundaries.walls.map((wall) => {
      const pointEvidence = wall?.evidence_kind === "point";
      const heatmapEvidence = wall?.evidence_kind === "heatmap";
      const minimumSupport = pointEvidence ? 3 : 5;
      const minimumSpanM = pointEvidence ? 0.10 : 0.18;
      const minimumIntensity = pointEvidence ? 0.50 : 0.90;
      if (!isPlainObject(wall) ||
          (!pointEvidence && !heatmapEvidence) ||
          !["left", "right"].includes(wall.side) ||
          wall.wall_id !== `boundary_${wall.side}` ||
          !Number.isInteger(wall.hits) ||
          wall.hits < boundaries.required_hits ||
          wall.hits > boundaries.history_frames ||
          !Number.isFinite(wall.confidence) || wall.confidence < 0 ||
          wall.confidence > 1 || !Number.isFinite(wall.heading_deg) ||
          Math.abs(wall.heading_deg) > 75 ||
          !Number.isFinite(wall.normal_offset_m) ||
          !Number.isInteger(wall.support_cells) ||
          wall.support_cells < minimumSupport ||
          !Number.isFinite(wall.span_m) || wall.span_m < minimumSpanM ||
          !Number.isFinite(wall.mean_intensity) ||
          wall.mean_intensity < minimumIntensity ||
          wall.mean_intensity > 1 ||
          !validPointPair(wall.start) || !validPointPair(wall.end) ||
          wall.start[0] < 0 || wall.end[0] <= wall.start[0]) {
        return invalid();
      }
      const midpointLateral = (wall.start[1] + wall.end[1]) / 2;
      if ((wall.side === "left" && midpointLateral >= 0) ||
          (wall.side === "right" && midpointLateral <= 0)) {
        return invalid();
      }
      const deltaForward = wall.end[0] - wall.start[0];
      const deltaLateral = wall.end[1] - wall.start[1];
      const endpointSpan = Math.hypot(deltaForward, deltaLateral);
      const endpointHeading = Math.atan2(
        deltaLateral, deltaForward,
      ) * 180 / Math.PI;
      const headingRad = wall.heading_deg * Math.PI / 180;
      const endpointNormalOffset =
        -Math.sin(headingRad) * wall.start[0] +
        Math.cos(headingRad) * wall.start[1];
      if (Math.abs(endpointSpan - wall.span_m) > 2e-5 ||
          Math.abs(endpointHeading - wall.heading_deg) > 5e-4 ||
          Math.abs(endpointNormalOffset - wall.normal_offset_m) > 2e-5) {
        return invalid();
      }
      return {
        wallId: wall.wall_id,
        evidenceKind: wall.evidence_kind,
        side: wall.side,
        hits: wall.hits,
        confidence: wall.confidence,
        headingDeg: wall.heading_deg,
        normalOffsetM: wall.normal_offset_m,
        supportCells: wall.support_cells,
        spanM: wall.span_m,
        meanIntensity: wall.mean_intensity,
        start: [...wall.start],
        end: [...wall.end],
      };
    });
    if (new Set(walls.map((wall) => wall.side)).size !== walls.length) {
      return invalid();
    }
    return {
      status: boundaries.status,
      method: boundaries.method,
      historyFrames: boundaries.history_frames,
      requiredHits: boundaries.required_hits,
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
      "clear", "limited", "blocked_near_field", "turn_side_unresolved",
      "initializing", "reset",
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
        (["blocked_near_field", "turn_side_unresolved"].includes(path.status) &&
          (path.length_m !== 0 || path.points.length !== 1))) {
      throw new Error("invalid planned path contract");
    }
    const rawTurnCandidates = path.turn_candidates == null
      ? [] : path.turn_candidates;
    if (!Array.isArray(rawTurnCandidates) ||
        rawTurnCandidates.some((candidate) =>
          !isPlainObject(candidate) ||
          !["left", "right"].includes(candidate.direction) ||
          !Number.isFinite(candidate.angle_deg) ||
          !Number.isFinite(candidate.cost) || candidate.cost < 0 ||
          !Number.isFinite(candidate.goal_forward_m) ||
          !Number.isFinite(candidate.goal_lateral_m) ||
          !Array.isArray(candidate.points) || candidate.points.length < 2 ||
          candidate.points.some((point) => !validPointPair(point)) ||
          Math.abs(candidate.points[0][0]) > 1e-9 ||
          Math.abs(candidate.points[0][1]) > 1e-9 ||
          candidate.points.slice(1).some((point, index) =>
            Math.abs(Math.hypot(
              point[0] - candidate.points[index][0],
              point[1] - candidate.points[index][1],
            ) - 0.01) > 0.001) ||
          (candidate.direction === "left" && candidate.angle_deg >= 0) ||
          (candidate.direction === "right" && candidate.angle_deg <= 0))) {
      throw new Error("invalid planned path contract");
    }
    if (path.status === "turn_side_unresolved" &&
        (rawTurnCandidates.length !== 2 ||
          new Set(rawTurnCandidates.map((candidate) => candidate.direction))
            .size !== 2)) {
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
      turn_candidates: deepClone(rawTurnCandidates),
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
    const hasProbabilityMap = Object.prototype.hasOwnProperty.call(
      scene, "probabilistic_local_map",
    );
    if (!["ok", "synthetic", "spi_raw"].includes(scene.calibration_status)) {
      try {
        const surface = hasProbabilityMap
          ? parseReflectivitySurface(scene) : null;
        if (surface && surface.source === "full_range_azimuth_heatmap" &&
            surface.cells.length > 0) {
          const grid = decodeSceneV1Grid(scene.grid);
          let adcRangeAngleSurface = null;
          const sceneWarnings = [`${scene.calibration_status}_evidence_only`];
          try {
            adcRangeAngleSurface = parseAdcRangeAngleSurface(scene);
          } catch (_adcError) {
            sceneWarnings.push("adc_range_angle_surface_invalid");
          }
          return {
            blocked: false,
            evidenceOnly: true,
            calibrationStatus: scene.calibration_status,
            poseMode: scene.pose_mode,
            grid,
            gridMeta: deepClone(scene.grid),
            probabilisticLocalMap: null,
            reflectivitySurface: surface,
            adcRangeAngleSurface,
            reflectivityCorridor: {
              status: "unavailable", walls: [],
            },
            reflectivityBoundaries: {
              status: "unavailable", walls: [],
            },
            reflectivityContours: {
              status: "unavailable", segments: [],
            },
            sceneWarnings,
            groundFilter: parseGroundFilter(
              scene.ground_filter, hasProbabilityMap,
            ),
            filteredTracks: [],
            tracks: [],
            referenceTracks: [],
            diagnostics: deepClone(scene.diagnostics || {}),
            plannedPath: {
              status: "unavailable",
              stability_status: "evidence_only",
              length_m: 0,
              points: [],
              minimum_passage_width_m: 0.21,
              blocked_previous_path: null,
            },
            motionCompensation: { status: "unavailable", applied: false },
            accumulatedObstacles: [],
            rollingMapObstacles: [],
            rollingMap: null,
            travelTrail: {
              radiusM: 0.50,
              resolutionM: 0.01,
              source: "unavailable",
              points: [[0, 0]],
            },
            obstacleMemoryWindowMs: 0,
            hazard: {
              level: "UNKNOWN",
              threshold_m: 0.10,
              nearest_confirmed_m: null,
              reason: scene.calibration_status,
            },
            hazardCopy: "반사강도만 표시 · 충돌 판단은 미검증",
          };
        }
      } catch (_error) {
        // A malformed evidence payload remains blocked below.
      }
      return { blocked: true, reason: scene.calibration_status };
    }
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
    const sceneWarnings = [];
    let adcRangeAngleSurface = null;
    try {
      adcRangeAngleSurface = parseAdcRangeAngleSurface(scene);
    } catch (_error) {
      sceneWarnings.push("adc_range_angle_surface_invalid");
    }
    let reflectivityCorridor = null;
    let reflectivityBoundaries = null;
    if (hasProbabilityMap) {
      try {
        reflectivityCorridor = parseReflectivityCorridor(scene);
      } catch (error) {
        if (reflectivitySurface?.source !==
            "full_range_azimuth_heatmap") {
          throw error;
        }
        reflectivityCorridor = {
          status: "unavailable",
          method: "deterministic_boundary_fit",
          geometry: null,
          historyFrames: 5,
          requiredHits: 3,
          hits: 0,
          measuredHeadingDeg: null,
          guidanceHeadingDeg: null,
          widthM: null,
          confidence: 0,
          walls: [],
        };
        sceneWarnings.push("reflectivity_corridor_invalid");
      }
      try {
        reflectivityBoundaries = parseReflectivityBoundaries(scene);
      } catch (error) {
        if (reflectivitySurface?.source !==
            "full_range_azimuth_heatmap") {
          throw error;
        }
        reflectivityBoundaries = {
          status: "unavailable",
          method: "independent_boundary_fit",
          historyFrames: 5,
          requiredHits: 3,
          walls: [],
        };
        sceneWarnings.push("reflectivity_boundaries_invalid");
      }
    }
    let reflectivityContours = null;
    if (hasProbabilityMap) {
      try {
        reflectivityContours = parseReflectivityContours(scene);
      } catch (_error) {
        reflectivityContours = {
          status: "unavailable",
          method: "calibrated_nearest_reflection_segments",
          historyFrames: 5,
          requiredHits: 3,
          maxRangeM: 0.30,
          segments: [],
        };
        sceneWarnings.push("reflectivity_contours_invalid");
      }
    }
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
      adcRangeAngleSurface,
      reflectivityCorridor,
      reflectivityBoundaries,
      reflectivityContours,
      sceneWarnings,
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
    parseRadarEvidenceFallback,
    parseAdcRangeAngleSurface,
    makeMapTransform,
    projectMapPoint,
  };
});
