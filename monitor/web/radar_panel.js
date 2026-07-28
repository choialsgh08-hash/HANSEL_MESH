(function () {
  "use strict";

  const STATUS_LABELS = {
    live: "LIVE",
    degraded: "DEGRADED",
    stale: "STALE",
    fault: "FAULT",
    waiting: "WAITING",
    replay_end: "REPLAY END",
  };
  const UI_BUILD_ID = "20260726-depth-camera-r8";
  const SECTOR_NAMES = ["좌측 끝", "좌측", "정면", "우측", "우측 끝"];
  const FOV_MIN_DEG = -70;
  const FOV_MAX_DEG = 70;
  const ELEVATION_MIN_DEG = -60;
  const ELEVATION_MAX_DEG = 60;
  const SECTOR_WIDTH_DEG = (FOV_MAX_DEG - FOV_MIN_DEG) / 5;
  const VALID_MIN_RANGE_M = 0.01;
  const VALID_MAX_RANGE_M = 7.5;
  const DANGER_RANGE_M = 0.1;
  const CAUTION_RANGE_M = 0.25;
  const HEATMAP_DECAY_DB_PER_WINDOW = 24;
  const WALL_TRACK_HOLD_MS = 900;
  const WALL_CONTOUR_MIN_M = 0.12;

  class RadarPanel {
    constructor(root, options) {
      this.root = root;
      this.canvas = root.querySelector("canvas");
      this.ctx = this.canvas.getContext("2d");
      this.viewSelect = document.querySelector("#view-select");
      this.rangeSelect = document.querySelector("#range-select");
      this.persistenceSelect = document.querySelector("#persistence-select");
      this.rawToggle = document.querySelector("#raw-toggle");
      this.outlineToggle = document.querySelector("#outline-toggle");
      this.fullscreenButton = document.querySelector("#fullscreen-button");
      this.state = null;
      this.fetchFailed = false;
      this.options = Object.assign({ endpoint: "/api/radar" }, options || {});
      this.lastHeatmapKey = null;
      this.lastHeatmapIdentity = null;
      this.smoothedHeatmap = null;
      this.heatmapMeta = null;
      this.lastHeatmapAt = null;
      this.heatmapSourceAt = null;
      this.heatmapClearReason = "waiting";
      this.depthContourTrack = [];
      this.lastSectorStats = this.emptySectorStats();

      this.resizeObserver = new ResizeObserver(() => this.draw());
      this.resizeObserver.observe(this.canvas);
      this.viewSelect.addEventListener("change", () => {
        this.updateViewLabels();
        this.draw();
      });
      this.rangeSelect.addEventListener("change", () => {
        this.updateSectors();
        this.draw();
      });
      this.persistenceSelect.addEventListener("change", () => {
        this.updateSectors();
        this.draw();
      });
      this.rawToggle.addEventListener("change", () => this.draw());
      this.outlineToggle.addEventListener("change", () => {
        this.updateViewLabels();
        this.draw();
      });
      this.fullscreenButton.addEventListener("click", async () => {
        if (document.fullscreenElement) {
          await document.exitFullscreen();
        } else if (this.root.requestFullscreen) {
          await this.root.requestFullscreen();
        }
      });
      document.addEventListener("fullscreenchange", () => {
        this.fullscreenButton.textContent = document.fullscreenElement
          ? "전체화면 종료"
          : "레이더 화면 전체화면";
        this.draw();
      });
      this.updateViewLabels();
    }

    async start() {
      while (true) {
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), 1200);
        try {
          const response = await fetch(this.options.endpoint, {
            cache: "no-store",
            signal: controller.signal,
          });
          if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
          }
          const nextState = await response.json();
          if (
            nextState.ui_build_id &&
            nextState.ui_build_id !== UI_BUILD_ID
          ) {
            window.location.reload();
            return;
          }
          this.state = nextState;
          this.fetchFailed = false;
          this.rememberHeatmap();
          this.updateText();
          this.updateSectors();
          this.draw();
        } catch (error) {
          this.fetchFailed = true;
          this.updateText(error);
          this.updateSectors();
          this.draw();
        } finally {
          window.clearTimeout(timeout);
        }
        await new Promise((resolve) => window.setTimeout(resolve, 100));
      }
    }

    rememberHeatmap() {
      const frame = this.state && this.state.frame;
      const payload = frame && frame.heatmap;
      const status = this.state ? this.state.status : "waiting";
      if (["waiting", "stale", "fault", "replay_end"].includes(status)) {
        this.clearHeatmap("stale_or_invalid_source");
        return;
      }
      if (!payload || !payload.data_base64) {
        this.clearHeatmap(
          frame && frame.heatmap_status
            ? frame.heatmap_status
            : "latest_frame_missing_heatmap",
        );
        return;
      }
      const identity = [
        frame.producer_id || "",
        frame.profile_id || "",
        payload.range_bins,
        payload.azimuth_bins,
        payload.range_step_m,
        payload.azimuth_layout || "",
        payload.lambda_over_d_x || "",
        payload.valid_min_range_m || "",
        payload.valid_max_range_m || "",
      ].join(":");
      const key = `${frame.producer_id || ""}:${frame.seq}:${payload.data_base64.length}`;
      if (key === this.lastHeatmapKey) {
        return;
      }
      const decoded = this.decodeHeatmapDb(payload);
      if (!decoded) {
        this.clearHeatmap("invalid_heatmap_payload");
        return;
      }
      const now = Date.now();
      const sameShape =
        this.smoothedHeatmap &&
        this.heatmapMeta &&
        this.lastHeatmapIdentity === identity &&
        this.heatmapMeta.range_bins === payload.range_bins &&
        this.heatmapMeta.azimuth_bins === payload.azimuth_bins &&
        this.heatmapMeta.range_step_m === payload.range_step_m;
      if (!sameShape) {
        this.smoothedHeatmap = new Float32Array(decoded);
      } else {
        const persistenceMs = Number(this.persistenceSelect.value);
        const elapsed = Math.max(0, now - this.lastHeatmapAt);
        const decayDb =
          (HEATMAP_DECAY_DB_PER_WINDOW * elapsed) / persistenceMs;
        const floorDb = Number(payload.floor_db);
        for (let index = 0; index < decoded.length; index += 1) {
          this.smoothedHeatmap[index] = Math.max(
            decoded[index],
            Math.max(floorDb, this.smoothedHeatmap[index] - decayDb),
          );
        }
      }
      this.heatmapMeta = payload;
      this.lastHeatmapKey = key;
      this.lastHeatmapIdentity = identity;
      this.lastHeatmapAt = now;
      this.heatmapSourceAt =
        now -
        (this.state && Number.isFinite(this.state.age_ms)
          ? this.state.age_ms
          : 0);
      this.heatmapClearReason = null;
      this.updateDepthContourTrack();
    }

    clearHeatmap(reason) {
      this.lastHeatmapKey = null;
      this.lastHeatmapIdentity = null;
      this.smoothedHeatmap = null;
      this.heatmapMeta = null;
      this.lastHeatmapAt = null;
      this.heatmapSourceAt = null;
      this.heatmapClearReason = reason;
      this.depthContourTrack = [];
    }

    decodeHeatmapDb(payload) {
      try {
        const binary = window.atob(payload.data_base64);
        const expected = payload.range_bins * payload.azimuth_bins;
        const floorDb = Number(payload.floor_db);
        const ceilingDb = Number(payload.ceiling_db);
        if (
          binary.length !== expected ||
          !Number.isFinite(floorDb) ||
          !Number.isFinite(ceilingDb) ||
          ceilingDb <= floorDb
        ) {
          return null;
        }
        const values = new Float32Array(expected);
        const spanDb = ceilingDb - floorDb;
        for (let index = 0; index < expected; index += 1) {
          values[index] =
            floorDb + (binary.charCodeAt(index) / 255) * spanDb;
        }
        return values;
      } catch (error) {
        return null;
      }
    }

    heatmapAgeMs() {
      return this.heatmapSourceAt === null
        ? Infinity
        : Math.max(0, Date.now() - this.heatmapSourceAt);
    }

    isHeatmapFresh() {
      if (!this.smoothedHeatmap || !this.heatmapMeta) {
        return false;
      }
      const staleAfter =
        this.state &&
        this.state.limits &&
        Number.isFinite(this.state.limits.stale_after_ms)
          ? this.state.limits.stale_after_ms
          : 750;
      return this.heatmapAgeMs() <= staleAfter;
    }

    decayedHeatmapDb(index, now) {
      const floorDb = Number(this.heatmapMeta.floor_db);
      const persistenceMs = Number(this.persistenceSelect.value);
      const elapsed = Math.max(0, now - this.lastHeatmapAt);
      const decayDb =
        (HEATMAP_DECAY_DB_PER_WINDOW * elapsed) / persistenceMs;
      return Math.max(floorDb, this.smoothedHeatmap[index] - decayDb);
    }

    updateText(error) {
      const state = this.state;
      const status = this.fetchFailed
        ? "fault"
        : state
          ? state.status
          : "waiting";
      const badge = document.querySelector("#radar-status");
      badge.dataset.status = status;
      badge.textContent = this.fetchFailed
        ? "HTTP LOST"
        : STATUS_LABELS[status] || String(status).toUpperCase();

      document.querySelector("#warning-text").textContent = this.fetchFailed
        ? `화면 데이터 연결 실패 · ${error ? error.message : "unknown"} · 즉시 정지`
        : state
          ? state.warning
          : "레이더 프레임 대기 중 · 주행하지 마세요";

      const frame = state && state.frame;
      const occupancy = state && state.occupancy;
      const counters = state && state.counters;
      const heatmap = frame && frame.heatmap;
      let mode = "HEATMAP 대기";
      if (frame && frame.heatmap_status === "disabled_nondefault_axes") {
        mode = "HEATMAP OFF · AXIS";
        document.querySelector("#warning-text").textContent =
          "장착축이 기본값이 아니므로 RAW HEATMAP을 차단했습니다 · 포인트 증거만 사용";
      } else if (heatmap && heatmap.source === "radar") {
        mode = "RAW HEATMAP + 3D";
      } else if (heatmap) {
        mode = "DEMO HEATMAP";
      } else if (occupancy && occupancy.points && occupancy.points.length) {
        mode = "POINT EVIDENCE";
      }
      document.querySelector("#radar-mode").textContent = mode;

      const nearest =
        frame && Number.isFinite(frame.nearest_corridor_m)
          ? `${frame.nearest_corridor_m.toFixed(2)} m`
          : "--";
      this.setMetric(
        "metric-nearest",
        nearest,
        frame ? "중앙 ±0.6 m 반사" : "중앙 ±0.6 m",
      );
      this.setMetric(
        "metric-evidence",
        occupancy && Array.isArray(occupancy.points)
          ? String(occupancy.points.length)
          : "--",
        occupancy
          ? `${occupancy.frames}프레임 · 높이 ${occupancy.points.filter((point) => Math.abs(Number(point[2])) > 0.001).length}점`
          : "최근 누적 반사",
      );
      this.setMetric(
        "metric-fps",
        state && Number.isFinite(state.fps) ? state.fps.toFixed(1) : "--",
        "frame/s",
      );
      this.setMetric(
        "metric-age",
        state && Number.isFinite(state.age_ms) ? String(state.age_ms) : "--",
        "ms since frame",
      );

      document.querySelector("#profile-value").textContent =
        frame && frame.profile_id ? frame.profile_id : "--";
      document.querySelector("#calibration-value").textContent =
        frame && frame.calibration_id
          ? frame.calibration_id
          : "uncalibrated";
      document.querySelector("#frame-value").textContent = frame
        ? `${frame.number} · ${frame.display_point_count}점`
        : "--";
      document.querySelector("#gap-value").textContent = counters
        ? String(
            (counters.frame_gaps_total || 0) +
              Math.max(
                counters.sensor_sequence_gaps_total || 0,
                counters.writer_drops_total || 0,
              ),
          )
        : "--";
      document.querySelector("#axis-value").textContent = state
        ? `${state.axes.forward_sign > 0 ? "+" : "-"}${state.axes.forward_axis.toUpperCase()} 전방 · ` +
          `${state.axes.lateral_sign > 0 ? "+" : "-"}${state.axes.lateral_axis.toUpperCase()} 우측`
        : "+Y 전방 · +X 우측";

      const uncalibrated =
        !frame ||
        !frame.calibration_id ||
        frame.calibration_id === "uncalibrated";
      document.querySelector("#axis-warning").hidden = !uncalibrated;
    }

    setMetric(id, value, detail) {
      const element = document.querySelector(`#${id}`);
      element.querySelector("strong").textContent = value;
      element.querySelector("span").textContent = detail;
    }

    draw() {
      const canvas = this.canvas;
      const rect = canvas.getBoundingClientRect();
      const ratio = Math.max(1, window.devicePixelRatio || 1);
      const width = Math.max(320, Math.floor(rect.width));
      const height = Math.max(420, Math.floor(rect.height));
      if (
        canvas.width !== Math.floor(width * ratio) ||
        canvas.height !== Math.floor(height * ratio)
      ) {
        canvas.width = Math.floor(width * ratio);
        canvas.height = Math.floor(height * ratio);
      }
      const ctx = this.ctx;
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      ctx.clearRect(0, 0, width, height);

      const maxRange = Number(this.rangeSelect.value);
      const status = this.currentStatus();
      const unsafe = ["waiting", "stale", "fault", "replay_end"].includes(status);

      if (this.viewSelect.value === "camera") {
        this.drawDepthCamera(ctx, width, height, maxRange, unsafe);
      } else if (this.viewSelect.value === "perspective") {
        this.drawPerspective(ctx, width, height, maxRange, unsafe);
      } else {
        this.drawTopView(ctx, width, height, maxRange, unsafe);
      }
      this.drawOverlay(ctx, width, height, status);
    }

    updateViewLabels() {
      const mode = this.viewSelect.value;
      document.querySelector("#view-tag").textContent =
        mode === "camera"
          ? "ROBOT POV · MONO DEPTH"
          : mode === "perspective"
            ? "3D HEMISPHERE MAP"
            : "2D TOP VIEW";
      document.querySelector("#height-tag").hidden = mode === "top";
      document.querySelector("#outline-tag").hidden =
        mode === "top" || !this.outlineToggle.checked;
    }

    drawTopView(ctx, width, height, maxRange, unsafe) {
      const robotX = width / 2;
      const robotY = height - 64;
      const verticalScale = (height - 90) / maxRange;
      const horizontalScale =
        (width * 0.49) /
        (maxRange * Math.sin((FOV_MAX_DEG * Math.PI) / 180));
      const scale = Math.max(20, Math.min(verticalScale, horizontalScale));
      this.drawBackground(ctx, width, height, robotX, robotY, scale, maxRange);
      this.drawIntensity(ctx, robotX, robotY, scale, maxRange, unsafe);
      this.drawSectorArcs(ctx, robotX, robotY, scale);
      if (this.rawToggle.checked) {
        this.drawRawPoints(ctx, robotX, robotY, scale, maxRange, unsafe);
      }
      this.drawRobot(ctx, robotX, robotY, unsafe);
    }

    currentStatus() {
      if (this.fetchFailed) {
        return "fault";
      }
      return this.state ? this.state.status : "waiting";
    }

    drawDepthCamera(ctx, width, height, maxRange, unsafe) {
      const contour = this.stableDepthContour(maxRange);
      this.drawDepthCameraBackground(ctx, width, height, unsafe);
      if (this.isHeatmapFresh()) {
        this.drawDepthCameraEchoColumns(
          ctx,
          width,
          height,
          maxRange,
          unsafe,
          contour,
        );
      }
      this.drawDepthCameraPoints(ctx, width, height, maxRange, unsafe);
      if (this.rawToggle.checked) {
        this.drawDepthCameraLatestPoints(ctx, width, height, maxRange, unsafe);
      }
      this.drawDepthCameraHud(ctx, width, height, maxRange, unsafe, contour);
    }

    drawDepthCameraBackgroundLegacy(ctx, width, height, unsafe) {
      const background = ctx.createLinearGradient(0, 0, 0, height);
      background.addColorStop(0, "#02070b");
      background.addColorStop(0.5, "#07171c");
      background.addColorStop(1, "#010406");
      ctx.fillStyle = background;
      ctx.fillRect(0, 0, width, height);

      const horizonY = height * 0.48;
      ctx.save();
      ctx.lineWidth = 1;
      ctx.font = "9px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      [-60, -30, 0, 30, 60].forEach((azimuth) => {
        const x = this.cameraXForAngle(azimuth, width);
        ctx.strokeStyle =
          azimuth === 0
            ? "rgba(255, 193, 91, 0.25)"
            : "rgba(117, 205, 192, 0.1)";
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
        ctx.fillStyle = "rgba(179, 211, 205, 0.5)";
        ctx.fillText(`${azimuth > 0 ? "+" : ""}${azimuth}°`, x, height - 76);
      });
      [-45, -30, 0, 30, 45].forEach((elevation) => {
        const y = this.cameraYForElevation(elevation, height);
        ctx.strokeStyle =
          elevation === 0
            ? "rgba(255, 193, 91, 0.22)"
            : "rgba(117, 205, 192, 0.1)";
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
        ctx.fillStyle = "rgba(179, 211, 205, 0.46)";
        ctx.textAlign = "left";
        ctx.fillText(
          `${elevation > 0 ? "+" : ""}${elevation}°`,
          8,
          Math.max(12, Math.min(height - 82, y - 4)),
        );
        ctx.textAlign = "center";
      });

      const horizonGlow = ctx.createLinearGradient(0, horizonY - 70, 0, horizonY + 70);
      horizonGlow.addColorStop(0, "rgba(30, 123, 132, 0)");
      horizonGlow.addColorStop(0.5, unsafe ? "rgba(170, 170, 170, 0.08)" : "rgba(54, 186, 180, 0.1)");
      horizonGlow.addColorStop(1, "rgba(30, 123, 132, 0)");
      ctx.fillStyle = horizonGlow;
      ctx.fillRect(0, horizonY - 70, width, 140);

      const vignette = ctx.createRadialGradient(
        width / 2,
        height * 0.45,
        Math.min(width, height) * 0.12,
        width / 2,
        height * 0.45,
        Math.max(width, height) * 0.72,
      );
      vignette.addColorStop(0, "rgba(0, 0, 0, 0)");
      vignette.addColorStop(1, "rgba(0, 0, 0, 0.72)");
      ctx.fillStyle = vignette;
      ctx.fillRect(0, 0, width, height);
      ctx.restore();
    }

    drawDepthCameraBackground(ctx, width, height, unsafe) {
      const horizonY = height * 0.48;
      ctx.fillStyle = "#010203";
      ctx.fillRect(0, 0, width, height);

      ctx.save();
      this.depthCameraAperturePath(ctx, width, height);
      ctx.clip();
      const background = ctx.createRadialGradient(
        width / 2,
        horizonY,
        Math.min(width, height) * 0.04,
        width / 2,
        horizonY,
        Math.max(width, height) * 0.62,
      );
      background.addColorStop(0, unsafe ? "#101010" : "#17191a");
      background.addColorStop(0.58, "#080a0b");
      background.addColorStop(1, "#010203");
      ctx.fillStyle = background;
      ctx.fillRect(0, 0, width, height);

      ctx.lineWidth = 1;
      ctx.font = "9px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      [-60, -30, 0, 30, 60].forEach((azimuth) => {
        const x = this.cameraXForAngle(azimuth, width);
        const bend = Math.abs(azimuth) / 60;
        ctx.strokeStyle = azimuth === 0
          ? "rgba(255,255,255,0.20)"
          : "rgba(255,255,255,0.075)";
        ctx.beginPath();
        ctx.moveTo(width / 2, horizonY);
        ctx.quadraticCurveTo(
          x,
          horizonY - height * 0.18 * (1 - bend * 0.3),
          x,
          height * 0.08,
        );
        ctx.moveTo(width / 2, horizonY);
        ctx.quadraticCurveTo(
          x,
          horizonY + height * 0.18 * (1 - bend * 0.3),
          x,
          height * 0.88,
        );
        ctx.stroke();
        ctx.fillStyle = "rgba(255,255,255,0.38)";
        ctx.fillText(`${azimuth > 0 ? "+" : ""}${azimuth}°`, x, height * 0.86);
      });
      [-45, -30, 0, 30, 45].forEach((elevation) => {
        const y = this.cameraYForElevation(elevation, height);
        const radiusScale = 1 - Math.abs(elevation) / 92;
        ctx.strokeStyle = elevation === 0
          ? "rgba(255,255,255,0.20)"
          : "rgba(255,255,255,0.075)";
        ctx.beginPath();
        ctx.ellipse(
          width / 2,
          horizonY,
          width * 0.48 * radiusScale,
          Math.max(1, Math.abs(y - horizonY)),
          0,
          elevation > 0 ? Math.PI : 0,
          elevation > 0 ? Math.PI * 2 : Math.PI,
        );
        ctx.stroke();
        ctx.fillStyle = "rgba(255,255,255,0.36)";
        ctx.textAlign = "left";
        ctx.fillText(
          `${elevation > 0 ? "+" : ""}${elevation}°`,
          width * 0.035,
          Math.max(20, Math.min(height * 0.86, y - 4)),
        );
        ctx.textAlign = "center";
      });
      ctx.restore();

      ctx.save();
      this.depthCameraAperturePath(ctx, width, height);
      ctx.strokeStyle = unsafe
        ? "rgba(255,255,255,0.28)"
        : "rgba(230,242,240,0.52)";
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.restore();
    }

    depthCameraAperturePath(ctx, width, height) {
      ctx.beginPath();
      ctx.ellipse(
        width / 2,
        height * 0.48,
        width * 0.485,
        height * 0.43,
        0,
        0,
        Math.PI * 2,
      );
    }

    drawDepthCameraEchoColumnsLegacy(ctx, width, height, maxRange, unsafe) {
      const meta = this.heatmapMeta;
      const rangeBins = Number(meta.range_bins);
      const azimuthBins = Number(meta.azimuth_bins);
      const rangeStep = Number(meta.range_step_m);
      const validMin = Math.max(
        VALID_MIN_RANGE_M,
        Number.isFinite(meta.valid_min_range_m)
          ? Number(meta.valid_min_range_m)
          : VALID_MIN_RANGE_M,
      );
      const validMax = Math.min(
        maxRange,
        VALID_MAX_RANGE_M,
        Number.isFinite(meta.valid_max_range_m)
          ? Number(meta.valid_max_range_m)
          : VALID_MAX_RANGE_M,
      );
      const levels = this.heatmapRenderLevels(validMin, validMax);
      const now = Date.now();
      const horizonY = height * 0.48;

      ctx.save();
      for (let azimuthIndex = 0; azimuthIndex < azimuthBins; azimuthIndex += 1) {
        let nearest = Infinity;
        let strongest = 0;
        for (let rangeIndex = 0; rangeIndex < rangeBins; rangeIndex += 1) {
          const distance = rangeIndex * rangeStep;
          if (distance < validMin || distance > validMax) {
            continue;
          }
          const db = this.decayedHeatmapDb(
            rangeIndex * azimuthBins + azimuthIndex,
            now,
          );
          const intensity = Math.max(0, Math.min(1, (db - levels.low) / levels.span));
          strongest = Math.max(strongest, intensity);
          if (intensity >= 0.42) {
            nearest = Math.min(nearest, distance);
          }
        }
        if (!Number.isFinite(nearest) || strongest < 0.12) {
          continue;
        }
        const angles = this.heatmapCellAngles(meta, azimuthIndex, azimuthIndex + 1);
        if (!angles) {
          continue;
        }
        const x0 = this.cameraXForAngle(angles[0], width);
        const x1 = this.cameraXForAngle(angles[1], width);
        const centerX = (x0 + x1) / 2;
        const columnWidth = Math.max(12, Math.abs(x1 - x0) * 1.18);
        const alpha = 0.08 + strongest * 0.2;
        const color = this.heatmapColor(
          nearest,
          maxRange,
          strongest,
          unsafe,
          alpha,
        );
        const column = ctx.createLinearGradient(0, height * 0.12, 0, height * 0.82);
        column.addColorStop(0, this.heatmapColor(nearest, maxRange, strongest, unsafe, 0));
        column.addColorStop(0.38, color);
        column.addColorStop(0.62, color);
        column.addColorStop(1, this.heatmapColor(nearest, maxRange, strongest, unsafe, 0));
        ctx.fillStyle = column;
        ctx.fillRect(centerX - columnWidth / 2, height * 0.12, columnWidth, height * 0.7);

        ctx.beginPath();
        ctx.ellipse(
          centerX,
          horizonY,
          Math.max(4, columnWidth * 0.3),
          2 + strongest * 5,
          0,
          0,
          Math.PI * 2,
        );
        ctx.fillStyle = this.heatmapColor(
          nearest,
          maxRange,
          strongest,
          unsafe,
          0.42 + strongest * 0.38,
        );
        ctx.fill();
      }
      ctx.restore();
    }

    drawDepthCameraEchoColumns(
      ctx,
      width,
      height,
      maxRange,
      unsafe,
      contour,
    ) {
      if (!Array.isArray(contour) || !contour.some(Boolean)) {
        return;
      }
      const horizonY = height * 0.48;
      const panels = contour.map((point, index) => {
        if (!point) {
          return null;
        }
        const angles = this.heatmapCellAngles(
          this.heatmapMeta,
          index,
          index + 1,
        );
        if (!angles) {
          return null;
        }
        const normalized = Math.max(0, Math.min(1, point.distance / maxRange));
        const closeness = 1 - normalized;
        const wallHeight = height * (0.16 + Math.pow(closeness, 1.25) * 0.61);
        return {
          point,
          x0: this.cameraXForAngle(angles[0], width),
          x1: this.cameraXForAngle(angles[1], width),
          top: horizonY - wallHeight * 0.56,
          bottom: horizonY + wallHeight * 0.44,
          closeness,
        };
      });

      ctx.save();
      this.depthCameraAperturePath(ctx, width, height);
      ctx.clip();
      panels.forEach((panel, index) => {
        if (!panel) {
          return;
        }
        const previous = index > 0 ? panels[index - 1] : null;
        const next = index + 1 < panels.length ? panels[index + 1] : null;
        const leftTop = previous
          ? (previous.top + panel.top) / 2
          : panel.top;
        const rightTop = next ? (next.top + panel.top) / 2 : panel.top;
        const leftBottom = previous
          ? (previous.bottom + panel.bottom) / 2
          : panel.bottom;
        const rightBottom = next
          ? (next.bottom + panel.bottom) / 2
          : panel.bottom;
        const grey = Math.round(62 + panel.closeness * 183);
        const alpha = unsafe
          ? 0.18
          : (0.26 + panel.point.strength * 0.34) * panel.point.freshness;
        const gradient = ctx.createLinearGradient(0, panel.top, 0, panel.bottom);
        gradient.addColorStop(0, `rgba(${grey},${grey},${grey},${alpha * 0.34})`);
        gradient.addColorStop(0.18, `rgba(${grey},${grey},${grey},${alpha * 0.84})`);
        gradient.addColorStop(0.52, `rgba(${grey},${grey},${grey},${alpha})`);
        gradient.addColorStop(0.86, `rgba(${grey},${grey},${grey},${alpha * 0.78})`);
        gradient.addColorStop(1, `rgba(${grey},${grey},${grey},${alpha * 0.22})`);
        ctx.beginPath();
        ctx.moveTo(panel.x0 - 1, leftTop);
        ctx.lineTo(panel.x1 + 1, rightTop);
        ctx.lineTo(panel.x1 + 1, rightBottom);
        ctx.lineTo(panel.x0 - 1, leftBottom);
        ctx.closePath();
        ctx.fillStyle = gradient;
        ctx.fill();

        if (this.outlineToggle.checked) {
          ctx.strokeStyle = `rgba(255,255,255,${0.18 + panel.point.strength * 0.5})`;
          ctx.lineWidth = 1.1;
          ctx.beginPath();
          ctx.moveTo(panel.x0, leftTop);
          ctx.lineTo(panel.x1, rightTop);
          ctx.moveTo(panel.x0, leftBottom);
          ctx.lineTo(panel.x1, rightBottom);
          ctx.stroke();
        }

        const stripeAlpha = 0.035 + panel.point.strength * 0.07;
        ctx.strokeStyle = `rgba(255,255,255,${stripeAlpha})`;
        ctx.lineWidth = 1;
        for (let y = panel.top + 7; y < panel.bottom; y += 11) {
          ctx.beginPath();
          ctx.moveTo(panel.x0 + 2, y);
          ctx.lineTo(panel.x1 - 2, y);
          ctx.stroke();
        }
      });
      ctx.restore();

      const labelCandidates = panels
        .filter(Boolean)
        .sort((left, right) => left.point.distance - right.point.distance);
      const labelled = [];
      ctx.save();
      labelCandidates.forEach((panel) => {
        const x = (panel.x0 + panel.x1) / 2;
        if (
          labelled.length >= 5 ||
          labelled.some((candidate) => Math.abs(candidate - x) < 92)
        ) {
          return;
        }
        labelled.push(x);
        const y = Math.max(38, panel.top - 8);
        const label = `${Math.round(panel.point.distance * 100)} cm`;
        ctx.font = "800 12px ui-monospace, SFMono-Regular, Consolas, monospace";
        const textWidth = ctx.measureText(label).width;
        ctx.fillStyle = "rgba(0,0,0,0.84)";
        ctx.fillRect(x - textWidth / 2 - 5, y - 13, textWidth + 10, 18);
        ctx.fillStyle = unsafe
          ? "rgba(205,205,205,0.72)"
          : "rgba(255,255,255,0.94)";
        ctx.textAlign = "center";
        ctx.fillText(label, x, y);
      });
      ctx.restore();
    }

    drawDepthCameraPointsLegacy(ctx, width, height, maxRange, unsafe) {
      const occupancy = this.state && this.state.occupancy;
      if (!occupancy || !Array.isArray(occupancy.points)) {
        return;
      }
      const persistenceMs = Number(this.persistenceSelect.value);
      const cells = new Map();
      occupancy.points.forEach((point) => {
        const forward = Number(point[0]);
        const lateral = Number(point[1]);
        const measuredHeight = Number(point[2]);
        const velocity = Number(point[3]);
        const snr = point[4] === null ? 14 : Number(point[4]);
        const ageMs = Number(point[5]);
        const projected = this.projectRadarCamera(
          forward,
          lateral,
          measuredHeight,
          width,
          height,
        );
        if (
          !projected ||
          projected.distance < VALID_MIN_RANGE_M ||
          projected.distance > maxRange ||
          projected.distance > VALID_MAX_RANGE_M ||
          !Number.isFinite(ageMs) ||
          ageMs > persistenceMs
        ) {
          return;
        }
        const fade = Math.max(0, 1 - ageMs / persistenceMs);
        const strength = Math.max(0.2, Math.min(1, snr / 28));
        const weight = Math.max(0.04, fade * strength);
        const key = `${Math.round(projected.x / 24)}:${Math.round(projected.y / 24)}:${Math.round(projected.distance / 0.35)}`;
        const cell = cells.get(key);
        if (!cell) {
          cells.set(key, {
            x: projected.x,
            y: projected.y,
            distance: projected.distance,
            height: measuredHeight,
            velocity,
            strength,
            confidence: weight,
            samples: 1,
            measuredZ: Math.abs(measuredHeight) > 0.001,
          });
          return;
        }
        const total = cell.confidence + weight;
        cell.x = (cell.x * cell.confidence + projected.x * weight) / total;
        cell.y = (cell.y * cell.confidence + projected.y * weight) / total;
        cell.distance = Math.min(cell.distance, projected.distance);
        cell.height = (cell.height * cell.confidence + measuredHeight * weight) / total;
        cell.velocity = (cell.velocity * cell.confidence + velocity * weight) / total;
        cell.strength = Math.max(cell.strength, strength);
        cell.confidence = total;
        cell.samples += 1;
        cell.measuredZ = cell.measuredZ || Math.abs(measuredHeight) > 0.001;
      });

      const clusters = Array.from(cells.values()).sort(
        (left, right) => right.distance - left.distance,
      );
      ctx.save();
      clusters.forEach((cluster) => {
        const closeness = 1 - Math.min(1, cluster.distance / maxRange);
        const confidence = Math.min(1, cluster.confidence / 2.5);
        const radius = Math.max(2.4, Math.min(7.5, 3 + closeness * 3 + confidence * 1.5));
        ctx.beginPath();
        ctx.arc(cluster.x, cluster.y, radius, 0, Math.PI * 2);
        ctx.fillStyle = this.depthCameraColor(
          cluster.distance,
          maxRange,
          cluster.strength,
          unsafe,
          0.68 + confidence * 0.28,
        );
        ctx.fill();
        ctx.strokeStyle = cluster.measuredZ
          ? "rgba(244, 255, 252, 0.96)"
          : "rgba(206, 231, 226, 0.62)";
        ctx.lineWidth = cluster.measuredZ ? 1.2 : 0.7;
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(cluster.x - radius - 3, cluster.y);
        ctx.lineTo(cluster.x + radius + 3, cluster.y);
        ctx.moveTo(cluster.x, cluster.y - radius - 3);
        ctx.lineTo(cluster.x, cluster.y + radius + 3);
        ctx.strokeStyle = this.depthCameraColor(
          cluster.distance,
          maxRange,
          cluster.strength,
          unsafe,
          0.35 + confidence * 0.3,
        );
        ctx.lineWidth = 0.8;
        ctx.stroke();
      });

      const labelled = [];
      clusters
        .filter((cluster) => cluster.confidence > 1.1)
        .sort((left, right) => left.distance - right.distance)
        .forEach((cluster) => {
          if (
            labelled.length >= 5 ||
            labelled.some(
              (existing) =>
                Math.hypot(existing.x - cluster.x, existing.y - cluster.y) < 58,
            )
          ) {
            return;
          }
          labelled.push(cluster);
        });
      labelled.forEach((cluster) => {
        ctx.font = "700 9px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "center";
        ctx.fillStyle = unsafe ? "rgba(220, 225, 224, 0.68)" : "rgba(239, 250, 247, 0.82)";
        ctx.fillText(`${cluster.distance.toFixed(1)}m`, cluster.x, cluster.y - 13);
      });
      ctx.restore();
    }

    drawDepthCameraPoints(ctx, width, height, maxRange, unsafe) {
      const occupancy = this.state && this.state.occupancy;
      if (!occupancy || !Array.isArray(occupancy.points)) {
        return;
      }
      const persistenceMs = Number(this.persistenceSelect.value);
      const points = occupancy.points
        .map((point) => {
          const forward = Number(point[0]);
          const lateral = Number(point[1]);
          const measuredHeight = Number(point[2]);
          const snr = point[4] === null ? 14 : Number(point[4]);
          const ageMs = Number(point[5]);
          const projected = this.projectRadarCamera(
            forward,
            lateral,
            measuredHeight,
            width,
            height,
          );
          return {
            projected,
            forward,
            lateral,
            height: measuredHeight,
            snr,
            ageMs,
            measuredHeight: Math.abs(measuredHeight) > 0.001,
          };
        })
        .filter((point) =>
          point.projected &&
          point.projected.distance >= VALID_MIN_RANGE_M &&
          point.projected.distance <= maxRange &&
          Number.isFinite(point.ageMs) &&
          point.ageMs <= persistenceMs
        )
        .sort((left, right) => right.projected.distance - left.projected.distance);

      const horizonY = this.cameraYForElevation(0, height);
      ctx.save();
      points.forEach((point) => {
        const projected = point.projected;
        const fade = Math.max(0.12, 1 - point.ageMs / persistenceMs);
        const closeness = 1 - Math.min(1, projected.distance / maxRange);
        const strength = Math.max(0.25, Math.min(1, point.snr / 28));
        const radius = 4 + closeness * 8;
        const halo = ctx.createRadialGradient(
          projected.x,
          projected.y,
          0,
          projected.x,
          projected.y,
          radius * 2.4,
        );
        halo.addColorStop(0, `rgba(255,255,255,${0.92 * fade})`);
        halo.addColorStop(0.3, `rgba(255,255,255,${0.42 * fade * strength})`);
        halo.addColorStop(1, "rgba(255,255,255,0)");
        ctx.fillStyle = halo;
        ctx.beginPath();
        ctx.arc(projected.x, projected.y, radius * 2.4, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = `rgba(255,255,255,${0.35 + 0.58 * fade})`;
        ctx.lineWidth = point.measuredHeight ? 1.4 : 0.85;
        if (point.measuredHeight) {
          ctx.beginPath();
          ctx.moveTo(projected.x, horizonY);
          ctx.lineTo(projected.x, projected.y);
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.arc(projected.x, projected.y, radius, 0, Math.PI * 2);
        ctx.moveTo(projected.x - radius - 5, projected.y);
        ctx.lineTo(projected.x + radius + 5, projected.y);
        ctx.moveTo(projected.x, projected.y - radius - 5);
        ctx.lineTo(projected.x, projected.y + radius + 5);
        ctx.stroke();
      });

      const labelled = [];
      points
        .slice()
        .sort((left, right) => left.projected.distance - right.projected.distance)
        .forEach((point) => {
          if (
            labelled.length >= 6 ||
            labelled.some((other) =>
              Math.hypot(
                other.projected.x - point.projected.x,
                other.projected.y - point.projected.y,
              ) < 76
            )
          ) {
            return;
          }
          labelled.push(point);
          const distanceCm = Math.round(point.projected.distance * 100);
          const heightText = point.measuredHeight
            ? `${point.height >= 0 ? "+" : ""}${Math.round(point.height * 100)}cm`
            : "--";
          const label = `D ${distanceCm}cm · H ${heightText}`;
          const x = Math.max(65, Math.min(width - 65, point.projected.x));
          const y = Math.max(28, point.projected.y - 19);
          ctx.font = "800 11px ui-monospace, SFMono-Regular, Consolas, monospace";
          const textWidth = ctx.measureText(label).width;
          ctx.fillStyle = "rgba(0,0,0,0.88)";
          ctx.fillRect(x - textWidth / 2 - 5, y - 13, textWidth + 10, 18);
          ctx.fillStyle = unsafe
            ? "rgba(210,210,210,0.76)"
            : "rgba(255,255,255,0.96)";
          ctx.textAlign = "center";
          ctx.fillText(label, x, y);
        });
      ctx.restore();
    }

    drawDepthCameraLatestPoints(ctx, width, height, maxRange, unsafe) {
      const frame = this.state && this.state.frame;
      if (!frame || !Array.isArray(frame.points)) {
        return;
      }
      ctx.save();
      frame.points.forEach((point) => {
        const projected = this.projectRadarCamera(
          Number(point[0]),
          Number(point[1]),
          Number(point[2]),
          width,
          height,
        );
        if (
          !projected ||
          projected.distance < VALID_MIN_RANGE_M ||
          projected.distance > maxRange
        ) {
          return;
        }
        ctx.beginPath();
        ctx.arc(projected.x, projected.y, 5, 0, Math.PI * 2);
        ctx.strokeStyle = unsafe
          ? "rgba(210, 216, 214, 0.7)"
          : "rgba(255, 255, 255, 0.94)";
        ctx.lineWidth = 1.3;
        ctx.stroke();
      });
      ctx.restore();
    }

    drawDepthCameraHudLegacy(ctx, width, height, maxRange, unsafe) {
      const centerX = width / 2;
      const centerY = this.cameraYForElevation(0, height);
      ctx.save();
      ctx.strokeStyle = unsafe
        ? "rgba(255, 81, 81, 0.8)"
        : "rgba(255, 193, 91, 0.78)";
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(centerX - 18, centerY);
      ctx.lineTo(centerX - 5, centerY);
      ctx.moveTo(centerX + 5, centerY);
      ctx.lineTo(centerX + 18, centerY);
      ctx.moveTo(centerX, centerY - 18);
      ctx.lineTo(centerX, centerY - 5);
      ctx.moveTo(centerX, centerY + 5);
      ctx.lineTo(centerX, centerY + 18);
      ctx.stroke();

      ctx.fillStyle = "rgba(215, 235, 231, 0.72)";
      ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "right";
      ctx.fillText(`DEPTH 0.25-${maxRange.toFixed(0)}m`, width - 14, 72);
      ctx.fillText("X/Y/Z POINTS · ELEV FFT 8", width - 14, 88);
      ctx.textAlign = "left";
      ctx.fillText("색/크기 = 거리·SNR", 14, height - 102);
      ctx.fillText("세로 = 실측 고도각", 14, height - 87);
      ctx.restore();
    }

    drawDepthCameraHud(ctx, width, height, maxRange, unsafe, contour) {
      const centerX = width / 2;
      const centerY = this.cameraYForElevation(0, height);
      const frameNearest = Number(
        this.state && this.state.frame
          ? this.state.frame.nearest_corridor_m
          : NaN,
      );
      const contourDistances = Array.isArray(contour)
        ? contour.filter(Boolean).map((point) => point.distance)
        : [];
      const nearest = Math.min(
        Number.isFinite(frameNearest) && frameNearest > 0 && frameNearest <= maxRange
          ? frameNearest
          : Infinity,
        contourDistances.length ? Math.min(...contourDistances) : Infinity,
      );
      const safetyColor = unsafe
        ? "rgba(210,210,210,0.72)"
        : nearest <= DANGER_RANGE_M
          ? "rgba(255,74,67,0.96)"
          : nearest <= CAUTION_RANGE_M
            ? "rgba(255,158,72,0.94)"
            : "rgba(235,245,243,0.84)";

      ctx.save();
      ctx.strokeStyle = safetyColor;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(centerX - 20, centerY);
      ctx.lineTo(centerX - 6, centerY);
      ctx.moveTo(centerX + 6, centerY);
      ctx.lineTo(centerX + 20, centerY);
      ctx.moveTo(centerX, centerY - 20);
      ctx.lineTo(centerX, centerY - 6);
      ctx.moveTo(centerX, centerY + 6);
      ctx.lineTo(centerX, centerY + 20);
      ctx.stroke();

      const distanceLabel = Number.isFinite(nearest)
        ? `FRONT ${Math.round(nearest * 100)} cm`
        : "FRONT -- cm";
      ctx.font = "900 18px ui-monospace, SFMono-Regular, Consolas, monospace";
      const textWidth = ctx.measureText(distanceLabel).width;
      ctx.fillStyle = "rgba(0,0,0,0.82)";
      ctx.fillRect(centerX - textWidth / 2 - 10, 58, textWidth + 20, 29);
      ctx.strokeStyle = safetyColor;
      ctx.strokeRect(centerX - textWidth / 2 - 10, 58, textWidth + 20, 29);
      ctx.fillStyle = safetyColor;
      ctx.textAlign = "center";
      ctx.fillText(distanceLabel, centerX, 79);

      if (Number.isFinite(nearest) && nearest < 0.07) {
        ctx.font = "800 11px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.fillStyle = "rgba(255,90,78,0.96)";
        ctx.fillText("< 7cm · TOO CLOSE / BLIND ZONE", centerX, 101);
      }

      const legendX = Math.max(16, width - 236);
      const legendY = height - 104;
      const legendWidth = 210;
      const gradient = ctx.createLinearGradient(legendX, 0, legendX + legendWidth, 0);
      gradient.addColorStop(0, "#3c3c3c");
      gradient.addColorStop(1, "#f7f7f7");
      ctx.fillStyle = gradient;
      ctx.fillRect(legendX, legendY, legendWidth, 7);
      ctx.strokeStyle = "rgba(255,255,255,0.34)";
      ctx.strokeRect(legendX, legendY, legendWidth, 7);
      ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.fillStyle = "rgba(240,240,240,0.78)";
      ctx.textAlign = "left";
      ctx.fillText("50cm · 멀고 어두움", legendX, legendY + 20);
      ctx.textAlign = "right";
      ctx.fillText("10cm · 가깝고 밝음", legendX + legendWidth, legendY + 20);

      ctx.textAlign = "left";
      ctx.fillStyle = "rgba(235,235,235,0.66)";
      ctx.fillText("D=거리 · H=레이더 기준 높이", 16, height - 102);
      ctx.fillText("벽면은 반사 윤곽 · 점은 실측 X/Y/Z", 16, height - 86);
      ctx.restore();
    }

    projectRadarCamera(forward, lateral, measuredHeight, width, height) {
      if (
        !Number.isFinite(forward) ||
        !Number.isFinite(lateral) ||
        !Number.isFinite(measuredHeight) ||
        forward <= 0
      ) {
        return null;
      }
      const horizontalRange = Math.hypot(forward, lateral);
      const distance = Math.hypot(horizontalRange, measuredHeight);
      const azimuth = (Math.atan2(lateral, forward) * 180) / Math.PI;
      const elevation = (Math.atan2(measuredHeight, horizontalRange) * 180) / Math.PI;
      if (
        azimuth < FOV_MIN_DEG ||
        azimuth > FOV_MAX_DEG ||
        elevation < -60 ||
        elevation > 60
      ) {
        return null;
      }
      return {
        x: this.cameraXForAngle(azimuth, width),
        y: this.cameraYForElevation(elevation, height),
        azimuth,
        elevation,
        distance,
      };
    }

    cameraXForAngle(angle, width) {
      return width / 2 + (angle / FOV_MAX_DEG) * width * 0.46;
    }

    cameraYForElevation(elevation, height) {
      return height * 0.48 - (elevation / 60) * height * 0.36;
    }

    depthCameraColor(distance, maxRange, strength, unsafe, alpha) {
      const safeAlpha = Math.max(0, Math.min(0.96, alpha));
      if (unsafe) {
        const grey = Math.round(120 + Math.max(0, Math.min(1, strength)) * 90);
        return `rgba(${grey}, ${grey}, ${grey}, ${safeAlpha * 0.62})`;
      }
      const normalized = Math.max(0, Math.min(1, distance / maxRange));
      if (distance <= DANGER_RANGE_M) {
        return `rgba(255, 78, 68, ${safeAlpha})`;
      }
      if (distance <= CAUTION_RANGE_M) {
        return `rgba(255, 145, 64, ${safeAlpha})`;
      }
      if (normalized < 0.34) {
        return `rgba(255, 181, 73, ${safeAlpha})`;
      }
      if (normalized < 0.7) {
        return `rgba(80, 226, 204, ${safeAlpha})`;
      }
      return `rgba(71, 166, 205, ${safeAlpha})`;
    }

    heatmapColor(distance, maxRange, strength, unsafe, alpha) {
      const safeAlpha = Math.max(0, Math.min(0.96, alpha));
      if (unsafe) {
        const grey = Math.round(120 + Math.max(0, Math.min(1, strength)) * 90);
        return `rgba(${grey}, ${grey}, ${grey}, ${safeAlpha * 0.62})`;
      }
      const normalized = Math.max(0, Math.min(1, distance / maxRange));
      if (distance <= CAUTION_RANGE_M) {
        return `rgba(255, 181, 73, ${safeAlpha})`;
      }
      if (normalized < 0.34) {
        return `rgba(121, 231, 207, ${safeAlpha})`;
      }
      if (normalized < 0.7) {
        return `rgba(80, 210, 204, ${safeAlpha})`;
      }
      return `rgba(71, 166, 205, ${safeAlpha})`;
    }

    drawPerspective(ctx, width, height, maxRange, unsafe) {
      const camera = this.makePerspectiveCamera(width, height, maxRange);
      this.drawPerspectiveBackground(ctx, width, height, maxRange, camera);
      this.drawPerspectiveSafetyZones(ctx, maxRange, camera, unsafe);
      if (this.isHeatmapFresh()) {
        this.drawPerspectiveHeatmap(ctx, maxRange, camera, unsafe);
        if (this.outlineToggle.checked) {
          this.drawPerspectiveOutline(ctx, maxRange, camera, unsafe);
        }
      }
      this.drawPerspectivePoints(ctx, maxRange, camera, unsafe);
      if (this.rawToggle.checked) {
        this.drawPerspectiveLatestPoints(ctx, maxRange, camera, unsafe);
      }
      this.drawPerspectiveReticle(ctx, width, height, maxRange, camera, unsafe);
    }

    makePerspectiveCamera(width, height, maxRange) {
      const sceneRange = Math.max(0.5, maxRange);
      const position = {
        x: 0,
        y: sceneRange * 0.72,
        z: -sceneRange * 0.88,
      };
      const target = { x: 0, y: 0, z: sceneRange * 0.46 };
      const worldUp = { x: 0, y: 1, z: 0 };
      const forward = this.normalize3(this.subtract3(target, position));
      const right = this.normalize3(this.cross3(worldUp, forward));
      const up = this.normalize3(this.cross3(forward, right));
      return {
        position,
        forward,
        right,
        up,
        focal: Math.min(width * 0.72, height * 0.82),
        centerX: width / 2,
        centerY: height * 0.5,
      };
    }

    drawPerspectiveBackground(ctx, width, height, maxRange, camera) {
      const sky = ctx.createLinearGradient(0, 0, 0, height);
      sky.addColorStop(0, "#02080d");
      sky.addColorStop(0.42, "#07151b");
      sky.addColorStop(1, "#020507");
      ctx.fillStyle = sky;
      ctx.fillRect(0, 0, width, height);

      const floor = [];
      for (let angle = FOV_MIN_DEG; angle <= FOV_MAX_DEG; angle += 4) {
        floor.push(this.project3d(this.worldPolar(maxRange, angle, 0), camera));
      }
      for (let angle = FOV_MAX_DEG; angle >= FOV_MIN_DEG; angle -= 4) {
        floor.push(this.project3d(this.worldPolar(VALID_MIN_RANGE_M, angle, 0), camera));
      }
      const visibleFloor = floor.filter(Boolean);
      if (visibleFloor.length >= 3) {
        const floorGradient = ctx.createLinearGradient(0, height * 0.28, 0, height);
        floorGradient.addColorStop(0, "rgba(19, 48, 52, 0.05)");
        floorGradient.addColorStop(1, "rgba(8, 27, 31, 0.5)");
        ctx.beginPath();
        ctx.moveTo(visibleFloor[0].x, visibleFloor[0].y);
        visibleFloor.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
        ctx.closePath();
        ctx.fillStyle = floorGradient;
        ctx.fill();
      }

      ctx.save();
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      const ringStep =
        maxRange <= 0.5 ? 0.1 : maxRange <= 1 ? 0.2 : maxRange <= 3 ? 0.5 : 1;

      // Nested range cross-sections make point depth readable inside the dome.
      ctx.lineWidth = 0.8;
      ctx.strokeStyle = "rgba(100, 196, 190, 0.16)";
      for (let distance = ringStep; distance <= maxRange + 0.001; distance += ringStep) {
        this.strokeWorldArc(ctx, distance, camera);
        this.strokeHemisphereMeridian(ctx, distance, 0, camera);
      }

      // The outer range shell is a true 3D spherical-sector wireframe.
      ctx.lineWidth = 1.15;
      ctx.strokeStyle = "rgba(112, 225, 211, 0.32)";
      [-60, -40, -20, 0, 20, 40, 60].forEach((elevation) => {
        this.strokeHemisphereLatitude(ctx, maxRange, elevation, camera);
      });
      [-70, -42, -14, 14, 42, 70].forEach((azimuth) => {
        this.strokeHemisphereMeridian(ctx, maxRange, azimuth, camera);
      });

      ctx.lineWidth = 0.85;
      ctx.strokeStyle = "rgba(112, 225, 211, 0.2)";
      [
        [FOV_MIN_DEG, ELEVATION_MIN_DEG],
        [FOV_MIN_DEG, 0],
        [FOV_MIN_DEG, ELEVATION_MAX_DEG],
        [0, ELEVATION_MIN_DEG],
        [0, ELEVATION_MAX_DEG],
        [FOV_MAX_DEG, ELEVATION_MIN_DEG],
        [FOV_MAX_DEG, 0],
        [FOV_MAX_DEG, ELEVATION_MAX_DEG],
      ].forEach(([azimuth, elevation]) => {
        this.strokeHemisphereRadius(ctx, maxRange, azimuth, elevation, camera);
      });

      ctx.fillStyle = "rgba(193, 226, 220, 0.72)";
      ctx.font = "10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      for (let distance = ringStep; distance <= maxRange + 0.001; distance += ringStep) {
        const label = this.project3d({ x: 0, y: 0.015, z: distance }, camera);
        if (label) {
          ctx.fillText(`${distance.toFixed(distance < 1 ? 1 : 0)} m`, label.x, label.y - 4);
        }
      }

      ctx.textAlign = "left";
      ctx.fillStyle = "rgba(112, 225, 211, 0.72)";
      [ELEVATION_MAX_DEG, 0, ELEVATION_MIN_DEG].forEach((elevation) => {
        const label = this.project3d(
          this.worldSpherical(maxRange, FOV_MAX_DEG, elevation),
          camera,
        );
        if (label) {
          ctx.fillText(`${elevation > 0 ? "+" : ""}${elevation}°`, label.x + 5, label.y);
        }
      });
      ctx.restore();
    }

    drawPerspectiveSafetyZones(ctx, maxRange, camera, unsafe) {
      const zones = [
        {
          near: VALID_MIN_RANGE_M,
          far: Math.min(DANGER_RANGE_M, maxRange),
          color: unsafe ? "rgba(160, 165, 164, 0.06)" : "rgba(255, 70, 65, 0.09)",
        },
        {
          near: Math.min(DANGER_RANGE_M, maxRange),
          far: Math.min(CAUTION_RANGE_M, maxRange),
          color: unsafe ? "rgba(145, 150, 149, 0.045)" : "rgba(255, 145, 64, 0.07)",
        },
        {
          near: Math.min(CAUTION_RANGE_M, maxRange),
          far: maxRange,
          color: unsafe ? "rgba(130, 135, 134, 0.03)" : "rgba(80, 210, 204, 0.035)",
        },
      ];
      ctx.save();
      zones.forEach((zone) => {
        if (zone.far <= zone.near) {
          return;
        }
        const polygon = [];
        for (let angle = FOV_MIN_DEG; angle <= FOV_MAX_DEG; angle += 3) {
          polygon.push(this.project3d(this.worldPolar(zone.far, angle, 0.012), camera));
        }
        for (let angle = FOV_MAX_DEG; angle >= FOV_MIN_DEG; angle -= 3) {
          polygon.push(this.project3d(this.worldPolar(zone.near, angle, 0.012), camera));
        }
        const visible = polygon.filter(Boolean);
        if (visible.length < 3) {
          return;
        }
        ctx.beginPath();
        ctx.moveTo(visible[0].x, visible[0].y);
        visible.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
        ctx.closePath();
        ctx.fillStyle = zone.color;
        ctx.fill();
      });
      ctx.restore();
    }

    drawPerspectiveHeatmap(ctx, maxRange, camera, unsafe) {
      const meta = this.heatmapMeta;
      const rangeBins = Number(meta.range_bins);
      const azimuthBins = Number(meta.azimuth_bins);
      const rangeStep = Number(meta.range_step_m);
      const validMin = Math.max(
        VALID_MIN_RANGE_M,
        Number.isFinite(meta.valid_min_range_m)
          ? Number(meta.valid_min_range_m)
          : VALID_MIN_RANGE_M,
      );
      const validMax = Math.min(
        maxRange,
        VALID_MAX_RANGE_M,
        Number.isFinite(meta.valid_max_range_m)
          ? Number(meta.valid_max_range_m)
          : VALID_MAX_RANGE_M,
      );
      const levels = this.heatmapRenderLevels(validMin, validMax);
      const now = Date.now();
      const lastRangeIndex = Math.min(
        rangeBins - 1,
        Math.floor(validMax / rangeStep),
      );

      ctx.save();
      for (let rangeIndex = lastRangeIndex; rangeIndex >= 0; rangeIndex -= 1) {
        const nearRange = Math.max(validMin, rangeIndex * rangeStep);
        const farRange = Math.min(validMax, (rangeIndex + 1.18) * rangeStep);
        if (farRange <= nearRange) {
          continue;
        }
        const offset = rangeIndex * azimuthBins;
        for (let azimuthIndex = 0; azimuthIndex < azimuthBins; azimuthIndex += 1) {
          const angles = this.heatmapCellAngles(
            meta,
            azimuthIndex,
            azimuthIndex + 1,
          );
          if (!angles) {
            continue;
          }
          const db = this.decayedHeatmapDb(offset + azimuthIndex, now);
          const intensity = Math.max(
            0,
            Math.min(1, (db - levels.low) / levels.span),
          );
          if (intensity < 0.045) {
            continue;
          }
          const vertices = [
            this.worldPolar(nearRange, angles[0], 0.006),
            this.worldPolar(farRange, angles[0], 0.006),
            this.worldPolar(farRange, angles[1], 0.006),
            this.worldPolar(nearRange, angles[1], 0.006),
          ].map((point) => this.project3d(point, camera));
          if (vertices.some((point) => point === null)) {
            continue;
          }
          ctx.beginPath();
          ctx.moveTo(vertices[0].x, vertices[0].y);
          vertices.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
          ctx.closePath();
          ctx.fillStyle = this.heatmapColor(
            (nearRange + farRange) / 2,
            maxRange,
            Math.pow(intensity, 0.78),
            unsafe,
            0.42,
          );
          ctx.fill();
        }
      }
      ctx.restore();
    }

    heatmapRenderLevels(validMin, validMax) {
      const meta = this.heatmapMeta;
      const rangeBins = Number(meta.range_bins);
      const azimuthBins = Number(meta.azimuth_bins);
      const rangeStep = Number(meta.range_step_m);
      const values = [];
      const now = Date.now();
      for (let rangeIndex = 0; rangeIndex < rangeBins; rangeIndex += 1) {
        const distance = rangeIndex * rangeStep;
        if (distance < validMin || distance > validMax) {
          continue;
        }
        const offset = rangeIndex * azimuthBins;
        for (let azimuthIndex = 0; azimuthIndex < azimuthBins; azimuthIndex += 1) {
          values.push(this.decayedHeatmapDb(offset + azimuthIndex, now));
        }
      }
      values.sort((left, right) => left - right);
      if (!values.length) {
        const low = Number(meta.floor_db);
        return { low, high: Number(meta.ceiling_db), span: Math.max(1, Number(meta.ceiling_db) - low) };
      }
      const percentile = (fraction) =>
        values[Math.min(values.length - 1, Math.floor((values.length - 1) * fraction))];
      const low = Math.max(Number(meta.floor_db), percentile(0.7));
      const high = Math.max(low + 8, percentile(0.992));
      return { low, high, span: high - low };
    }

    depthContourCandidates(maxRange) {
      if (!this.smoothedHeatmap || !this.heatmapMeta) {
        return [];
      }
      const meta = this.heatmapMeta;
      const rangeBins = Number(meta.range_bins);
      const azimuthBins = Number(meta.azimuth_bins);
      const rangeStep = Number(meta.range_step_m);
      const validMin = Math.max(
        WALL_CONTOUR_MIN_M,
        Number.isFinite(Number(meta.valid_min_range_m))
          ? Number(meta.valid_min_range_m)
          : WALL_CONTOUR_MIN_M,
      );
      const validMax = Math.min(
        maxRange,
        VALID_MAX_RANGE_M,
        Number.isFinite(Number(meta.valid_max_range_m))
          ? Number(meta.valid_max_range_m)
          : VALID_MAX_RANGE_M,
      );
      const candidates = Array.from({ length: azimuthBins }, () => null);
      if (!(validMax > validMin) || !(rangeStep > 0)) {
        return candidates;
      }
      const levels = this.heatmapRenderLevels(validMin, validMax);
      const supportThreshold = levels.low + levels.span * 0.3;
      const now = Date.now();
      const firstRange = Math.max(0, Math.ceil(validMin / rangeStep));
      const lastRange = Math.min(
        rangeBins - 1,
        Math.floor(validMax / rangeStep),
      );
      for (let azimuthIndex = 0; azimuthIndex < azimuthBins; azimuthIndex += 1) {
        const angle = this.heatmapBinCenterAngle(meta, azimuthIndex);
        if (angle === null) {
          continue;
        }
        let best = null;
        for (let rangeIndex = firstRange; rangeIndex <= lastRange; rangeIndex += 1) {
          const db = this.decayedHeatmapDb(
            rangeIndex * azimuthBins + azimuthIndex,
            now,
          );
          const strength = Math.max(
            0,
            Math.min(1, (db - levels.low) / levels.span),
          );
          if (
            strength < 0.28 ||
            !this.hasSpatialHeatmapSupport(
              rangeIndex,
              azimuthIndex,
              supportThreshold,
              now,
            )
          ) {
            continue;
          }
          const score = strength - (rangeIndex - firstRange) * 0.015;
          if (!best || score > best.score) {
            best = {
              angle,
              distance: rangeIndex * rangeStep,
              strength,
              score,
              source: "heatmap",
            };
          }
        }
        candidates[azimuthIndex] = best;
      }

      const frame = this.state && this.state.frame;
      if (frame && Array.isArray(frame.points)) {
        const binAngles = Array.from({ length: azimuthBins }, (_, index) =>
          this.heatmapBinCenterAngle(meta, index),
        );
        frame.points.forEach((point) => {
          const forward = Number(point[0]);
          const lateral = Number(point[1]);
          const height = Number(point[2]);
          const snr = point[4] === null ? 14 : Number(point[4]);
          const distance = Math.hypot(forward, lateral, height);
          if (
            !Number.isFinite(distance) ||
            forward <= 0 ||
            distance < VALID_MIN_RANGE_M ||
            distance > maxRange
          ) {
            return;
          }
          const angle = (Math.atan2(lateral, forward) * 180) / Math.PI;
          let bestIndex = -1;
          let bestDelta = Infinity;
          binAngles.forEach((candidateAngle, index) => {
            if (candidateAngle === null) {
              return;
            }
            const delta = Math.abs(candidateAngle - angle);
            if (delta < bestDelta) {
              bestDelta = delta;
              bestIndex = index;
            }
          });
          if (bestIndex < 0 || bestDelta > 18) {
            return;
          }
          const existing = candidates[bestIndex];
          const strength = Math.max(0.42, Math.min(1, snr / 30));
          if (!existing || distance <= existing.distance + rangeStep * 0.7) {
            candidates[bestIndex] = {
              angle,
              distance,
              strength,
              score: 1 + strength,
              source: "point",
            };
          }
        });
      }
      return candidates;
    }

    updateDepthContourTrack() {
      const selectedRange = Number(this.rangeSelect && this.rangeSelect.value);
      const maxRange = Number.isFinite(selectedRange) ? selectedRange : 0.5;
      const candidates = this.depthContourCandidates(maxRange);
      const now = Date.now();
      if (this.depthContourTrack.length !== candidates.length) {
        this.depthContourTrack = Array.from(
          { length: candidates.length },
          () => null,
        );
      }
      candidates.forEach((candidate, index) => {
        if (!candidate) {
          return;
        }
        const previous = this.depthContourTrack[index];
        const compatible =
          previous &&
          Math.abs(previous.distance - candidate.distance) <= 0.24;
        this.depthContourTrack[index] = {
          angle: candidate.angle,
          distance: compatible
            ? previous.distance * 0.28 + candidate.distance * 0.72
            : candidate.distance,
          strength: compatible
            ? previous.strength * 0.25 + candidate.strength * 0.75
            : candidate.strength,
          source: candidate.source,
          seenAt: now,
        };
      });
    }

    stableDepthContour(maxRange) {
      const now = Date.now();
      return this.depthContourTrack.map((point) => {
        if (!point || point.distance > maxRange) {
          return null;
        }
        const age = Math.max(0, now - point.seenAt);
        if (age > WALL_TRACK_HOLD_MS) {
          return null;
        }
        return Object.assign({}, point, {
          freshness: Math.max(0.16, 1 - age / WALL_TRACK_HOLD_MS),
        });
      });
    }

    perspectiveContour(maxRange) {
      if (!this.isHeatmapFresh()) {
        return [];
      }
      const meta = this.heatmapMeta;
      const rangeBins = Number(meta.range_bins);
      const azimuthBins = Number(meta.azimuth_bins);
      const rangeStep = Number(meta.range_step_m);
      const validMin = Math.max(
        VALID_MIN_RANGE_M,
        Number.isFinite(meta.valid_min_range_m)
          ? Number(meta.valid_min_range_m)
          : VALID_MIN_RANGE_M,
      );
      const validMax = Math.min(
        maxRange,
        VALID_MAX_RANGE_M,
        Number.isFinite(meta.valid_max_range_m)
          ? Number(meta.valid_max_range_m)
          : VALID_MAX_RANGE_M,
      );
      const levels = this.heatmapRenderLevels(validMin, validMax);
      const thresholdDb = levels.low + levels.span * 0.56;
      const now = Date.now();
      const firstRange = Math.max(0, Math.ceil(validMin / rangeStep));
      const lastRange = Math.min(
        rangeBins - 1,
        Math.floor(validMax / rangeStep),
      );
      const contour = [];
      for (let azimuthIndex = 0; azimuthIndex < azimuthBins; azimuthIndex += 1) {
        const angle = this.heatmapBinCenterAngle(meta, azimuthIndex);
        if (angle === null) {
          contour.push(null);
          continue;
        }
        let hit = null;
        for (let rangeIndex = firstRange; rangeIndex <= lastRange; rangeIndex += 1) {
          const index = rangeIndex * azimuthBins + azimuthIndex;
          const db = this.decayedHeatmapDb(index, now);
          if (
            db < thresholdDb ||
            !this.hasSpatialHeatmapSupport(
              rangeIndex,
              azimuthIndex,
              thresholdDb,
              now,
            )
          ) {
            continue;
          }
          hit = {
            angle,
            distance: Math.max(validMin, rangeIndex * rangeStep),
            strength: Math.max(0, Math.min(1, (db - levels.low) / levels.span)),
          };
          break;
        }
        contour.push(hit);
      }
      return contour.map((point, index) => {
        if (!point) {
          return null;
        }
        const neighbors = contour
          .slice(Math.max(0, index - 1), Math.min(contour.length, index + 2))
          .filter(Boolean)
          .map((candidate) => candidate.distance)
          .sort((left, right) => left - right);
        const median = neighbors[Math.floor(neighbors.length / 2)];
        return Object.assign({}, point, {
          distance:
            Math.abs(point.distance - median) <= 0.5
              ? point.distance * 0.62 + median * 0.38
              : point.distance,
        });
      });
    }

    drawPerspectiveOutline(ctx, maxRange, camera, unsafe) {
      const contour = this.perspectiveContour(maxRange);
      if (!contour.some(Boolean)) {
        return;
      }
      const projected = contour.map((point) =>
        point
          ? Object.assign({}, point, {
              screen: this.project3d(
                this.worldPolar(point.distance, point.angle, 0.045),
                camera,
              ),
            })
          : null,
      );
      ctx.save();
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      for (let index = 1; index < projected.length; index += 1) {
        const left = projected[index - 1];
        const right = projected[index];
        if (!left || !right || !left.screen || !right.screen) {
          continue;
        }
        const continuityLimit = Math.max(
          0.42,
          Math.min(left.distance, right.distance) * 0.28,
        );
        if (Math.abs(left.distance - right.distance) > continuityLimit) {
          continue;
        }
        ctx.beginPath();
        ctx.moveTo(left.screen.x, left.screen.y);
        ctx.lineTo(right.screen.x, right.screen.y);
        ctx.strokeStyle = "rgba(1, 5, 7, 0.9)";
        ctx.lineWidth = 5;
        ctx.stroke();
        ctx.strokeStyle = this.heatmapColor(
          (left.distance + right.distance) / 2,
          maxRange,
          Math.max(left.strength, right.strength),
          unsafe,
          0.96,
        );
        ctx.lineWidth = 2.2;
        ctx.stroke();
      }
      projected.filter((point) => point && point.screen).forEach((point) => {
        ctx.save();
        ctx.translate(point.screen.x, point.screen.y);
        ctx.rotate(Math.PI / 4);
        ctx.fillStyle = unsafe
          ? "rgba(190, 198, 196, 0.78)"
          : "rgba(255, 224, 132, 0.96)";
        ctx.fillRect(-2.2, -2.2, 4.4, 4.4);
        ctx.restore();
      });
      ctx.restore();
    }

    drawPerspectivePoints(ctx, maxRange, camera, unsafe) {
      const frame = this.state && this.state.frame;
      if (!frame || !Array.isArray(frame.points)) {
        return;
      }
      const points = frame.points
        .map((point) => ({
          forward: Number(point[0]),
          lateral: Number(point[1]),
          height: Number(point[2]),
          snr: point[4] === null ? 14 : Number(point[4]),
        }))
        .filter((point) => {
          const distance = Math.hypot(
            point.forward,
            point.lateral,
            point.height,
          );
          const angle = (Math.atan2(point.lateral, point.forward) * 180) / Math.PI;
          return (
            Number.isFinite(point.forward) &&
            Number.isFinite(point.lateral) &&
            Number.isFinite(point.height) &&
            point.forward > 0 &&
            distance >= VALID_MIN_RANGE_M &&
            distance <= maxRange &&
            distance <= VALID_MAX_RANGE_M &&
            Math.abs(point.height) <= maxRange &&
            angle >= FOV_MIN_DEG &&
            angle <= FOV_MAX_DEG
          );
        })
        .sort((left, right) => right.forward - left.forward);

      const projectedPoints = points
        .map((point) => {
          const distance = Math.hypot(
            point.forward,
            point.lateral,
            point.height,
          );
          return Object.assign({}, point, {
            distance,
            projected: this.project3d(
              { x: point.lateral, y: point.height, z: point.forward },
              camera,
            ),
            anchor: this.project3d(
              { x: point.lateral, y: 0, z: point.forward },
              camera,
            ),
          });
        })
        .filter((point) => point.projected && point.anchor);

      this.drawPerspectiveSurfaceMesh(ctx, projectedPoints, maxRange, unsafe);

      ctx.save();
      const groundOrdered = projectedPoints
        .slice()
        .sort(
          (left, right) =>
            Math.atan2(left.lateral, left.forward) -
            Math.atan2(right.lateral, right.forward),
        );
      for (let index = 1; index < groundOrdered.length; index += 1) {
        const left = groundOrdered[index - 1];
        const right = groundOrdered[index];
        const leftAngle = Math.atan2(left.lateral, left.forward);
        const rightAngle = Math.atan2(right.lateral, right.forward);
        if (
          Math.abs(left.distance - right.distance) > 0.65 ||
          Math.abs(leftAngle - rightAngle) > (22 * Math.PI) / 180
        ) {
          continue;
        }
        ctx.beginPath();
        ctx.moveTo(left.anchor.x, left.anchor.y);
        ctx.lineTo(right.anchor.x, right.anchor.y);
        ctx.strokeStyle = "rgba(1, 5, 7, 0.94)";
        ctx.lineWidth = 4.2;
        ctx.stroke();
        ctx.strokeStyle = unsafe
          ? "rgba(190, 198, 196, 0.62)"
          : "rgba(116, 244, 220, 0.88)";
        ctx.lineWidth = 1.65;
        ctx.stroke();
      }
      ctx.setLineDash([4, 4]);
      const linkedPairs = new Set();
      projectedPoints.forEach((point, pointIndex) => {
        let nearestIndex = -1;
        let nearestDistance = Infinity;
        projectedPoints.forEach((candidate, candidateIndex) => {
          if (candidateIndex === pointIndex) {
            return;
          }
          const separation = Math.hypot(
            point.forward - candidate.forward,
            point.lateral - candidate.lateral,
            point.height - candidate.height,
          );
          if (
            separation < nearestDistance &&
            separation <= 0.9 &&
            Math.abs(point.forward - candidate.forward) <= 0.55 &&
            Math.abs(point.height - candidate.height) <= 0.9
          ) {
            nearestDistance = separation;
            nearestIndex = candidateIndex;
          }
        });
        if (nearestIndex < 0) {
          return;
        }
        const pairKey = [pointIndex, nearestIndex].sort((a, b) => a - b).join(":");
        if (linkedPairs.has(pairKey)) {
          return;
        }
        linkedPairs.add(pairKey);
        const candidate = projectedPoints[nearestIndex];
        ctx.beginPath();
        ctx.moveTo(point.projected.x, point.projected.y);
        ctx.lineTo(candidate.projected.x, candidate.projected.y);
        ctx.strokeStyle = unsafe
          ? "rgba(180, 188, 186, 0.36)"
          : "rgba(255, 221, 144, 0.52)";
        ctx.lineWidth = 1.1;
        ctx.stroke();
      });
      ctx.setLineDash([]);

      const rendered = [];
      projectedPoints.forEach((point) => {
        const projected = point.projected;
        const anchor = point.anchor;
        const distance = point.distance;
        const strength = Math.max(
          0.2,
          Math.min(1, (Number.isFinite(point.snr) ? point.snr : 14) / 28),
        );
        const markerSize = Math.max(
          9,
          Math.min(16, (14 + strength * 5) / (0.7 + projected.depth * 0.05)),
        );
        if (Math.abs(projected.y - anchor.y) > 2) {
          ctx.beginPath();
          ctx.moveTo(anchor.x, anchor.y);
          ctx.lineTo(projected.x, projected.y);
          ctx.strokeStyle = "rgba(1, 5, 7, 0.92)";
          ctx.lineWidth = 3.2;
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(anchor.x, anchor.y);
          ctx.lineTo(projected.x, projected.y);
          ctx.strokeStyle = unsafe
            ? "rgba(180, 188, 186, 0.65)"
            : "rgba(185, 245, 233, 0.94)";
          ctx.lineWidth = 1.35;
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.arc(anchor.x, anchor.y, 3, 0, Math.PI * 2);
        ctx.fillStyle = "rgba(2, 8, 11, 0.88)";
        ctx.fill();
        ctx.strokeStyle = unsafe
          ? "rgba(205, 211, 210, 0.72)"
          : "rgba(157, 240, 226, 0.9)";
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.save();
        ctx.translate(projected.x, projected.y);
        ctx.rotate(Math.PI / 4);
        ctx.fillStyle = this.depthCameraColor(
          distance,
          maxRange,
          strength,
          unsafe,
          1,
        );
        ctx.fillRect(
          -markerSize / 2,
          -markerSize / 2,
          markerSize,
          markerSize,
        );
        ctx.strokeStyle = "rgba(1, 5, 7, 0.96)";
        ctx.lineWidth = 3.2;
        ctx.strokeRect(
          -markerSize / 2,
          -markerSize / 2,
          markerSize,
          markerSize,
        );
        ctx.strokeStyle = unsafe
          ? "rgba(225, 230, 229, 0.72)"
          : "rgba(250, 255, 254, 0.98)";
        ctx.lineWidth = 1.25;
        ctx.strokeRect(
          -markerSize / 2,
          -markerSize / 2,
          markerSize,
          markerSize,
        );
        ctx.fillStyle = "rgba(255, 255, 255, 0.96)";
        ctx.fillRect(-1, -1, 2, 2);
        ctx.restore();
        rendered.push({
          x: projected.x,
          y: projected.y,
          distance,
          height: point.height,
          strength,
        });
      });

      const labelled = [];
      rendered
        .slice()
        .sort((left, right) => left.distance - right.distance)
        .forEach((point) => {
          if (
            labelled.length >= 5 ||
            labelled.some(
              (existing) => Math.hypot(existing.x - point.x, existing.y - point.y) < 78,
            )
          ) {
            return;
          }
          labelled.push(point);
        });
      ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      labelled.forEach((point) => {
        const label = `${point.distance.toFixed(1)}m · z${point.height >= 0 ? "+" : ""}${point.height.toFixed(1)}`;
        const width = ctx.measureText(label).width + 10;
        const labelX = Math.max(width / 2 + 5, Math.min(camera.centerX * 2 - width / 2 - 5, point.x));
        const labelY = Math.max(18, point.y - 17);
        ctx.fillStyle = "rgba(2, 8, 11, 0.84)";
        ctx.fillRect(labelX - width / 2, labelY - 11, width, 15);
        ctx.strokeStyle = unsafe
          ? "rgba(190, 198, 196, 0.55)"
          : "rgba(255, 224, 132, 0.72)";
        ctx.lineWidth = 1;
        ctx.strokeRect(labelX - width / 2, labelY - 11, width, 15);
        ctx.fillStyle = unsafe
          ? "rgba(220, 225, 224, 0.82)"
          : "rgba(249, 255, 253, 0.96)";
        ctx.fillText(label, labelX, labelY);
      });
      ctx.restore();
    }

    drawPerspectiveSurfaceMesh(ctx, points, maxRange, unsafe) {
      if (points.length < 3) {
        return;
      }
      const connectionLimit = Math.min(0.35, Math.max(0.06, maxRange * 0.28));
      const triangles = [];
      const used = new Set();
      points.forEach((point, pointIndex) => {
        const nearest = points
          .map((candidate, candidateIndex) => ({
            candidate,
            candidateIndex,
            separation: Math.hypot(
              point.forward - candidate.forward,
              point.lateral - candidate.lateral,
              point.height - candidate.height,
            ),
          }))
          .filter(
            (item) =>
              item.candidateIndex !== pointIndex &&
              item.separation <= connectionLimit,
          )
          .sort((left, right) => left.separation - right.separation)
          .slice(0, 2);
        if (nearest.length < 2) {
          return;
        }
        const pairSeparation = Math.hypot(
          nearest[0].candidate.forward - nearest[1].candidate.forward,
          nearest[0].candidate.lateral - nearest[1].candidate.lateral,
          nearest[0].candidate.height - nearest[1].candidate.height,
        );
        if (pairSeparation > connectionLimit * 1.25) {
          return;
        }
        const key = [pointIndex, nearest[0].candidateIndex, nearest[1].candidateIndex]
          .sort((left, right) => left - right)
          .join(":");
        if (used.has(key)) {
          return;
        }
        used.add(key);
        const vertices = [
          point.projected,
          nearest[0].candidate.projected,
          nearest[1].candidate.projected,
        ];
        const area = Math.abs(
          (vertices[0].x * (vertices[1].y - vertices[2].y) +
            vertices[1].x * (vertices[2].y - vertices[0].y) +
            vertices[2].x * (vertices[0].y - vertices[1].y)) /
            2,
        );
        if (area < 8 || area > 9000) {
          return;
        }
        triangles.push({
          vertices,
          distance:
            (point.distance +
              nearest[0].candidate.distance +
              nearest[1].candidate.distance) /
            3,
          depth:
            (vertices[0].depth + vertices[1].depth + vertices[2].depth) / 3,
        });
      });
      triangles.sort((left, right) => right.depth - left.depth);
      ctx.save();
      triangles.slice(0, 18).forEach((triangle) => {
        ctx.beginPath();
        ctx.moveTo(triangle.vertices[0].x, triangle.vertices[0].y);
        ctx.lineTo(triangle.vertices[1].x, triangle.vertices[1].y);
        ctx.lineTo(triangle.vertices[2].x, triangle.vertices[2].y);
        ctx.closePath();
        ctx.fillStyle = this.depthCameraColor(
          triangle.distance,
          maxRange,
          0.8,
          unsafe,
          0.16,
        );
        ctx.fill();
        ctx.strokeStyle = unsafe
          ? "rgba(205, 211, 210, 0.34)"
          : "rgba(202, 255, 246, 0.52)";
        ctx.lineWidth = 0.9;
        ctx.stroke();
      });
      ctx.restore();
    }

    drawPerspectiveLatestPoints(ctx, maxRange, camera, unsafe) {
      const frame = this.state && this.state.frame;
      if (!frame || !Array.isArray(frame.points)) {
        return;
      }
      ctx.save();
      frame.points.forEach((point) => {
        const forward = Number(point[0]);
        const lateral = Number(point[1]);
        const height = Number(point[2]);
        const distance = Math.hypot(forward, lateral);
        if (
          !Number.isFinite(forward) ||
          !Number.isFinite(lateral) ||
          !Number.isFinite(height) ||
          forward <= 0 ||
          distance < VALID_MIN_RANGE_M ||
          distance > maxRange ||
          distance > VALID_MAX_RANGE_M
        ) {
          return;
        }
        const projected = this.project3d({ x: lateral, y: height, z: forward }, camera);
        if (!projected) {
          return;
        }
        ctx.beginPath();
        ctx.moveTo(projected.x - 4, projected.y);
        ctx.lineTo(projected.x + 4, projected.y);
        ctx.moveTo(projected.x, projected.y - 4);
        ctx.lineTo(projected.x, projected.y + 4);
        ctx.strokeStyle = unsafe ? "rgba(210, 216, 214, 0.7)" : "rgba(255, 255, 255, 0.94)";
        ctx.lineWidth = 1.2;
        ctx.stroke();
      });
      ctx.restore();
    }

    drawPerspectiveReticle(ctx, width, height, maxRange, camera, unsafe) {
      const origin = this.project3d({ x: 0, y: 0.02, z: VALID_MIN_RANGE_M }, camera);
      const forward = this.project3d({ x: 0, y: 0.02, z: Math.min(1, maxRange) }, camera);
      if (origin && forward) {
        ctx.beginPath();
        ctx.moveTo(origin.x, origin.y);
        ctx.lineTo(forward.x, forward.y);
        ctx.strokeStyle = unsafe ? "rgba(255, 81, 81, 0.9)" : "rgba(255, 193, 91, 0.9)";
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      ctx.fillStyle = "rgba(216, 235, 231, 0.72)";
      ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "right";
      ctx.fillText("반구 격자: 실측 X/Y/Z 공간 · 바닥 띠: 거리-방위 강도", width - 14, 72);
      ctx.fillText("마름모/높이선/점선: 반구 내부 최신 실측 포인트", width - 14, 88);
      ctx.textAlign = "left";
      ctx.fillStyle = "rgba(92, 228, 208, 0.78)";
      ctx.fillText("+Z 높이", 14, height - 87);
      ctx.fillText("+Y 전방 · +X 우측", 14, height - 71);
    }

    strokeWorldArc(ctx, distance, camera) {
      let drawing = false;
      ctx.beginPath();
      for (let angle = FOV_MIN_DEG; angle <= FOV_MAX_DEG; angle += 3) {
        const projected = this.project3d(this.worldPolar(distance, angle, 0.002), camera);
        if (!projected) {
          drawing = false;
          continue;
        }
        if (!drawing) {
          ctx.moveTo(projected.x, projected.y);
          drawing = true;
        } else {
          ctx.lineTo(projected.x, projected.y);
        }
      }
      ctx.stroke();
    }

    strokeHemisphereLatitude(ctx, distance, elevation, camera) {
      let drawing = false;
      ctx.beginPath();
      for (let azimuth = FOV_MIN_DEG; azimuth <= FOV_MAX_DEG; azimuth += 3) {
        const projected = this.project3d(
          this.worldSpherical(distance, azimuth, elevation),
          camera,
        );
        if (!projected) {
          drawing = false;
          continue;
        }
        if (!drawing) {
          ctx.moveTo(projected.x, projected.y);
          drawing = true;
        } else {
          ctx.lineTo(projected.x, projected.y);
        }
      }
      ctx.stroke();
    }

    strokeHemisphereMeridian(ctx, distance, azimuth, camera) {
      let drawing = false;
      ctx.beginPath();
      for (
        let elevation = ELEVATION_MIN_DEG;
        elevation <= ELEVATION_MAX_DEG;
        elevation += 3
      ) {
        const projected = this.project3d(
          this.worldSpherical(distance, azimuth, elevation),
          camera,
        );
        if (!projected) {
          drawing = false;
          continue;
        }
        if (!drawing) {
          ctx.moveTo(projected.x, projected.y);
          drawing = true;
        } else {
          ctx.lineTo(projected.x, projected.y);
        }
      }
      ctx.stroke();
    }

    strokeHemisphereRadius(ctx, distance, azimuth, elevation, camera) {
      const near = this.project3d(
        this.worldSpherical(VALID_MIN_RANGE_M, azimuth, elevation),
        camera,
      );
      const far = this.project3d(
        this.worldSpherical(distance, azimuth, elevation),
        camera,
      );
      if (!near || !far) {
        return;
      }
      ctx.beginPath();
      ctx.moveTo(near.x, near.y);
      ctx.lineTo(far.x, far.y);
      ctx.stroke();
    }

    worldPolar(distance, angleDegrees, height) {
      const angle = (angleDegrees * Math.PI) / 180;
      return {
        x: Math.sin(angle) * distance,
        y: height,
        z: Math.cos(angle) * distance,
      };
    }

    worldSpherical(distance, azimuthDegrees, elevationDegrees) {
      const azimuth = (azimuthDegrees * Math.PI) / 180;
      const elevation = (elevationDegrees * Math.PI) / 180;
      const horizontal = Math.cos(elevation) * distance;
      return {
        x: Math.sin(azimuth) * horizontal,
        y: Math.sin(elevation) * distance,
        z: Math.cos(azimuth) * horizontal,
      };
    }

    project3d(point, camera) {
      const relative = this.subtract3(point, camera.position);
      const depth = this.dot3(relative, camera.forward);
      if (!Number.isFinite(depth) || depth <= 0.08) {
        return null;
      }
      const horizontal = this.dot3(relative, camera.right);
      const vertical = this.dot3(relative, camera.up);
      return {
        x: camera.centerX + (horizontal * camera.focal) / depth,
        y: camera.centerY - (vertical * camera.focal) / depth,
        depth,
      };
    }

    subtract3(left, right) {
      return {
        x: left.x - right.x,
        y: left.y - right.y,
        z: left.z - right.z,
      };
    }

    dot3(left, right) {
      return left.x * right.x + left.y * right.y + left.z * right.z;
    }

    cross3(left, right) {
      return {
        x: left.y * right.z - left.z * right.y,
        y: left.z * right.x - left.x * right.z,
        z: left.x * right.y - left.y * right.x,
      };
    }

    normalize3(vector) {
      const magnitude = Math.hypot(vector.x, vector.y, vector.z);
      if (!Number.isFinite(magnitude) || magnitude <= 0) {
        return { x: 0, y: 0, z: 1 };
      }
      return {
        x: vector.x / magnitude,
        y: vector.y / magnitude,
        z: vector.z / magnitude,
      };
    }

    drawBackground(ctx, width, height, robotX, robotY, scale, maxRange) {
      const fanRadius = maxRange * scale;
      const left = this.polarPoint(robotX, robotY, fanRadius, FOV_MIN_DEG);
      const right = this.polarPoint(robotX, robotY, fanRadius, FOV_MAX_DEG);
      const gradient = ctx.createRadialGradient(
        robotX,
        robotY,
        0,
        robotX,
        robotY,
        fanRadius,
      );
      gradient.addColorStop(0, "rgba(13, 38, 41, 0.98)");
      gradient.addColorStop(0.55, "rgba(5, 17, 22, 0.98)");
      gradient.addColorStop(1, "rgba(2, 7, 10, 1)");
      ctx.fillStyle = "#020608";
      ctx.fillRect(0, 0, width, height);
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(left.x, left.y);
      ctx.arc(
        robotX,
        robotY,
        fanRadius,
        this.canvasAngle(FOV_MIN_DEG),
        this.canvasAngle(FOV_MAX_DEG),
      );
      ctx.lineTo(robotX, robotY);
      ctx.closePath();
      ctx.fillStyle = gradient;
      ctx.fill();

      ctx.save();
      ctx.strokeStyle = "rgba(117, 205, 192, 0.15)";
      ctx.fillStyle = "rgba(185, 222, 216, 0.58)";
      ctx.lineWidth = 1;
      ctx.font = "10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      const ringStep = maxRange <= 3 ? 0.5 : 1;
      for (
        let meters = ringStep;
        meters <= maxRange + 0.001;
        meters += ringStep
      ) {
        ctx.beginPath();
        ctx.arc(
          robotX,
          robotY,
          meters * scale,
          this.canvasAngle(FOV_MIN_DEG),
          this.canvasAngle(FOV_MAX_DEG),
        );
        ctx.stroke();
        if (
          meters === ringStep ||
          Number.isInteger(meters) ||
          meters >= maxRange
        ) {
          ctx.fillText(
            `${meters.toFixed(meters < 1 ? 1 : 0)} m`,
            robotX + 4,
            robotY - meters * scale + 12,
          );
        }
      }
      Array.from(
        { length: 6 },
        (_, index) => FOV_MIN_DEG + index * SECTOR_WIDTH_DEG,
      ).forEach((degrees) => {
        const edge = this.polarPoint(
          robotX,
          robotY,
          maxRange * scale,
          degrees,
        );
        ctx.beginPath();
        ctx.moveTo(robotX, robotY);
        ctx.lineTo(edge.x, edge.y);
        ctx.stroke();
      });
      ctx.strokeStyle = "rgba(117, 205, 192, 0.3)";
      ctx.beginPath();
      ctx.moveTo(left.x, left.y);
      ctx.arc(
        robotX,
        robotY,
        fanRadius,
        this.canvasAngle(FOV_MIN_DEG),
        this.canvasAngle(FOV_MAX_DEG),
      );
      ctx.lineTo(right.x, right.y);
      ctx.stroke();
      const blindRadius = VALID_MIN_RANGE_M * scale;
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.arc(
        robotX,
        robotY,
        blindRadius,
        this.canvasAngle(FOV_MIN_DEG),
        this.canvasAngle(FOV_MAX_DEG),
      );
      ctx.closePath();
      ctx.fillStyle = "rgba(255, 81, 81, 0.12)";
      ctx.fill();
      ctx.fillStyle = "rgba(255, 163, 154, 0.72)";
      ctx.font = "9px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(
        `${VALID_MIN_RANGE_M.toFixed(2)} m 미만 제외`,
        robotX,
        robotY - blindRadius - 5,
      );
      ctx.restore();
    }

    drawIntensity(ctx, robotX, robotY, scale, maxRange, unsafe) {
      if (this.isHeatmapFresh()) {
        this.drawHeatmap(
          ctx,
          robotX,
          robotY,
          scale,
          maxRange,
          unsafe,
        );
      }
      this.drawOccupancy(
        ctx,
        robotX,
        robotY,
        scale,
        maxRange,
        unsafe,
      );
    }

    drawHeatmap(ctx, robotX, robotY, scale, maxRange, unsafe) {
      const meta = this.heatmapMeta;
      const rangeBins = Number(meta.range_bins);
      const azimuthBins = Number(meta.azimuth_bins);
      const rangeStep = Number(meta.range_step_m);
      const floorDb = Number(meta.floor_db);
      const ceilingDb = Number(meta.ceiling_db);
      const spanDb = ceilingDb - floorDb;
      const validMin = Math.max(
        VALID_MIN_RANGE_M,
        Number.isFinite(meta.valid_min_range_m)
          ? Number(meta.valid_min_range_m)
          : VALID_MIN_RANGE_M,
      );
      const validMax = Math.min(
        VALID_MAX_RANGE_M,
        Number.isFinite(meta.valid_max_range_m)
          ? Number(meta.valid_max_range_m)
          : VALID_MAX_RANGE_M,
      );
      const rangeStride = Math.max(1, Math.ceil(rangeBins / 150));
      const angleStride = Math.max(1, Math.ceil(azimuthBins / 96));
      const now = Date.now();

      ctx.save();
      this.clipFan(ctx, robotX, robotY, maxRange * scale);
      for (
        let rangeIndex = 0;
        rangeIndex < rangeBins;
        rangeIndex += rangeStride
      ) {
        const nearRange = Math.max(validMin, rangeIndex * rangeStep);
        if (nearRange >= maxRange || nearRange >= validMax) {
          break;
        }
        const farRange = Math.min(
          maxRange,
          validMax,
          (rangeIndex + rangeStride + 0.35) * rangeStep,
        );
        if (farRange <= nearRange) {
          continue;
        }
        for (
          let angleIndex = 0;
          angleIndex < azimuthBins;
          angleIndex += angleStride
        ) {
          let peakDb = -Infinity;
          const rangeEnd = Math.min(rangeBins, rangeIndex + rangeStride);
          const angleEnd = Math.min(azimuthBins, angleIndex + angleStride);
          for (
            let sampleRange = rangeIndex;
            sampleRange < rangeEnd;
            sampleRange += 1
          ) {
            const offset = sampleRange * azimuthBins;
            for (
              let sampleAngle = angleIndex;
              sampleAngle < angleEnd;
              sampleAngle += 1
            ) {
              peakDb = Math.max(
                peakDb,
                this.decayedHeatmapDb(offset + sampleAngle, now),
              );
            }
          }
          const intensity = Math.max(
            0,
            Math.min(1, (peakDb - floorDb) / spanDb),
          );
          if (intensity < 0.055) {
            continue;
          }
          const angles = this.heatmapCellAngles(
            meta,
            angleIndex,
            angleEnd,
          );
          if (!angles) {
            continue;
          }
          const [angle0, angle1] = angles;
          ctx.beginPath();
          const p0 = this.polarPoint(
            robotX,
            robotY,
            nearRange * scale,
            angle0,
          );
          const p1 = this.polarPoint(
            robotX,
            robotY,
            farRange * scale,
            angle0,
          );
          const p2 = this.polarPoint(
            robotX,
            robotY,
            farRange * scale,
            angle1,
          );
          const p3 = this.polarPoint(
            robotX,
            robotY,
            nearRange * scale,
            angle1,
          );
          ctx.moveTo(p0.x, p0.y);
          ctx.lineTo(p1.x, p1.y);
          ctx.lineTo(p2.x, p2.y);
          ctx.lineTo(p3.x, p3.y);
          ctx.closePath();
          ctx.fillStyle = this.heatmapColor(
            (nearRange + farRange) / 2,
            maxRange,
            intensity,
            unsafe,
            0.76,
          );
          ctx.fill();
        }
      }
      ctx.restore();
    }

    drawOccupancy(ctx, robotX, robotY, scale, maxRange, unsafe) {
      const occupancy = this.state && this.state.occupancy;
      if (!occupancy || !Array.isArray(occupancy.points)) {
        return;
      }
      const persistenceMs = Number(this.persistenceSelect.value);
      const points = occupancy.points;
      ctx.save();
      this.clipFan(ctx, robotX, robotY, maxRange * scale);
      points.forEach((point) => {
        const forward = Number(point[0]);
        const lateral = Number(point[1]);
        const snr = point[4] === null ? 14 : Number(point[4]);
        const ageMs = Number(point[5]);
        if (
          !Number.isFinite(forward) ||
          !Number.isFinite(lateral) ||
          !Number.isFinite(ageMs) ||
          ageMs > persistenceMs
        ) {
          return;
        }
        const distance = Math.hypot(forward, lateral);
        const angle = (Math.atan2(lateral, forward) * 180) / Math.PI;
        if (
          forward <= 0 ||
          distance < VALID_MIN_RANGE_M ||
          distance > maxRange ||
          distance > VALID_MAX_RANGE_M ||
          angle < FOV_MIN_DEG ||
          angle > FOV_MAX_DEG
        ) {
          return;
        }
        const fade = Math.max(0, 1 - ageMs / persistenceMs);
        const strength = Math.max(
          0.18,
          Math.min(1, (Number.isFinite(snr) ? snr : 14) / 28),
        );
        const x = robotX + lateral * scale;
        const y = robotY - forward * scale;
        const radius = Math.max(9, scale * (0.08 + strength * 0.1));
        const gradient = ctx.createRadialGradient(x, y, 0, x, y, radius);
        gradient.addColorStop(
          0,
          this.intensityColor(
            strength,
            distance,
            unsafe,
            0.34 * fade,
          ),
        );
        gradient.addColorStop(
          1,
          this.intensityColor(strength, distance, unsafe, 0),
        );
        ctx.fillStyle = gradient;
        ctx.fillRect(x - radius, y - radius, radius * 2, radius * 2);
      });
      ctx.restore();
    }

    drawRawPoints(ctx, robotX, robotY, scale, maxRange, unsafe) {
      const frame = this.state && this.state.frame;
      if (!frame || !Array.isArray(frame.points)) {
        return;
      }
      frame.points.forEach((point) => {
        const forward = Number(point[0]);
        const lateral = Number(point[1]);
        const velocity = Number(point[3]);
        const distance = Math.hypot(forward, lateral);
        if (
          forward <= 0 ||
          distance < VALID_MIN_RANGE_M ||
          distance > maxRange ||
          distance > VALID_MAX_RANGE_M
        ) {
          return;
        }
        let color = unsafe ? "rgba(180, 188, 186, 0.5)" : "#e9f4f1";
        if (!unsafe && velocity < -0.15) {
          color = "#ff6b4a";
        } else if (!unsafe && velocity > 0.15) {
          color = "#55bfff";
        }
        ctx.beginPath();
        ctx.fillStyle = color;
        ctx.arc(
          robotX + lateral * scale,
          robotY - forward * scale,
          2.4,
          0,
          Math.PI * 2,
        );
        ctx.fill();
      });
    }

    drawSectorArcs(ctx, robotX, robotY, scale) {
      const nearRadius = DANGER_RANGE_M * scale;
      const farRadius = CAUTION_RANGE_M * scale;
      this.lastSectorStats.forEach((sector, index) => {
        if (sector.level === "unknown") {
          return;
        }
        let color = "rgba(255, 193, 91, 0.18)";
        if (sector.level === "near") {
          color = "rgba(255, 122, 69, 0.28)";
        } else if (
          sector.level === "danger" ||
          sector.level === "invalid"
        ) {
          color = "rgba(255, 81, 81, 0.38)";
        }
        const angle0 = FOV_MIN_DEG + index * SECTOR_WIDTH_DEG;
        const angle1 = angle0 + SECTOR_WIDTH_DEG;
        ctx.beginPath();
        ctx.arc(
          robotX,
          robotY,
          farRadius,
          this.canvasAngle(angle0),
          this.canvasAngle(angle1),
        );
        ctx.arc(
          robotX,
          robotY,
          nearRadius,
          this.canvasAngle(angle1),
          this.canvasAngle(angle0),
          true,
        );
        ctx.closePath();
        ctx.fillStyle = color;
        ctx.fill();
      });
    }

    drawRobot(ctx, robotX, robotY, unsafe) {
      ctx.save();
      ctx.translate(robotX, robotY);
      ctx.fillStyle = unsafe ? "#ff5151" : "#ffc15b";
      ctx.strokeStyle = unsafe ? "#ffd0d0" : "#fff0cf";
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(0, -20);
      ctx.lineTo(13, 7);
      ctx.lineTo(7, 14);
      ctx.lineTo(-7, 14);
      ctx.lineTo(-13, 7);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#071015";
      ctx.fillRect(-4, 0, 8, 10);
      ctx.restore();
    }

    drawOverlay(ctx, width, height, status) {
      const occupancy = this.state && this.state.occupancy;
      const noEvidence =
        !occupancy ||
        !Array.isArray(occupancy.points) ||
        occupancy.points.length === 0;
      if (status === "live") {
        if (noEvidence && !this.smoothedHeatmap) {
          this.overlayMessage(
            ctx,
            width,
            height,
            "반사 없음 ≠ 통로",
            "현재 구역은 미확인입니다",
            "rgba(255, 193, 91, 0.94)",
            false,
          );
        }
        return;
      }
      if (status === "degraded") {
        this.overlayMessage(
          ctx,
          width,
          height,
          "데이터 품질 저하",
          "누락 또는 불완전 프레임 · 저속 확인",
          "rgba(255, 193, 91, 0.94)",
          false,
        );
        return;
      }
      const messages = {
        waiting: ["레이더 대기", "새 프레임 수신 전 주행 금지"],
        stale: ["오래된 화면", "표시가 동결되었습니다 · 즉시 정지"],
        fault: ["레이더 연결 끊김", "표시를 신뢰하지 말고 즉시 정지"],
        replay_end: ["재생 종료", "마지막 프레임 고정 · 주행 금지"],
      };
      const message = messages[status] || messages.fault;
      this.overlayMessage(
        ctx,
        width,
        height,
        message[0],
        message[1],
        "rgba(255, 81, 81, 0.98)",
        true,
      );
    }

    overlayMessage(ctx, width, height, title, subtitle, color, blockView) {
      if (blockView) {
        ctx.fillStyle = "rgba(2, 5, 7, 0.76)";
        ctx.fillRect(0, 0, width, height);
      }
      const boxWidth = Math.min(440, width - 30);
      const boxHeight = 86;
      const x = (width - boxWidth) / 2;
      const y = Math.max(50, height * 0.35);
      ctx.fillStyle = "rgba(3, 8, 11, 0.92)";
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.fillRect(x, y, boxWidth, boxHeight);
      ctx.strokeRect(x, y, boxWidth, boxHeight);
      ctx.fillStyle = color;
      ctx.textAlign = "center";
      ctx.font = "750 21px system-ui, sans-serif";
      ctx.fillText(title, width / 2, y + 34);
      ctx.fillStyle = "rgba(239, 248, 246, 0.9)";
      ctx.font = "600 12px system-ui, sans-serif";
      ctx.fillText(subtitle, width / 2, y + 61);
    }

    updateSectors() {
      const stats = this.computeSectorStats();
      this.lastSectorStats = stats;
      stats.forEach((sector, index) => {
        const element = document.querySelector(`#sector-${index}`);
        element.dataset.level = sector.level;
        element.querySelector("b").textContent = SECTOR_NAMES[index];
        element.querySelector("span").textContent = sector.label;
      });
    }

    computeSectorStats() {
      const stats = this.emptySectorStats();
      const status = this.currentStatus();
      if (["waiting", "stale", "fault", "replay_end"].includes(status)) {
        return stats.map((sector) =>
          Object.assign(sector, { level: "invalid", label: "정지" }),
        );
      }
      const maxRange = Number(this.rangeSelect.value);
      const persistenceMs = Number(this.persistenceSelect.value);
      const occupancy = this.state && this.state.occupancy;
      if (occupancy && Array.isArray(occupancy.points)) {
        occupancy.points.forEach((point) => {
          const forward = Number(point[0]);
          const lateral = Number(point[1]);
          const ageMs = Number(point[5]);
          const distance = Math.hypot(forward, lateral);
          const angle = (Math.atan2(lateral, forward) * 180) / Math.PI;
          if (
            forward <= 0 ||
            distance < VALID_MIN_RANGE_M ||
            distance > maxRange ||
            distance > VALID_MAX_RANGE_M ||
            ageMs > persistenceMs ||
            angle < FOV_MIN_DEG ||
            angle > FOV_MAX_DEG
          ) {
            return;
          }
          const sectorIndex = Math.min(
            4,
            Math.max(
              0,
              Math.floor((angle - FOV_MIN_DEG) / SECTOR_WIDTH_DEG),
            ),
          );
          const sector = stats[sectorIndex];
          sector.evidence += Math.max(0, 1 - ageMs / persistenceMs);
          sector.nearest = Math.min(sector.nearest, distance);
          sector.pointNearest = Math.min(sector.pointNearest, distance);
        });
      }
      this.addHeatmapSectorEvidence(stats, maxRange);
      return stats.map((sector) => {
        if (sector.pointNearest <= DANGER_RANGE_M) {
          sector.level = "danger";
          sector.label = `${sector.pointNearest.toFixed(2)}m 실측`;
        } else if (sector.pointNearest <= CAUTION_RANGE_M) {
          sector.level = "near";
          sector.label = `${sector.pointNearest.toFixed(2)}m 실측`;
        } else if (sector.evidence >= 0.3) {
          sector.level = "evidence";
          sector.label = Number.isFinite(sector.nearest)
            ? `${sector.nearest.toFixed(1)}m 반사`
            : "반사 있음";
        } else {
          sector.level = "unknown";
          sector.label = "미확인";
        }
        return sector;
      });
    }

    addHeatmapSectorEvidence(stats, maxRange) {
      if (!this.isHeatmapFresh()) {
        return;
      }
      const meta = this.heatmapMeta;
      const rangeBins = Number(meta.range_bins);
      const azimuthBins = Number(meta.azimuth_bins);
      const rangeStep = Number(meta.range_step_m);
      const validMin = Math.max(
        VALID_MIN_RANGE_M,
        Number.isFinite(meta.valid_min_range_m)
          ? Number(meta.valid_min_range_m)
          : VALID_MIN_RANGE_M,
      );
      const validMax = Math.min(
        VALID_MAX_RANGE_M,
        Number.isFinite(meta.valid_max_range_m)
          ? Number(meta.valid_max_range_m)
          : VALID_MAX_RANGE_M,
      );
      const floorDb = Number(meta.floor_db);
      const ceilingDb = Number(meta.ceiling_db);
      const supportThresholdDb =
        floorDb + (ceilingDb - floorDb) * 0.64;
      const now = Date.now();
      for (let rangeIndex = 0; rangeIndex < rangeBins; rangeIndex += 1) {
        const distance = rangeIndex * rangeStep;
        if (distance > maxRange || distance > validMax) {
          break;
        }
        if (distance < validMin) {
          continue;
        }
        const offset = rangeIndex * azimuthBins;
        for (
          let azimuthIndex = 0;
          azimuthIndex < azimuthBins;
          azimuthIndex += 1
        ) {
          const db = this.decayedHeatmapDb(
            offset + azimuthIndex,
            now,
          );
          if (
            db < supportThresholdDb ||
            !this.hasSpatialHeatmapSupport(
              rangeIndex,
              azimuthIndex,
              supportThresholdDb,
              now,
            )
          ) {
            continue;
          }
          const angle = this.heatmapBinCenterAngle(meta, azimuthIndex);
          if (angle === null) {
            continue;
          }
          const sectorIndex = Math.min(
            4,
            Math.max(
              0,
              Math.floor((angle - FOV_MIN_DEG) / SECTOR_WIDTH_DEG),
            ),
          );
          const confidence = Math.max(
            0,
            Math.min(
              1,
              (db - supportThresholdDb) /
                Math.max(1, ceilingDb - supportThresholdDb),
            ),
          );
          stats[sectorIndex].evidence += 0.08 + confidence * 0.08;
          stats[sectorIndex].nearest = Math.min(
            stats[sectorIndex].nearest,
            distance,
          );
        }
      }
    }

    hasSpatialHeatmapSupport(
      rangeIndex,
      azimuthIndex,
      thresholdDb,
      now,
    ) {
      const rangeBins = Number(this.heatmapMeta.range_bins);
      const azimuthBins = Number(this.heatmapMeta.azimuth_bins);
      let supportedCells = 0;
      for (let rangeOffset = -1; rangeOffset <= 1; rangeOffset += 1) {
        const candidateRange = rangeIndex + rangeOffset;
        if (candidateRange < 0 || candidateRange >= rangeBins) {
          continue;
        }
        for (let angleOffset = -1; angleOffset <= 1; angleOffset += 1) {
          const candidateAngle = azimuthIndex + angleOffset;
          if (candidateAngle < 0 || candidateAngle >= azimuthBins) {
            continue;
          }
          const index = candidateRange * azimuthBins + candidateAngle;
          if (this.decayedHeatmapDb(index, now) >= thresholdDb) {
            supportedCells += 1;
            if (supportedCells >= 3) {
              return true;
            }
          }
        }
      }
      return false;
    }

    heatmapBinCenterAngle(meta, index) {
      const bins = Number(meta.azimuth_bins);
      let angle;
      if (meta.azimuth_layout === "fft-shifted-spatial-frequency") {
        const lambdaOverDx = Number(meta.lambda_over_d_x);
        const spatial =
          (lambdaOverDx * (index - bins / 2)) / bins;
        if (!Number.isFinite(spatial) || Math.abs(spatial) > 1) {
          return null;
        }
        angle = (Math.asin(spatial) * 180) / Math.PI;
      } else {
        const minAngle = Number(meta.azimuth_min_deg);
        const maxAngle = Number(meta.azimuth_max_deg);
        angle =
          minAngle + ((index + 0.5) / bins) * (maxAngle - minAngle);
      }
      const validMin = Math.max(
        FOV_MIN_DEG,
        Number.isFinite(meta.azimuth_min_deg)
          ? Number(meta.azimuth_min_deg)
          : FOV_MIN_DEG,
      );
      const validMax = Math.min(
        FOV_MAX_DEG,
        Number.isFinite(meta.azimuth_max_deg)
          ? Number(meta.azimuth_max_deg)
          : FOV_MAX_DEG,
      );
      return angle >= validMin && angle <= validMax ? angle : null;
    }

    heatmapBinEdgeAngle(meta, edgeIndex) {
      const bins = Number(meta.azimuth_bins);
      if (meta.azimuth_layout === "fft-shifted-spatial-frequency") {
        const lambdaOverDx = Number(meta.lambda_over_d_x);
        const spatial =
          (lambdaOverDx * (edgeIndex - 0.5 - bins / 2)) / bins;
        if (!Number.isFinite(spatial)) {
          return null;
        }
        return (
          (Math.asin(Math.max(-1, Math.min(1, spatial))) * 180) /
          Math.PI
        );
      }
      const minAngle = Number(meta.azimuth_min_deg);
      const maxAngle = Number(meta.azimuth_max_deg);
      if (!Number.isFinite(minAngle) || !Number.isFinite(maxAngle)) {
        return null;
      }
      return minAngle + (edgeIndex / bins) * (maxAngle - minAngle);
    }

    heatmapCellAngles(meta, startIndex, endIndex) {
      const first = this.heatmapBinEdgeAngle(meta, startIndex);
      const second = this.heatmapBinEdgeAngle(meta, endIndex);
      if (first === null || second === null) {
        return null;
      }
      const validMin = Math.max(
        FOV_MIN_DEG,
        Number.isFinite(meta.azimuth_min_deg)
          ? Number(meta.azimuth_min_deg)
          : FOV_MIN_DEG,
      );
      const validMax = Math.min(
        FOV_MAX_DEG,
        Number.isFinite(meta.azimuth_max_deg)
          ? Number(meta.azimuth_max_deg)
          : FOV_MAX_DEG,
      );
      const low = Math.max(validMin, Math.min(first, second));
      const high = Math.min(validMax, Math.max(first, second));
      return high > low ? [low, high] : null;
    }

    emptySectorStats() {
      return Array.from({ length: 5 }, () => ({
        evidence: 0,
        nearest: Infinity,
        pointNearest: Infinity,
        level: "unknown",
        label: "미확인",
      }));
    }

    intensityColor(intensity, distance, unsafe, alphaScale) {
      const alpha = Math.max(0, Math.min(0.92, intensity * alphaScale));
      if (unsafe) {
        const grey = Math.round(105 + intensity * 95);
        return `rgba(${grey}, ${grey}, ${grey}, ${alpha * 0.5})`;
      }
      if (distance <= DANGER_RANGE_M && intensity > 0.28) {
        return `rgba(255, 74, 67, ${alpha})`;
      }
      if (distance <= CAUTION_RANGE_M && intensity > 0.28) {
        return `rgba(255, 132, 62, ${alpha})`;
      }
      if (intensity > 0.76) {
        return `rgba(255, 177, 67, ${alpha})`;
      }
      if (intensity > 0.42) {
        return `rgba(92, 228, 208, ${alpha})`;
      }
      return `rgba(48, 143, 160, ${alpha * 0.78})`;
    }

    clipFan(ctx, robotX, robotY, radius) {
      const left = this.polarPoint(robotX, robotY, radius, FOV_MIN_DEG);
      ctx.beginPath();
      ctx.moveTo(robotX, robotY);
      ctx.lineTo(left.x, left.y);
      ctx.arc(
        robotX,
        robotY,
        radius,
        this.canvasAngle(FOV_MIN_DEG),
        this.canvasAngle(FOV_MAX_DEG),
      );
      ctx.closePath();
      ctx.clip();
    }

    polarPoint(robotX, robotY, radius, angleDegrees) {
      const angle = (angleDegrees * Math.PI) / 180;
      return {
        x: robotX + Math.sin(angle) * radius,
        y: robotY - Math.cos(angle) * radius,
      };
    }

    canvasAngle(angleDegrees) {
      return ((angleDegrees - 90) * Math.PI) / 180;
    }
  }

  window.HanselRadarPanel = RadarPanel;
})();
