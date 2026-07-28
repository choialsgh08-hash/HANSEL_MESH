(function () {
  "use strict";

  const UI_BUILD_ID = "20260729-lidar-operator-r10";
  const MAIN_MAX_RANGE_M = 3.0;
  const MAIN_HALF_WIDTH_M = 1.5;
  const CLOSE_MAX_RANGE_M = 0.5;
  const DANGER_RANGE_M = 0.1;
  const TRACK_MAX_AGE_MS = 300;
  const NORMAL_COPY =
    "10cm 이내 확인 장애물 없음 · 미관측 영역 존재";
  const STATUS_LABELS = {
    live: "LIVE",
    degraded: "DEGRADED",
    stale: "STALE",
    fault: "FAULT",
    waiting: "WAITING",
    replay_end: "REPLAY END",
    sensor_fault: "SENSOR FAULT",
    http_lost: "HTTP LOST",
  };
  const SECTOR_NAMES = ["좌측 끝", "좌측", "정면", "우측", "우측 끝"];

  class RadarPanel {
    constructor(root, options) {
      if (!root) {
        throw new Error("radar panel root is required");
      }
      if (!window.HanselRadarScene) {
        throw new Error("HanselRadarScene must load before RadarPanel");
      }
      this.root = root;
      this.mainCanvas = root.querySelector("#radar-main-canvas");
      this.collisionCanvas = root.querySelector("#collision-canvas");
      if (!this.mainCanvas || !this.collisionCanvas) {
        throw new Error("radar map canvases are required");
      }
      this.mainContext = this.mainCanvas.getContext("2d");
      this.collisionContext = this.collisionCanvas.getContext("2d");
      this.fullscreenButton = document.querySelector("#fullscreen-button");
      this.rawToggle = document.querySelector("#raw-toggle");
      this.options = Object.assign({ endpoint: "/api/radar" }, options || {});
      this.snapshot = null;
      this.presentation = {
        blocked: true,
        reason: "waiting",
      };
      this.fetchFailed = false;
      this.lastError = null;

      if (typeof ResizeObserver === "function") {
        this.resizeObserver = new ResizeObserver(() => this.draw());
        this.resizeObserver.observe(this.root);
        this.resizeObserver.observe(this.collisionCanvas);
      } else {
        window.addEventListener("resize", () => this.draw());
      }
      if (this.rawToggle) {
        this.rawToggle.addEventListener("change", () => this.draw());
      }
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
      this.updateText(this.presentation);
      this.updateDiagnostics(this.presentation);
      this.updateSectors(this.presentation);
      this.draw();
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
          this.acceptSnapshot(await response.json());
        } catch (error) {
          this.fetchFailed = true;
          this.lastError = error;
          this.presentation = {
            blocked: true,
            reason: "http_lost",
            error,
          };
          this.updateText(this.presentation);
          this.updateDiagnostics(this.presentation);
          this.updateSectors(this.presentation);
          this.draw();
        } finally {
          window.clearTimeout(timeout);
        }
        await new Promise((resolve) => window.setTimeout(resolve, 100));
      }
    }

    acceptSnapshot(snapshot) {
      if (
        snapshot &&
        snapshot.ui_build_id &&
        snapshot.ui_build_id !== UI_BUILD_ID
      ) {
        window.location.reload();
        return;
      }
      this.snapshot = snapshot;
      this.fetchFailed = false;
      this.lastError = null;
      try {
        this.presentation =
          window.HanselRadarScene.parseRadarScene(snapshot);
      } catch (error) {
        this.presentation = {
          blocked: true,
          reason: "invalid_scene",
          error,
        };
      }
      this.updateText(this.presentation);
      this.updateDiagnostics(this.presentation);
      this.updateSectors(this.presentation);
      this.draw();
    }

    resizeCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const ratio = Math.max(1, window.devicePixelRatio || 1);
      const width = Math.max(1, Math.floor(rect.width));
      const height = Math.max(1, Math.floor(rect.height));
      const pixelWidth = Math.floor(width * ratio);
      const pixelHeight = Math.floor(height * ratio);
      if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
        canvas.width = pixelWidth;
        canvas.height = pixelHeight;
      }
      const ctx = canvas.getContext("2d");
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      return { ctx, width, height };
    }

    draw() {
      const main = this.resizeCanvas(this.mainCanvas);
      const collision = this.resizeCanvas(this.collisionCanvas);
      this.clearCanvas(main.ctx, main.width, main.height);
      this.clearCanvas(collision.ctx, collision.width, collision.height);

      const presentation = this.presentation;
      if (!presentation || presentation.blocked) {
        this.drawBlockingOverlay(
          main.ctx,
          main.width,
          main.height,
          presentation || { blocked: true, reason: "waiting" },
        );
        this.drawBlockingOverlay(
          collision.ctx,
          collision.width,
          collision.height,
          presentation || { blocked: true, reason: "waiting" },
        );
        return;
      }

      const mainTransform = this.makeViewportTransform(
        main.width,
        main.height,
        MAIN_MAX_RANGE_M,
        MAIN_HALF_WIDTH_M,
        { top: 50, right: 26, bottom: 64, left: 26 },
      );
      const closeTransform = this.makeViewportTransform(
        collision.width,
        collision.height,
        CLOSE_MAX_RANGE_M,
        CLOSE_MAX_RANGE_M,
        { top: 16, right: 15, bottom: 28, left: 15 },
      );
      this.drawLidarTopView(main.ctx, mainTransform, presentation);
      this.drawCollisionInset(
        collision.ctx,
        closeTransform,
        presentation,
      );
    }

    clearCanvas(ctx, width, height) {
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#020609";
      ctx.fillRect(0, 0, width, height);
    }

    makeViewportTransform(
      width,
      height,
      forwardMaxM,
      halfWidthM,
      margin,
    ) {
      const mapWidth = Math.max(1, width - margin.left - margin.right);
      const mapHeight = Math.max(1, height - margin.top - margin.bottom);
      const base = window.HanselRadarScene.makeMapTransform(
        mapWidth,
        mapHeight,
        forwardMaxM,
        halfWidthM,
      );
      return {
        originX: margin.left + mapWidth / 2,
        originY: margin.top + mapHeight,
        scale: base.scale,
        width,
        height,
        forwardMaxM,
        halfWidthM,
      };
    }

    drawLidarTopView(ctx, transform, scene) {
      this.drawMetricGrid(ctx, transform, {
        forwardMaxM: MAIN_MAX_RANGE_M,
        halfWidthM: MAIN_HALF_WIDTH_M,
        rangeStepM: 0.5,
        labelUnit: "m",
        clipShape: "rectangular",
      });
      ctx.save();
      this.clipToMapBoundary(ctx, transform, "rectangular");
      this.drawEvidenceGrid(ctx, transform, scene, {
        clipShape: "rectangular",
      });
      this.drawTracks(ctx, transform, scene.tracks, {
        scene,
        labelLimit: 5,
        maxRangeM: MAIN_MAX_RANGE_M,
        clipShape: "rectangular",
      });
      if (this.rawToggle && this.rawToggle.checked) {
        this.drawRawDebugPoints(ctx, transform, this.snapshot && this.snapshot.frame);
      }
      ctx.restore();
      this.drawRobot(ctx, transform);
      ctx.fillStyle = "rgba(225, 246, 243, 0.72)";
      ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "left";
      ctx.fillText(
        `CURRENT EVIDENCE · TRACK TTL < ${TRACK_MAX_AGE_MS}ms`,
        14,
        transform.height - 16,
      );
    }

    drawCollisionInset(ctx, transform, scene) {
      this.drawMetricGrid(ctx, transform, {
        forwardMaxM: CLOSE_MAX_RANGE_M,
        halfWidthM: CLOSE_MAX_RANGE_M,
        rangeStepM: 0.1,
        labelUnit: "cm",
        clipShape: "radial",
      });
      ctx.save();
      this.clipToMapBoundary(ctx, transform, "radial");
      this.drawEvidenceGrid(ctx, transform, scene, {
        clipShape: "radial",
      });
      this.drawTracks(ctx, transform, scene.tracks, {
        scene,
        labelLimit: 3,
        maxRangeM: CLOSE_MAX_RANGE_M,
        close: true,
        clipShape: "radial",
      });
      ctx.restore();
      this.drawRobot(ctx, transform);
    }

    drawMetricGrid(ctx, transform, options) {
      const {
        forwardMaxM,
        halfWidthM,
        rangeStepM,
        labelUnit,
      } = options;
      const left = transform.originX - halfWidthM * transform.scale;
      const right = transform.originX + halfWidthM * transform.scale;
      const top = transform.originY - forwardMaxM * transform.scale;

      ctx.save();
      this.clipToMapBoundary(ctx, transform, options.clipShape);
      ctx.strokeStyle = "rgba(92, 228, 208, 0.13)";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 5]);
      for (
        let rangeM = rangeStepM;
        rangeM <= forwardMaxM + 0.0001;
        rangeM += rangeStepM
      ) {
        this.drawRangeGuideArc(ctx, transform, rangeM);
        ctx.fillStyle = "rgba(176, 210, 206, 0.74)";
        ctx.font = "650 9px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "left";
        const label = labelUnit === "cm"
          ? `${Math.round(rangeM * 100)}cm`
          : `${rangeM.toFixed(rangeM % 1 === 0 ? 0 : 1)}m`;
        ctx.fillText(
          label,
          transform.originX + 5,
          transform.originY - rangeM * transform.scale + 10,
        );
      }
      const lateralStep = labelUnit === "cm" ? 0.1 : 0.5;
      for (
        let lateralM = -halfWidthM;
        lateralM <= halfWidthM + 0.0001;
        lateralM += lateralStep
      ) {
        const x = transform.originX + lateralM * transform.scale;
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, transform.originY);
        ctx.stroke();
      }
      ctx.setLineDash([]);
      ctx.strokeStyle = "rgba(92, 228, 208, 0.42)";
      ctx.beginPath();
      ctx.moveTo(transform.originX, top);
      ctx.lineTo(transform.originX, transform.originY);
      ctx.stroke();
      ctx.fillStyle = "rgba(114, 150, 148, 0.48)";
      ctx.font = "700 9px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "left";
      ctx.fillText("UNKNOWN", left + 7, top + 14);
      ctx.restore();
      this.strokeMapBoundary(ctx, transform, options.clipShape);
    }

    drawRangeGuideArc(ctx, transform, rangeM) {
      ctx.beginPath();
      ctx.arc(
        transform.originX,
        transform.originY,
        rangeM * transform.scale,
        Math.PI,
        Math.PI * 2,
      );
      ctx.stroke();
    }

    clipToMapBoundary(ctx, transform, clipShape) {
      const left =
        transform.originX - transform.halfWidthM * transform.scale;
      const top =
        transform.originY - transform.forwardMaxM * transform.scale;
      ctx.beginPath();
      if (clipShape === "radial") {
        ctx.moveTo(transform.originX, transform.originY);
        ctx.lineTo(
          transform.originX - transform.forwardMaxM * transform.scale,
          transform.originY,
        );
        ctx.arc(
          transform.originX,
          transform.originY,
          transform.forwardMaxM * transform.scale,
          Math.PI,
          Math.PI * 2,
        );
        ctx.closePath();
      } else {
        ctx.rect(
          left,
          top,
          transform.halfWidthM * transform.scale * 2,
          transform.forwardMaxM * transform.scale,
        );
      }
      ctx.clip();
    }

    strokeMapBoundary(ctx, transform, clipShape) {
      const left =
        transform.originX - transform.halfWidthM * transform.scale;
      const top =
        transform.originY - transform.forwardMaxM * transform.scale;
      ctx.save();
      ctx.strokeStyle = "rgba(156, 198, 194, 0.28)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      if (clipShape === "radial") {
        ctx.moveTo(
          transform.originX - transform.forwardMaxM * transform.scale,
          transform.originY,
        );
        ctx.arc(
          transform.originX,
          transform.originY,
          transform.forwardMaxM * transform.scale,
          Math.PI,
          Math.PI * 2,
        );
        ctx.closePath();
      } else {
        ctx.rect(
          left,
          top,
          transform.halfWidthM * transform.scale * 2,
          transform.forwardMaxM * transform.scale,
        );
      }
      ctx.stroke();
      ctx.restore();
    }

    drawEvidenceGrid(ctx, transform, scene, options) {
      const grid = scene.grid;
      const meta = scene.gridMeta;
      if (!grid || !meta) {
        return;
      }
      const resolution = Number(meta.resolution_m);
      const forwardCells = Number(meta.forward_cells);
      const lateralCells = Number(meta.lateral_cells);
      const forwardOrigin = Number(meta.origin_forward_cell);
      const lateralOrigin = Number(meta.origin_lateral_cell);
      if (
        !Number.isFinite(resolution) ||
        !Number.isInteger(forwardCells) ||
        !Number.isInteger(lateralCells)
      ) {
        return;
      }
      ctx.save();
      for (let forwardIndex = 0; forwardIndex < forwardCells; forwardIndex += 1) {
        const forward0 = (forwardIndex - forwardOrigin) * resolution;
        const forward1 = forward0 + resolution;
        if (forward1 <= 0 || forward0 >= transform.forwardMaxM) {
          continue;
        }
        for (let lateralIndex = 0; lateralIndex < lateralCells; lateralIndex += 1) {
          const confidenceByte =
            grid[forwardIndex * lateralCells + lateralIndex];
          if (confidenceByte === 0) {
            continue;
          }
          const lateral0 = (lateralIndex - lateralOrigin) * resolution;
          const lateral1 = lateral0 + resolution;
          if (
            lateral1 <= -transform.halfWidthM ||
            lateral0 >= transform.halfWidthM
          ) {
            continue;
          }
          if (
            options.clipShape === "radial" &&
            !this.cellIntersectsRadialLimit(
              forward0,
              forward1,
              lateral0,
              lateral1,
              transform.forwardMaxM,
            )
          ) {
            continue;
          }
          const confidence = confidenceByte / 255;
          const nearCorner = window.HanselRadarScene.projectMapPoint(
            transform,
            forward0,
            lateral0,
          );
          const farCorner = window.HanselRadarScene.projectMapPoint(
            transform,
            forward1,
            lateral1,
          );
          const red = Math.round(70 + confidence * 185);
          const green = Math.round(205 + confidence * 50);
          const blue = Math.round(198 + confidence * 57);
          const alpha = 0.24 + confidence * 0.7;
          ctx.fillStyle =
            `rgba(${red}, ${green}, ${blue}, ${alpha.toFixed(3)})`;
          ctx.fillRect(
            nearCorner.x,
            farCorner.y,
            Math.max(1, farCorner.x - nearCorner.x),
            Math.max(1, nearCorner.y - farCorner.y),
          );
        }
      }
      ctx.restore();
    }

    cellIntersectsRadialLimit(
      forward0,
      forward1,
      lateral0,
      lateral1,
      radialLimitM,
    ) {
      // Include a 5cm evidence cell when any part intersects the radius;
      // the canvas clip removes the portion beyond the semicircle.
      const closestForward = forward0 > 0
        ? forward0
        : forward1 < 0
          ? -forward1
          : 0;
      const closestLateral = lateral0 > 0
        ? lateral0
        : lateral1 < 0
          ? -lateral1
          : 0;
      return (
        Math.hypot(closestForward, closestLateral) <= radialLimitM
      );
    }

    drawTracks(ctx, transform, tracks, options) {
      const freshTracks = (Array.isArray(tracks) ? tracks : [])
        .filter((track) =>
          Number.isFinite(track.forward_m) &&
          Number.isFinite(track.lateral_m) &&
          this.trackInsideMapBounds(track, transform, options))
        .sort((left, right) => {
          const leftDistance = Number.isFinite(left.distance_m)
            ? left.distance_m
            : Math.hypot(left.forward_m, left.lateral_m);
          const rightDistance = Number.isFinite(right.distance_m)
            ? right.distance_m
            : Math.hypot(right.forward_m, right.lateral_m);
          return leftDistance - rightDistance;
        });
      freshTracks.forEach((track, index) => {
        const visualAlpha = this.trackVisualAlpha(track);
        if (visualAlpha <= 0) {
          return;
        }
        const point = window.HanselRadarScene.projectMapPoint(
          transform,
          track.forward_m,
          track.lateral_m,
        );
        const danger = this.isDangerTrack(track, options.scene);
        if (track.source === "heatmap") {
          this.drawHeatmapUncertainty(
            ctx,
            transform,
            track,
            danger,
            visualAlpha,
          );
        } else {
          this.drawPointMarker(
            ctx,
            point,
            track,
            danger,
            options.close,
            visualAlpha,
          );
        }
        if (index < options.labelLimit) {
          this.drawTrackLabel(
            ctx,
            point,
            track,
            danger,
            options.close,
            visualAlpha,
          );
        }
      });
    }

    trackInsideMapBounds(track, transform, options) {
      if (
        track.forward_m < 0 ||
        track.forward_m > transform.forwardMaxM ||
        Math.abs(track.lateral_m) > transform.halfWidthM
      ) {
        return false;
      }
      if (options.clipShape === "radial") {
        return (
          Math.hypot(track.forward_m, track.lateral_m) <=
          options.maxRangeM
        );
      }
      return true;
    }

    trackVisualAlpha(track) {
      const ageMs = Number(track.age_ms);
      if (!Number.isFinite(ageMs) || ageMs < 0) {
        return 0;
      }
      const rawAlpha = 1 - ageMs / TRACK_MAX_AGE_MS;
      return Math.max(0, Math.min(1, rawAlpha));
    }

    drawPointMarker(ctx, point, track, danger, close, visualAlpha) {
      const confirmed = track.point_confirmed === true;
      const radius = close ? 5.5 : confirmed ? 5 : 3.5;
      const color = danger
        ? "#ff5151"
        : confirmed
          ? "#f4ffff"
          : "#66e7d5";
      ctx.save();
      ctx.globalAlpha = visualAlpha;
      ctx.strokeStyle = color;
      ctx.fillStyle = danger
        ? "rgba(255, 81, 81, 0.34)"
        : "rgba(92, 228, 208, 0.2)";
      ctx.lineWidth = confirmed ? 2 : 1.3;
      ctx.beginPath();
      ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(point.x - radius - 3, point.y);
      ctx.lineTo(point.x + radius + 3, point.y);
      ctx.moveTo(point.x, point.y - radius - 3);
      ctx.lineTo(point.x, point.y + radius + 3);
      ctx.stroke();
      if (confirmed) {
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(point.x, point.y, 1.8, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
    }

    drawHeatmapUncertainty(
      ctx,
      transform,
      track,
      danger,
      visualAlpha,
    ) {
      const distance = Number.isFinite(track.distance_m)
        ? track.distance_m
        : Math.hypot(track.forward_m, track.lateral_m);
      if (!Number.isFinite(distance) || distance <= 0) {
        return;
      }
      const angle = Math.atan2(track.lateral_m, track.forward_m);
      const uncertainty = Math.max(
        0.025,
        Number(track.range_uncertainty_m) || 0.05,
      );
      const angularHalfWidth = Math.max(
        Math.PI / 90,
        Math.min(Math.PI / 12, uncertainty / Math.max(distance, 0.05)),
      );
      const radius = distance * transform.scale;
      const canvasStart = angle - angularHalfWidth - Math.PI / 2;
      const canvasEnd = angle + angularHalfWidth - Math.PI / 2;
      ctx.save();
      ctx.globalAlpha = visualAlpha;
      ctx.strokeStyle = danger
        ? "#ff5151"
        : "rgba(92, 228, 208, 0.82)";
      ctx.lineWidth = Math.max(2, uncertainty * transform.scale);
      ctx.beginPath();
      ctx.arc(
        transform.originX,
        transform.originY,
        radius,
        canvasStart,
        canvasEnd,
      );
      ctx.stroke();
      ctx.lineWidth = 1;
      ctx.strokeStyle = danger
        ? "rgba(255, 81, 81, 0.8)"
        : "rgba(207, 248, 243, 0.72)";
      for (const radialOffset of [-uncertainty, uncertainty]) {
        const offsetRadius = Math.max(
          1,
          (distance + radialOffset) * transform.scale,
        );
        ctx.beginPath();
        ctx.arc(
          transform.originX,
          transform.originY,
          offsetRadius,
          canvasStart,
          canvasEnd,
        );
        ctx.stroke();
      }
      ctx.restore();
    }

    drawTrackLabel(
      ctx,
      point,
      track,
      danger,
      close,
      visualAlpha,
    ) {
      const distance = Number.isFinite(track.distance_m)
        ? track.distance_m
        : Math.hypot(track.forward_m, track.lateral_m);
      if (!Number.isFinite(distance)) {
        return;
      }
      const distanceText = close
        ? `${Math.round(distance * 100)}cm`
        : `${distance.toFixed(2)}m`;
      const heightText =
        track.height_m !== null && Number.isFinite(track.height_m)
          ? ` · z ${Number(track.height_m).toFixed(2)}m`
          : "";
      const sourceText = track.source === "point" ? "P" : "H";
      const text = `${sourceText} ${distanceText}${heightText}`;
      ctx.save();
      ctx.globalAlpha = visualAlpha;
      ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
      const width = ctx.measureText(text).width + 10;
      const x = Math.min(
        transform.width - width - 3,
        Math.max(3, point.x + 8),
      );
      const y = Math.min(
        transform.height - 5,
        Math.max(14, point.y - 8),
      );
      ctx.fillStyle = "rgba(2, 8, 11, 0.88)";
      ctx.fillRect(x, y - 12, width, 16);
      ctx.fillStyle = danger ? "#ff7777" : "#dff9f5";
      ctx.textAlign = "left";
      ctx.fillText(text, x + 5, y);
      ctx.restore();
    }

    drawRawDebugPoints(ctx, transform, frame) {
      if (!frame || !Array.isArray(frame.points)) {
        return;
      }
      ctx.save();
      ctx.fillStyle = "rgba(90, 184, 255, 0.48)";
      for (const rawPoint of frame.points) {
        const forward = Number(rawPoint[0]);
        const lateral = Number(rawPoint[1]);
        if (
          !Number.isFinite(forward) ||
          !Number.isFinite(lateral) ||
          forward < 0 ||
          forward > transform.forwardMaxM ||
          Math.abs(lateral) > transform.halfWidthM
        ) {
          continue;
        }
        const point = window.HanselRadarScene.projectMapPoint(
          transform,
          forward,
          lateral,
        );
        ctx.fillRect(point.x - 1, point.y - 1, 2, 2);
      }
      ctx.restore();
    }

    drawRobot(ctx, transform) {
      ctx.save();
      ctx.translate(transform.originX, transform.originY - 2);
      ctx.fillStyle = "#ffc15b";
      ctx.strokeStyle = "#fff0cf";
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(0, -18);
      ctx.lineTo(11, 7);
      ctx.lineTo(6, 12);
      ctx.lineTo(-6, 12);
      ctx.lineTo(-11, 7);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#061014";
      ctx.fillRect(-3.5, 0, 7, 8);
      ctx.restore();
    }

    drawBlockingOverlay(ctx, width, height, presentation) {
      const reason = presentation && presentation.reason;
      const messages = {
        waiting: [
          "RADAR STARTING · DRIVE STOP",
          "레이더 준비 중 · 주행을 정지하세요",
        ],
        stale: [
          "RADAR RECONNECTING · DRIVE STOP",
          "레이더 재연결 중 · 주행을 정지하세요",
        ],
        fault: [
          "RADAR RECONNECTING · DRIVE STOP",
          "레이더 재연결 중 · 주행을 정지하세요",
        ],
        sensor_fault: [
          "RADAR RECONNECTING · DRIVE STOP",
          "레이더 재연결 중 · 주행을 정지하세요",
        ],
        replay_end: ["REPLAY END", "마지막 프레임을 지도처럼 사용하지 마세요"],
        calibration_required: [
          "CALIBRATION REQUIRED",
          "빈 장면 self-clutter 보정 파일이 필요합니다",
        ],
        calibration_unavailable: [
          "CALIBRATION UNAVAILABLE",
          "보정되지 않은 반사를 점유로 표시하지 않습니다",
        ],
        profile_mismatch: [
          "PROFILE MISMATCH",
          "보정 파일과 현재 레이더 프로필이 다릅니다",
        ],
        http_lost: [
          "RADAR RECONNECTING · DRIVE STOP",
          "레이더 재연결 중 · 주행을 정지하세요",
        ],
        invalid_scene: [
          "SCENE CONTRACT ERROR",
          "검증되지 않은 장면은 표시하지 않습니다",
        ],
      };
      const message = messages[reason] || [
        "MAP BLOCKED",
        String(reason || "unknown input state"),
      ];
      ctx.fillStyle = "#020609";
      ctx.fillRect(0, 0, width, height);
      const boxWidth = Math.max(180, Math.min(480, width - 28));
      const boxHeight = Math.min(116, Math.max(86, height - 24));
      const x = (width - boxWidth) / 2;
      const y = (height - boxHeight) / 2;
      ctx.fillStyle = "rgba(8, 17, 21, 0.98)";
      ctx.strokeStyle = "rgba(255, 193, 91, 0.84)";
      ctx.lineWidth = 2;
      ctx.fillRect(x, y, boxWidth, boxHeight);
      ctx.strokeRect(x, y, boxWidth, boxHeight);
      ctx.fillStyle = "#ffc15b";
      ctx.textAlign = "center";
      ctx.font = `800 ${width < 300 ? 13 : 20}px system-ui, sans-serif`;
      ctx.fillText(message[0], width / 2, y + boxHeight * 0.43);
      ctx.fillStyle = "rgba(232, 245, 242, 0.86)";
      ctx.font = `600 ${width < 300 ? 9 : 12}px system-ui, sans-serif`;
      ctx.fillText(message[1], width / 2, y + boxHeight * 0.68);
    }

    updateText(presentation) {
      const state = this.snapshot;
      const presentationStatus =
        presentation &&
        presentation.blocked &&
        [
          "waiting",
          "stale",
          "fault",
          "sensor_fault",
          "http_lost",
        ].includes(presentation.reason)
          ? presentation.reason
          : null;
      const status = presentationStatus || (this.fetchFailed
        ? "fault"
        : state
          ? state.status
          : "waiting");
      const badge = document.querySelector("#radar-status");
      badge.dataset.status = status;
      badge.textContent =
        STATUS_LABELS[status] || String(status).toUpperCase();
      const warning = document.querySelector("#warning-text");
      if (presentation && presentation.blocked) {
        const detail = presentation.error
          ? ` · ${presentation.error.message}`
          : "";
        warning.textContent = state && state.warning
          ? `${state.warning}${detail}`
          : `레이더 지도 차단 · ${presentation.reason || "unknown"}${detail}`;
      } else {
        warning.textContent =
          presentation && presentation.hazardCopy
            ? presentation.hazardCopy
            : NORMAL_COPY;
      }

      const mode = document.querySelector("#radar-mode");
      mode.textContent = presentation && !presentation.blocked
        ? "CALIBRATED SCENE"
        : "MAP BLOCKED";

      const hazard = presentation && !presentation.blocked
        ? presentation.hazard
        : null;
      const danger = hazard && hazard.level === "DANGER" &&
        presentation.tracks.some((track) =>
          this.isDangerTrack(track, presentation));
      const hazardLevel = presentation && presentation.blocked
        ? "SENSOR_FAULT"
        : danger
          ? "DANGER"
          : hazard && hazard.level === "DANGER"
            ? "UNKNOWN"
            : hazard
              ? hazard.level
              : "UNKNOWN";
      const hazardMetric = document.querySelector("#metric-hazard");
      hazardMetric.dataset.hazard = danger ? "DANGER" : hazardLevel;
      this.setMetric(
        "metric-hazard",
        hazardLevel,
        danger
          ? "확인된 point 장애물 ≤10cm"
          : hazardLevel === "NORMAL"
            ? NORMAL_COPY
            : "지도 입력 확인 필요",
      );

      const nearest = this.nearestConfirmedPoint(presentation);
      const collisionNearest =
        this.nearestConfirmedPoint(presentation, CLOSE_MAX_RANGE_M);
      this.setMetric(
        "metric-nearest",
        nearest === null ? "--" : `${nearest.toFixed(2)} m`,
        "확인된 point track",
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

      const collisionInset = document.querySelector("#collision-inset");
      collisionInset.dataset.hazard = presentation && presentation.blocked
        ? "SENSOR_FAULT"
        : danger
          ? "DANGER"
          : hazardLevel;
      document.querySelector("#collision-distance").textContent =
        collisionNearest === null
          ? "-- cm"
          : `${Math.round(collisionNearest * 100)} cm`;
    }

    updateDiagnostics(presentation) {
      const state = this.snapshot;
      const frame = state && state.frame;
      const scene = state && state.scene;
      const diagnostics = scene && scene.diagnostics;
      const counters = state && state.counters;
      document.querySelector("#axis-value").textContent = state && state.axes
        ? `${state.axes.forward_sign > 0 ? "+" : "-"}${String(state.axes.forward_axis).toUpperCase()} 전방 · ` +
          `${state.axes.lateral_sign > 0 ? "+" : "-"}${String(state.axes.lateral_axis).toUpperCase()} 우측`
        : "+Y 전방 · +X 우측";
      document.querySelector("#profile-value").textContent =
        frame && frame.profile_id ? frame.profile_id : "--";
      document.querySelector("#calibration-value").textContent =
        scene && scene.calibration_status
          ? scene.calibration_status
          : "--";
      const poseMode = presentation && !presentation.blocked
        ? presentation.poseMode
        : null;
      document.querySelector("#pose-mode-value").textContent =
        poseMode === "motion_compensated"
          ? "MOTION COMPENSATED"
          : poseMode === "robot_relative"
            ? "ROBOT RELATIVE"
            : "--";
      document.querySelector("#frame-value").textContent = frame
        ? `${frame.number} · 표시 ${frame.display_point_count}점`
        : "--";
      document.querySelector("#gap-value").textContent = counters
        ? String(
            (counters.frame_gaps_total || 0) +
            (counters.sensor_sequence_gaps_total || 0) +
            (counters.writer_drops_total || 0),
          )
        : "--";
      document.querySelector("#raw-point-value").textContent = frame
        ? String(frame.source_point_count || 0)
        : diagnostics && Number.isFinite(diagnostics.scene_point_count)
          ? String(diagnostics.scene_point_count)
          : "--";
      const tracks = presentation && !presentation.blocked
        ? presentation.tracks
        : [];
      const confirmedCount = tracks.filter(
        (track) =>
          track.source === "point" && track.point_confirmed === true,
      ).length;
      document.querySelector("#confirmed-track-value").textContent =
        String(confirmedCount);
      document.querySelector("#clutter-rejected-value").textContent =
        diagnostics && Number.isFinite(diagnostics.clutter_points_rejected)
          ? String(diagnostics.clutter_points_rejected)
          : "--";
      document.querySelector("#heatmap-rejected-value").textContent =
        diagnostics && Number.isFinite(diagnostics.heatmap_cells_rejected)
          ? String(diagnostics.heatmap_cells_rejected)
          : "-- (API 미제공)";
      document.querySelector("#grid-status-value").textContent =
        presentation && !presentation.blocked && presentation.grid
          ? `OK · ${presentation.grid.length} bytes`
          : "차단/대기";
      document.querySelector("#scene-hazard-value").textContent =
        presentation && !presentation.blocked && presentation.hazard
          ? presentation.hazard.level
          : "SENSOR_FAULT";
      document.querySelector("#axis-warning").hidden =
        Boolean(
          scene &&
          ["ok", "synthetic"].includes(scene.calibration_status),
        );
    }

    updateSectors(presentation) {
      const sectors = Array.from({ length: 5 }, () => ({
        tracks: [],
        danger: false,
      }));
      if (!presentation || presentation.blocked) {
        sectors.forEach((sector, index) => {
          this.setSector(index, "invalid", "차단");
        });
        return;
      }
      presentation.tracks.forEach((track) => {
        const angle =
          (Math.atan2(track.lateral_m, track.forward_m) * 180) / Math.PI;
        if (!Number.isFinite(angle) || angle < -70 || angle > 70) {
          return;
        }
        const index = Math.min(
          4,
          Math.max(0, Math.floor(((angle + 70) / 140) * 5)),
        );
        sectors[index].tracks.push(track);
        sectors[index].danger =
          sectors[index].danger || this.isDangerTrack(track, presentation);
      });
      sectors.forEach((sector, index) => {
        if (sector.tracks.length === 0) {
          this.setSector(index, "unknown", "UNKNOWN");
          return;
        }
        const nearest = Math.min(
          ...sector.tracks.map((track) =>
            Number.isFinite(track.distance_m)
              ? track.distance_m
              : Math.hypot(track.forward_m, track.lateral_m)),
        );
        const hasConfirmedPoint = sector.tracks.some(
          (track) =>
            track.source === "point" && track.point_confirmed === true,
        );
        this.setSector(
          index,
          sector.danger
            ? "danger"
            : hasConfirmedPoint
              ? "point"
              : "evidence",
          `${nearest < 1 ? Math.round(nearest * 100) + "cm" : nearest.toFixed(1) + "m"} 반사`,
        );
      });
    }

    setSector(index, level, label) {
      const element = document.querySelector(`#sector-${index}`);
      element.dataset.level = level;
      element.querySelector("b").textContent = SECTOR_NAMES[index];
      element.querySelector("span").textContent = label;
    }

    nearestConfirmedPoint(presentation, maxDistanceM = Infinity) {
      if (!presentation || presentation.blocked) {
        return null;
      }
      const distances = presentation.tracks
        .filter((track) =>
          track.source === "point" &&
          track.point_confirmed === true &&
          Number.isFinite(track.distance_m) &&
          track.distance_m >= 0 &&
          track.distance_m <= maxDistanceM)
        .map((track) => track.distance_m);
      return distances.length ? Math.min(...distances) : null;
    }

    isDangerTrack(track, scene) {
      return Boolean(
        scene &&
        scene.hazard &&
        scene.hazard.level === "DANGER" &&
        track.source === "point" &&
        track.point_confirmed === true &&
        Number.isFinite(track.distance_m) &&
        Number.isFinite(scene.hazard.threshold_m) &&
        scene.hazard.threshold_m <= DANGER_RANGE_M + 1e-9 &&
        track.distance_m <= scene.hazard.threshold_m,
      );
    }

    setMetric(id, value, detail) {
      const element = document.querySelector(`#${id}`);
      element.querySelector("strong").textContent = value;
      element.querySelector("span").textContent = detail;
    }
  }

  window.HanselRadarPanel = RadarPanel;
})();
