(function () {
  "use strict";

  const UI_BUILD_ID = "20260807-probability-25d-r32";
  const MAIN_MAX_RANGE_M = 0.5;
  const MAIN_HALF_WIDTH_M = 0.5;
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
  const BLOCKED_MESSAGES = {
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
    replay_end: [
      "REPLAY END",
      "마지막 프레임을 지도처럼 사용하지 마세요",
    ],
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

  function blockedMessage(reason) {
    return BLOCKED_MESSAGES[reason] || [
      "MAP BLOCKED",
      String(reason || "unknown input state"),
    ];
  }
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
      this.pointCloud3dCanvas = root.querySelector("#pointcloud-3d-canvas");
      if (!this.mainCanvas || !this.collisionCanvas) {
        throw new Error("radar map canvases are required");
      }
      this.mainContext = this.mainCanvas.getContext("2d");
      this.collisionContext = this.collisionCanvas.getContext("2d");
      this.pointCloud3dContext = this.pointCloud3dCanvas
        ? this.pointCloud3dCanvas.getContext("2d")
        : null;
      this.pointCloud3dYaw = -0.62;
      this.pointCloud3dPitch = 0.38;
      this.pointCloud3dDrag = null;
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
        if (this.pointCloud3dCanvas) {
          this.resizeObserver.observe(this.pointCloud3dCanvas);
        }
      } else {
        window.addEventListener("resize", () => this.draw());
      }
      if (this.rawToggle) {
        this.rawToggle.addEventListener("change", () => this.draw());
      }
      if (this.pointCloud3dCanvas) {
        this.pointCloud3dCanvas.addEventListener("pointerdown", (event) => {
          this.pointCloud3dDrag = {
            x: event.clientX,
            y: event.clientY,
            yaw: this.pointCloud3dYaw,
            pitch: this.pointCloud3dPitch,
          };
          if (this.pointCloud3dCanvas.setPointerCapture) {
            this.pointCloud3dCanvas.setPointerCapture(event.pointerId);
          }
        });
        this.pointCloud3dCanvas.addEventListener("pointermove", (event) => {
          if (!this.pointCloud3dDrag) return;
          this.pointCloud3dYaw = this.pointCloud3dDrag.yaw +
            (event.clientX - this.pointCloud3dDrag.x) * 0.008;
          this.pointCloud3dPitch = Math.max(
            -1.15,
            Math.min(
              1.15,
              this.pointCloud3dDrag.pitch +
                (event.clientY - this.pointCloud3dDrag.y) * 0.008,
            ),
          );
          this.draw();
        });
        const finish3dDrag = () => {
          this.pointCloud3dDrag = null;
        };
        this.pointCloud3dCanvas.addEventListener("pointerup", finish3dDrag);
        this.pointCloud3dCanvas.addEventListener("pointercancel", finish3dDrag);
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
      const pointCloud3d = this.pointCloud3dCanvas
        ? this.resizeCanvas(this.pointCloud3dCanvas)
        : null;
      this.clearCanvas(main.ctx, main.width, main.height);
      this.clearCanvas(collision.ctx, collision.width, collision.height);
      if (pointCloud3d) {
        this.clearCanvas(
          pointCloud3d.ctx,
          pointCloud3d.width,
          pointCloud3d.height,
        );
      }

      const presentation = this.presentation;
      if (!presentation || presentation.blocked) {
        const blockedPresentation = presentation || {
          blocked: true,
          reason: "waiting",
        };
        this.updateDiagnostics(blockedPresentation);
        const pointCloudStatus = document.querySelector("#pointcloud-3d-status");
        if (pointCloudStatus) {
          pointCloudStatus.textContent = "SENSOR FAULT";
        }
        const poseLabel = document.querySelector("#pose-mode-value");
        if (poseLabel) {
          poseLabel.textContent = "--";
        }
        this.drawBlockingOverlay(
          main.ctx,
          main.width,
          main.height,
          blockedPresentation,
        );
        this.drawBlockingOverlay(
          collision.ctx,
          collision.width,
          collision.height,
          blockedPresentation,
        );
        if (pointCloud3d) {
          this.drawBlockingOverlay(
            pointCloud3d.ctx,
            pointCloud3d.width,
            pointCloud3d.height,
            blockedPresentation,
          );
        }
        return;
      }

      const mainTransform = this.makeCircularViewportTransform(
        main.width,
        main.height,
        MAIN_MAX_RANGE_M,
        { top: 50, right: 26, bottom: 64, left: 26 },
      );
      const closeTransform = this.makeViewportTransform(
        collision.width,
        collision.height,
        CLOSE_MAX_RANGE_M,
        CLOSE_MAX_RANGE_M,
        { top: 16, right: 15, bottom: 28, left: 15 },
      );
      const showRaw = Boolean(this.rawToggle && this.rawToggle.checked);
      let rawPoints = [];
      if (showRaw) {
        const frame = this.snapshot && this.snapshot.frame;
        const candidateRawPoints = frame ? frame.points : null;
        rawPoints = Array.isArray(candidateRawPoints) ? candidateRawPoints : [];
      }
      this.drawLidarTopView(
        main.ctx,
        mainTransform,
        presentation,
        rawPoints,
        showRaw,
      );
      this.drawCollisionInset(
        collision.ctx,
        closeTransform,
        presentation,
      );
      if (pointCloud3d) {
        this.drawPointCloud25d(
          pointCloud3d.ctx,
          pointCloud3d.width,
          pointCloud3d.height,
          presentation.filteredTracks,
          rawPoints,
          showRaw,
        );
      }
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

    makeCircularViewportTransform(width, height, radiusM, margin) {
      const mapWidth = Math.max(1, width - margin.left - margin.right);
      const mapHeight = Math.max(1, height - margin.top - margin.bottom);
      return {
        originX: margin.left + mapWidth / 2,
        originY: margin.top + mapHeight / 2,
        scale: Math.min(mapWidth, mapHeight) / (radiusM * 2),
        width,
        height,
        forwardMaxM: radiusM,
        halfWidthM: radiusM,
        centeredCircle: true,
      };
    }

    drawLidarTopView(ctx, transform, scene, rawPoints = [], showRaw = false) {
      this.drawMetricGrid(ctx, transform, {
        forwardMaxM: MAIN_MAX_RANGE_M,
        halfWidthM: MAIN_HALF_WIDTH_M,
        rangeStepM: 0.1,
        labelUnit: "cm",
        clipShape: "circle",
      });
      ctx.save();
      this.clipToMapBoundary(ctx, transform, "circle");
      if (scene.probabilisticLocalMap === null) {
        this.drawEvidenceGrid(ctx, transform, scene, {
          clipShape: "circle",
        });
        this.drawAccumulatedObstacles(ctx, transform, scene);
      } else {
        this.drawProbabilisticLocalMap(
          ctx,
          transform,
          scene.probabilisticLocalMap,
          { clipShape: "circle" },
        );
      }
      this.drawTravelTrail(ctx, transform, scene.travelTrail);
      this.drawPlannedPath(ctx, transform, scene.plannedPath);
      for (const track of Array.isArray(scene.filteredTracks)
        ? scene.filteredTracks : []) {
        this.drawUncertaintyOutline(ctx, transform, track);
      }
      this.drawTracks(ctx, transform, scene.referenceTracks, {
        scene,
        labelLimit: 0,
        maxRangeM: MAIN_MAX_RANGE_M,
        clipShape: "circle",
        reference: true,
      });
      this.drawTracks(ctx, transform, scene.tracks, {
        scene,
        labelLimit: 5,
        maxRangeM: MAIN_MAX_RANGE_M,
        clipShape: "circle",
      });
      if (showRaw) {
        this.drawRawDebugPoints(ctx, transform, rawPoints);
      }
      ctx.restore();
      this.drawRobotFootprint(ctx, transform);
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
      if (scene.probabilisticLocalMap === null) {
        this.drawEvidenceGrid(ctx, transform, scene, {
          clipShape: "radial",
        });
        this.drawAccumulatedObstacles(ctx, transform, scene);
      } else {
        this.drawProbabilisticLocalMap(
          ctx,
          transform,
          scene.probabilisticLocalMap,
          { clipShape: "radial" },
        );
      }
      this.drawPlannedPath(ctx, transform, scene.plannedPath);
      for (const track of Array.isArray(scene.filteredTracks)
        ? scene.filteredTracks : []) {
        this.drawUncertaintyOutline(ctx, transform, track);
      }
      this.drawTracks(ctx, transform, scene.tracks, {
        scene,
        labelLimit: 3,
        maxRangeM: CLOSE_MAX_RANGE_M,
        close: true,
        clipShape: "radial",
      });
      ctx.restore();
      this.drawRobotFootprint(ctx, transform);
      this.drawRobot(ctx, transform);
    }

    drawProbabilisticLocalMap(ctx, transform, localMap, options) {
      const resolution = Number(localMap && localMap.resolutionM);
      const cells = localMap && Array.isArray(localMap.cells)
        ? localMap.cells
        : [];
      if (!Number.isFinite(resolution) || resolution <= 0) {
        ctx.globalAlpha = 1;
        return;
      }
      const size = resolution * transform.scale;
      for (const cell of cells) {
        const forward = Number(cell && cell.forward_m);
        const lateral = Number(cell && cell.lateral_m);
        const probability = Number(cell && cell.probability);
        if (!Number.isFinite(forward) || !Number.isFinite(lateral) ||
            !Number.isFinite(probability)) {
          continue;
        }
        if (options && options.clipShape === "radial" &&
            Math.hypot(forward, lateral) > transform.forwardMaxM + resolution) {
          continue;
        }
        const certainty = Math.max(0, Math.min(1,
          (probability - 0.5) / (0.97 - 0.5)));
        ctx.globalAlpha = certainty * 0.70;
        const color = cell.occupied ? [255, 126, 54] : [92, 228, 208];
        ctx.fillStyle = `rgb(${color[0]}, ${color[1]}, ${color[2]})`;
        const center = window.HanselRadarScene.projectMapPoint(
          transform,
          forward,
          lateral,
        );
        ctx.fillRect(center.x - size / 2, center.y - size / 2, size, size);
      }
      ctx.globalAlpha = 1;
      for (const cell of cells) {
        this.drawUncertaintyOutline(ctx, transform, cell);
      }
    }

    drawUncertaintyOutline(ctx, transform, evidence) {
      const forward = Number(evidence && evidence.forward_m);
      const lateral = Number(evidence && evidence.lateral_m);
      const rangeUncertainty = Number(evidence && evidence.range_uncertainty_m);
      const lateralUncertainty = Number(
        evidence && evidence.lateral_uncertainty_m,
      );
      if (!Number.isFinite(forward) || !Number.isFinite(lateral) ||
          !Number.isFinite(rangeUncertainty) || rangeUncertainty < 0 ||
          !Number.isFinite(lateralUncertainty) || lateralUncertainty < 0) {
        return;
      }
      const point = window.HanselRadarScene.projectMapPoint(
        transform,
        forward,
        lateral,
      );
      ctx.save();
      ctx.shadowBlur = 0;
      ctx.strokeStyle = "rgba(223, 252, 255, 0.35)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.ellipse(
        point.x,
        point.y,
        lateralUncertainty * transform.scale,
        rangeUncertainty * transform.scale,
        Math.atan2(lateral, forward),
        0,
        Math.PI * 2,
      );
      ctx.stroke();
      ctx.restore();
    }

    drawRobotFootprint(ctx, transform) {
      ctx.save();
      ctx.shadowBlur = 0;
      for (const [radiusM, color, dash] of [
        [0.105, "rgba(255, 184, 77, 0.42)", [4, 4]],
        [0.075, "rgba(223, 252, 255, 0.66)", []],
      ]) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 1;
        ctx.setLineDash(dash);
        ctx.beginPath();
        ctx.arc(
          transform.originX,
          transform.originY,
          radiusM * transform.scale,
          0,
          Math.PI * 2,
        );
        ctx.stroke();
      }
      ctx.setLineDash([]);
      ctx.restore();
    }

    drawPointCloud25d(ctx, width, height, filteredTracks, rawPoints, showRaw) {
      const rangeM = 0.50;
      const azimuthLimit = Math.PI / 3;
      const tracks = (Array.isArray(filteredTracks) ? filteredTracks : [])
        .filter((track) => {
          const forward = Number(track && track.forward_m);
          const lateral = Number(track && track.lateral_m);
          const distance = Number.isFinite(track && track.distance_m)
            ? track.distance_m
            : Math.hypot(forward, lateral);
          return Number.isFinite(forward) && forward > 0 &&
            Number.isFinite(lateral) && Number.isFinite(distance) &&
            distance >= 0 && distance <= rangeM &&
            Math.abs(Math.atan2(lateral, forward)) <= azimuthLimit;
        })
        .sort((left, right) => left.distance_m - right.distance_m);
      const debugPoints = showRaw === true
        ? (Array.isArray(rawPoints) ? rawPoints : [])
          .filter((point) => Array.isArray(point) && point.length >= 2)
          .map((point) => ({
            forward: Number(point[0]),
            lateral: Number(point[1]),
            height: point[2] === null ? 0 : Number(point[2]),
          }))
          .filter((point) => Number.isFinite(point.forward) &&
            point.forward > 0 && Number.isFinite(point.lateral) &&
            Number.isFinite(point.height) &&
            Math.hypot(point.forward, point.lateral) <= rangeM &&
            Math.abs(Math.atan2(point.lateral, point.forward)) <= azimuthLimit)
        : [];
      const yaw = this.pointCloud3dYaw;
      const pitch = this.pointCloud3dPitch;
      const cosineYaw = Math.cos(yaw);
      const sineYaw = Math.sin(yaw);
      const cosinePitch = Math.cos(pitch);
      const sinePitch = Math.sin(pitch);
      const baseScale = Math.min(width, height) * 0.72 / (rangeM * 2);
      const centerX = width * 0.50;
      const centerY = height * 0.62;
      const project = (lateral, vertical, forward) => {
        const yawX = cosineYaw * lateral - sineYaw * forward;
        const yawDepth = sineYaw * lateral + cosineYaw * forward;
        const viewY = cosinePitch * vertical - sinePitch * yawDepth;
        const depth = sinePitch * vertical + cosinePitch * yawDepth;
        const perspective = 1 / Math.max(0.62, 1.18 + depth * 0.7);
        return {
          x: centerX + yawX * baseScale * perspective,
          y: centerY - viewY * baseScale * perspective,
          depth,
          perspective,
        };
      };
      const line3d = (left, right, color, widthPx = 1) => {
        const a = project(left[0], left[1], left[2]);
        const b = project(right[0], right[1], right[2]);
        ctx.strokeStyle = color;
        ctx.lineWidth = widthPx;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      };

      const drawShell = (distance) => {
        ctx.strokeStyle = "rgba(73, 168, 255, 0.20)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let step = 0; step <= 24; step += 1) {
          const azimuth = -azimuthLimit + (azimuthLimit * 2 * step) / 24;
          const projected = project(
            Math.sin(azimuth) * distance,
            0,
            Math.cos(azimuth) * distance,
          );
          if (step === 0) ctx.moveTo(projected.x, projected.y);
          else ctx.lineTo(projected.x, projected.y);
        }
        ctx.stroke();
        const labelPoint = project(0, 0, distance);
        ctx.fillStyle = "rgba(137, 205, 255, 0.74)";
        ctx.fillText(`${Math.round(distance * 100)}cm`, labelPoint.x + 4, labelPoint.y);
      };

      ctx.save();
      ctx.shadowBlur = 0;
      ctx.lineCap = "round";
      ctx.font = "700 9px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "left";
      for (const azimuth of [-azimuthLimit, 0, azimuthLimit]) {
        line3d(
          [0, 0, 0],
          [Math.sin(azimuth) * rangeM, 0, Math.cos(azimuth) * rangeM],
          "rgba(73, 168, 255, 0.18)",
        );
      }
      for (const elevation of [-40, 40]) {
        const radians = elevation * Math.PI / 180;
        line3d(
          [0, 0, 0],
          [0, Math.sin(radians) * rangeM, Math.cos(radians) * rangeM],
          "rgba(92, 228, 208, 0.16)",
        );
      }
      for (const shell of [0.10, 0.30, 0.50]) {
        drawShell(shell);
      }

      const bandColor = (track) => track.height_band === "below_sensor"
        ? "#5bbcff"
        : track.height_band === "above_sensor"
          ? "#ffb84d"
          : "#dffcff";
      tracks.forEach((track, index) => {
        const isHeatmap = track.source === "heatmap";
        const height = isHeatmap ? 0 : Number(track.height_m);
        const projected = project(track.lateral_m, height, track.forward_m);
        const markerRadius = 4.5;
        ctx.shadowBlur = 0;
        if (isHeatmap) {
          ctx.fillStyle = "#a98cff";
          ctx.strokeStyle = "#a98cff";
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(projected.x, projected.y - markerRadius);
          ctx.lineTo(projected.x + markerRadius, projected.y);
          ctx.lineTo(projected.x, projected.y + markerRadius);
          ctx.lineTo(projected.x - markerRadius, projected.y);
          ctx.lineTo(projected.x, projected.y - markerRadius);
          ctx.fill();
          ctx.stroke();
        } else {
          const color = bandColor(track);
          ctx.fillStyle = color;
          ctx.strokeStyle = color;
          ctx.lineWidth = track.point_confirmed === true ? 1.5 : 1;
          ctx.beginPath();
          ctx.arc(projected.x, projected.y, markerRadius, 0, Math.PI * 2);
          if (track.point_confirmed === true) ctx.fill();
          ctx.stroke();
          ctx.beginPath();
          ctx.moveTo(projected.x - markerRadius - 2, projected.y);
          ctx.lineTo(projected.x + markerRadius + 2, projected.y);
          ctx.moveTo(projected.x, projected.y - markerRadius - 2);
          ctx.lineTo(projected.x, projected.y + markerRadius + 2);
          ctx.stroke();
        }
        if (index < 5) {
          const distance = Number.isFinite(track.distance_m)
            ? track.distance_m
            : Math.hypot(track.forward_m, track.lateral_m);
          const zText = isHeatmap
            ? "Z UNKNOWN"
            : `Z ${height >= 0 ? "+" : ""}${height.toFixed(2)}m`;
          ctx.fillStyle = isHeatmap ? "#c7b7ff" : bandColor(track);
          ctx.fillText(
            `${distance.toFixed(2)}m · ${zText}`,
            projected.x + 7,
            projected.y - 6,
          );
        }
      });

      for (const point of debugPoints) {
        const projected = project(point.lateral, point.height, point.forward);
        ctx.strokeStyle = "rgba(170, 180, 180, 0.55)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(projected.x - 2, projected.y - 2);
        ctx.lineTo(projected.x + 2, projected.y + 2);
        ctx.moveTo(projected.x + 2, projected.y - 2);
        ctx.lineTo(projected.x - 2, projected.y + 2);
        ctx.stroke();
      }

      ctx.fillStyle = "rgba(220, 242, 255, 0.84)";
      ctx.fillText(
        showRaw === true
          ? `FILTERED ${tracks.length} · RAW DEBUG ${debugPoints.length}`
          : `FILTERED ${tracks.length} · RAW OFF · Z SENSOR-RELATIVE`,
        8,
        13,
      );
      ctx.fillStyle = "#ffc15b";
      ctx.fillText("±40° ELEVATION · DRAG TO ROTATE", 8, 27);
      ctx.fillStyle = "#5ce4d0";
      ctx.fillText("±60° AZIMUTH", 8, 40);
      ctx.restore();

      const status = document.querySelector("#pointcloud-3d-status");
      if (status) {
        status.textContent = `${tracks.length} FILTERED · SENSOR-RELATIVE Z`;
      }
    }

    drawAccumulatedObstacles(ctx, transform, scene) {
      const obstacles = scene && Array.isArray(scene.rollingMapObstacles)
        ? scene.rollingMapObstacles
        : [];
      const windowMs = Number.isFinite(scene && scene.obstacleMemoryWindowMs)
        ? scene.obstacleMemoryWindowMs
        : 5000;
      ctx.save();
      for (const obstacle of obstacles) {
        if (Math.hypot(obstacle.forward_m, obstacle.lateral_m) >
            transform.forwardMaxM ||
            (!transform.centeredCircle && obstacle.forward_m < 0)) {
          continue;
        }
        const point = window.HanselRadarScene.projectMapPoint(
          transform,
          obstacle.forward_m,
          obstacle.lateral_m,
        );
        const life = Number.isFinite(obstacle.remaining_ms)
          ? Math.max(0, Math.min(1, obstacle.remaining_ms / windowMs))
          : 1;
        const strength = Math.max(0, Math.min(1, obstacle.confidence));
        const hitTrust = Math.min(1, Math.max(0.35, obstacle.hits / 5));
        const alpha = Math.max(0.16, life * strength * hitTrust * 0.88);
        const radius = 5 + strength * 5;
        ctx.fillStyle = `rgba(255, 126, 54, ${alpha * 0.28})`;
        ctx.strokeStyle = `rgba(255, 173, 72, ${alpha})`;
        ctx.lineWidth = 2.2;
        ctx.beginPath();
        ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = `rgba(255, 231, 174, ${alpha})`;
        ctx.fillRect(point.x - 2, point.y - 2, 4, 4);
      }
      ctx.restore();
    }

    drawTravelTrail(ctx, transform, travelTrail) {
      const points = travelTrail && Array.isArray(travelTrail.points)
        ? travelTrail.points
        : [];
      if (points.length < 2) {
        return;
      }
      const visible = points.filter((point) =>
        Math.hypot(point[0], point[1]) <= transform.forwardMaxM,
      );
      if (visible.length < 2) {
        return;
      }
      ctx.save();
      ctx.strokeStyle = "rgba(73, 168, 255, 0.92)";
      ctx.fillStyle = "rgba(137, 205, 255, 0.96)";
      ctx.lineWidth = 2.4;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.setLineDash([6, 5]);
      ctx.beginPath();
      visible.forEach((coordinates, index) => {
        const point = window.HanselRadarScene.projectMapPoint(
          transform,
          coordinates[0],
          coordinates[1],
        );
        if (index === 0) {
          ctx.moveTo(point.x, point.y);
        } else {
          ctx.lineTo(point.x, point.y);
        }
      });
      ctx.stroke();
      ctx.setLineDash([]);
      const start = window.HanselRadarScene.projectMapPoint(
        transform,
        visible[0][0],
        visible[0][1],
      );
      ctx.beginPath();
      ctx.arc(start.x, start.y, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }

    drawPlannedPath(ctx, transform, plannedPath) {
      if (!plannedPath || !Array.isArray(plannedPath.points)) {
        return;
      }
      const strokePath = (points, color, width) => {
        if (!Array.isArray(points) || points.length < 2) return;
        ctx.beginPath();
        points.forEach((coordinates, index) => {
          const point = window.HanselRadarScene.projectMapPoint(
            transform,
            coordinates[0],
            coordinates[1],
          );
          if (index === 0) {
            ctx.moveTo(point.x, point.y);
          } else {
            ctx.lineTo(point.x, point.y);
          }
        });
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.stroke();
      };
      ctx.save();
      const passageWidthM = Number(plannedPath.minimum_passage_width_m);
      const underlayWidth = Number.isFinite(passageWidthM) && passageWidthM > 0
        ? passageWidthM * transform.scale
        : 7;
      strokePath(
        plannedPath.points,
        "rgba(5, 9, 12, 0.42)",
        underlayWidth,
      );
      strokePath(plannedPath.points, "rgba(255, 215, 64, 0.96)", 3.2);
      const blockedPrevious = plannedPath.blocked_previous_path;
      if (blockedPrevious && Array.isArray(blockedPrevious.points) &&
          Array.isArray(blockedPrevious.segments)) {
        for (const segment of blockedPrevious.segments) {
          const blockedPoints = blockedPrevious.points.slice(
            segment.start_index,
            segment.end_index + 1,
          );
          const blockedColor = "rgba(255, 81, 81, 0.92)";
          if (blockedPoints.length === 1) {
            const blockedPoint = window.HanselRadarScene.projectMapPoint(
              transform,
              blockedPoints[0][0],
              blockedPoints[0][1],
            );
            ctx.fillStyle = blockedColor;
            ctx.beginPath();
            ctx.arc(
              blockedPoint.x,
              blockedPoint.y,
              3.2,
              0,
              Math.PI * 2,
            );
            ctx.fill();
          } else {
            strokePath(blockedPoints, blockedColor, 3.2);
          }
        }
      }
      if (plannedPath.points.length > 0) {
        const end = plannedPath.points[plannedPath.points.length - 1];
        const endpoint = window.HanselRadarScene.projectMapPoint(
          transform,
          end[0],
          end[1],
        );
        ctx.fillStyle = "#ffe47a";
        ctx.beginPath();
        ctx.arc(endpoint.x, endpoint.y, 4, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
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
        this.drawRangeGuideArc(
          ctx,
          transform,
          rangeM,
          options.clipShape,
        );
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
        ctx.lineTo(
          x,
          options.clipShape === "circle"
            ? transform.originY + forwardMaxM * transform.scale
            : transform.originY,
        );
        ctx.stroke();
      }
      ctx.setLineDash([]);
      ctx.strokeStyle = "rgba(92, 228, 208, 0.42)";
      ctx.beginPath();
      ctx.moveTo(transform.originX, top);
      ctx.lineTo(
        transform.originX,
        options.clipShape === "circle"
          ? transform.originY + forwardMaxM * transform.scale
          : transform.originY,
      );
      ctx.stroke();
      ctx.fillStyle = "rgba(114, 150, 148, 0.48)";
      ctx.font = "700 9px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "left";
      ctx.fillText("UNKNOWN", left + 7, top + 14);
      ctx.restore();
      this.strokeMapBoundary(ctx, transform, options.clipShape);
    }

    drawRangeGuideArc(ctx, transform, rangeM, clipShape) {
      ctx.beginPath();
      ctx.arc(
        transform.originX,
        transform.originY,
        rangeM * transform.scale,
        clipShape === "circle" ? 0 : Math.PI,
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
      if (clipShape === "circle") {
        ctx.arc(
          transform.originX,
          transform.originY,
          transform.forwardMaxM * transform.scale,
          0,
          Math.PI * 2,
        );
      } else if (clipShape === "radial") {
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
      if (clipShape === "circle") {
        ctx.arc(
          transform.originX,
          transform.originY,
          transform.forwardMaxM * transform.scale,
          0,
          Math.PI * 2,
        );
      } else if (clipShape === "radial") {
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
        const visualAlpha = this.trackVisualAlpha(track) *
          (options.reference ? 0.28 : 1.0);
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
            options.reference === true,
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
            transform,
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
      reference = false,
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
        Math.min(Math.PI / 36, uncertainty / Math.max(distance, 0.05)),
      );
      const radius = distance * transform.scale;
      const canvasStart = angle - angularHalfWidth - Math.PI / 2;
      const canvasEnd = angle + angularHalfWidth - Math.PI / 2;
      ctx.save();
      ctx.globalAlpha = visualAlpha;
      ctx.strokeStyle = danger
        ? "#ff5151"
        : reference
          ? "rgba(255, 193, 91, 0.72)"
        : "rgba(92, 228, 208, 0.82)";
      ctx.lineWidth = reference
        ? 1.5
        : Math.max(2, Math.min(8, uncertainty * transform.scale));
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
        : reference
          ? "rgba(255, 213, 132, 0.62)"
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
      transform,
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
      const sourceText = track.source === "point" ? "P" : "H";
      const text = `${sourceText} ${distanceText}`;
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

    drawRawDebugPoints(ctx, transform, rawPoints) {
      if (!Array.isArray(rawPoints)) {
        return;
      }
      ctx.save();
      ctx.fillStyle = "rgba(90, 184, 255, 0.48)";
      for (const rawPoint of rawPoints) {
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
      const message = blockedMessage(reason);
      ctx.fillStyle = "#020609";
      ctx.fillRect(0, 0, width, height);
      const boxWidth = Math.max(180, Math.min(480, width - 28));
      const boxHeight = Math.min(116, Math.max(86, height - 24));
      const x = (width - boxWidth) / 2;
      const y = (height - boxHeight) / 2;
      ctx.fillStyle = "rgba(8, 17, 21, 0.98)";
      ctx.strokeStyle = "rgba(255, 81, 81, 0.92)";
      ctx.lineWidth = 2;
      ctx.fillRect(x, y, boxWidth, boxHeight);
      ctx.strokeRect(x, y, boxWidth, boxHeight);
      ctx.fillStyle = "#ff5151";
      ctx.textAlign = "center";
      ctx.font = `800 ${width < 300 ? 13 : 20}px system-ui, sans-serif`;
      ctx.fillText(message[0], width / 2, y + boxHeight * 0.43);
      ctx.fillStyle = "rgba(232, 245, 242, 0.86)";
      ctx.font = `600 ${width < 300 ? 9 : 12}px system-ui, sans-serif`;
      ctx.fillText(message[1], width / 2, y + boxHeight * 0.68);
    }

    updateText(presentation) {
      const state = this.snapshot;
      const blocked = Boolean(presentation && presentation.blocked);
      const presentationStatus =
        blocked &&
        [
          "waiting",
          "stale",
          "fault",
          "sensor_fault",
          "http_lost",
        ].includes(presentation.reason)
          ? presentation.reason
          : blocked
            ? "sensor_fault"
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
      const warningStrip = document.querySelector("#warning-strip");
      if (warningStrip) {
        warningStrip.dataset.blocked = blocked ? "true" : "false";
      }
      const warning = document.querySelector("#warning-text");
      if (presentation && presentation.blocked) {
        const message = blockedMessage(presentation.reason);
        const detail = presentation.error
          ? ` · ${presentation.error.message}`
          : "";
        warning.textContent = `${message[0]} · ${message[1]}${detail}`;
      } else {
        warning.textContent =
          presentation && presentation.hazardCopy
            ? presentation.hazardCopy
            : NORMAL_COPY;
      }
      const safetyFooter = document.querySelector("#safety-footer-text");
      if (safetyFooter) {
        safetyFooter.textContent = warning.textContent;
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
        !blocked && state && Number.isFinite(state.fps)
          ? state.fps.toFixed(1)
          : "--",
        "frame/s",
      );
      this.setMetric(
        "metric-age",
        !blocked && state && Number.isFinite(state.age_ms)
          ? String(state.age_ms)
          : "--",
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
      const visible = Boolean(presentation && !presentation.blocked);
      const frame = visible && state && state.frame;
      const scene = visible && state && state.scene;
      const diagnostics = visible && presentation && presentation.diagnostics;
      const counters = visible && state && state.counters;
      const diagnosticValue = (key) => diagnostics &&
        Number.isInteger(diagnostics[key]) && diagnostics[key] >= 0
        ? String(diagnostics[key])
        : "--";
      const setDiagnosticValue = (selector, key) => {
        const element = document.querySelector(selector);
        if (element) element.textContent = diagnosticValue(key);
      };
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
      const poseMode = visible
        ? presentation.poseMode
        : null;
      document.querySelector("#pose-mode-value").textContent =
        poseMode === "motion_compensated"
          ? "MOTION COMPENSATED"
          : poseMode === "robot_relative_short_memory"
            ? "ROBOT RELATIVE · SHORT MEMORY"
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
      setDiagnosticValue("#raw-point-value", "raw_point_count");
      setDiagnosticValue(
        "#clutter-rejected-value",
        "self_clutter_rejected_count",
      );
      setDiagnosticValue("#reflection-value", "heatmap_reflection_count");
      const pointConfirmed = diagnostics &&
        diagnostics.confirmed_point_track_count;
      const heatmapConfirmed = diagnostics &&
        diagnostics.confirmed_heatmap_track_count;
      document.querySelector("#confirmed-track-value").textContent =
        Number.isInteger(pointConfirmed) && pointConfirmed >= 0 &&
        Number.isInteger(heatmapConfirmed) && heatmapConfirmed >= 0
          ? String(pointConfirmed + heatmapConfirmed)
          : "--";
      for (const [selector, key] of [
        ["#quality-rejected-value", "quality_gate_rejected_count"],
        ["#probability-active-value", "probability_active_cell_count"],
        ["#probability-occupied-value", "probability_occupied_cell_count"],
        ["#probability-decayed-value", "probability_decayed_cell_count"],
        ["#probability-expired-value", "probability_ttl_expired_cell_count"],
        ["#path-pending-value", "path_switch_pending_count"],
        ["#path-accepted-value", "path_switch_accepted_count"],
      ]) {
        setDiagnosticValue(selector, key);
      }
      const heatmapCandidateElement = document.querySelector(
        "#heatmap-candidate-value",
      );
      if (heatmapCandidateElement) {
        heatmapCandidateElement.textContent = diagnosticValue(
          "heatmap_candidate_cells",
        );
      }
      const heatmapClusterElement = document.querySelector(
        "#heatmap-cluster-value",
      );
      if (heatmapClusterElement) {
        heatmapClusterElement.textContent = diagnosticValue(
          "heatmap_cluster_count",
        );
      }
      const memoryElement = document.querySelector("#obstacle-memory-value");
      if (memoryElement) {
        const count = diagnosticValue("rolling_local_map_cell_count");
        memoryElement.textContent = count === "--" ? "--" : `${count} · IN 50cm`;
      }
      const stableCellElement = document.querySelector("#stable-cell-value");
      if (stableCellElement) {
        const count = diagnosticValue("stable_occupancy_cell_count");
        stableCellElement.textContent = count === "--" ? "--" : `${count} · 5cm GRID`;
      }
      document.querySelector("#grid-status-value").textContent =
        presentation && !presentation.blocked && presentation.grid
          ? `OK · ${presentation.grid.length} bytes`
          : "차단/대기";
      document.querySelector("#scene-hazard-value").textContent =
        presentation && !presentation.blocked && presentation.hazard
          ? presentation.hazard.level
          : "SENSOR_FAULT";
      const motionElement = document.querySelector("#motion-status-value");
      if (motionElement) {
        const motion = visible && presentation.motionCompensation;
        motionElement.textContent = motion
          ? `${String(motion.status).toUpperCase()} · ${motion.matched_points || 0} match`
          : "--";
      }
      const pathElement = document.querySelector("#path-status-value");
      if (pathElement) {
        const path = visible && presentation.plannedPath;
        pathElement.textContent = path
          ? `${Math.round(path.length_m * 100)}cm · ${String(
              path.status || "unknown",
            ).toUpperCase()} · ${String(
              path.stability_status || "stable",
            ).toUpperCase()}`
          : "--";
      }
      const pathConfidenceElement = document.querySelector(
        "#path-confidence-value",
      );
      if (pathConfidenceElement) {
        const path = visible && presentation.plannedPath;
        pathConfidenceElement.textContent =
          path && Number.isFinite(path.confidence)
            ? `${Math.round(path.confidence * 100)}% · ${String(
                path.confidence_level || "unknown",
              ).toUpperCase()} · GAP ${Math.round(
                Math.max(0, Number(path.minimum_clearance_m) || 0) * 100,
              )}cm`
            : "--";
      }
      document.querySelector("#axis-warning").hidden =
        Boolean(
          scene &&
          ["ok", "synthetic"].includes(scene.calibration_status),
        );
      const pointCloudStatus = document.querySelector("#pointcloud-3d-status");
      if (pointCloudStatus && !visible) {
        pointCloudStatus.textContent = "SENSOR FAULT";
      }
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
