(function () {
  "use strict";

  const UI_BUILD_ID = "20260826-turn-confidence-r75";
  const MAIN_MAX_RANGE_M = 0.3;
  const MAIN_HALF_WIDTH_M = 0.3;
  const CLOSE_MAX_RANGE_M = 0.3;
  const OPERATOR_EVIDENCE_RADIUS_M = 0.3;
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
      this.options = Object.assign({
        endpoint: "/api/radar",
        now: () => Date.now(),
      }, options || {});
      this.snapshot = null;
      this.presentation = {
        blocked: true,
        reason: "waiting",
      };
      this.fetchFailed = false;
      this.lastError = null;
      this.lastValidPresentation = null;
      this.lastValidSnapshot = null;
      this.lastValidAcceptedAtMs = null;
      this.invalidSceneCount = 0;

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
          this.clearTransientSceneHold();
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
        await new Promise((resolve) => window.setTimeout(resolve, 200));
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
      this.fetchFailed = false;
      this.lastError = null;
      try {
        const parsed = window.HanselRadarScene.parseRadarScene(snapshot);
        this.presentation = parsed;
        this.snapshot = snapshot;
        if (parsed && !parsed.blocked) {
          this.lastValidPresentation = parsed;
          this.lastValidSnapshot = snapshot;
          this.lastValidAcceptedAtMs = this.options.now();
          this.invalidSceneCount = 0;
        } else {
          this.clearTransientSceneHold();
        }
      } catch (error) {
        this.invalidSceneCount += 1;
        try {
          const fallback = window.HanselRadarScene.parseRadarEvidenceFallback(
            snapshot,
            error,
          );
          this.snapshot = snapshot;
          this.presentation = {
            ...fallback,
            sceneVerification: null,
          };
        } catch (_fallbackError) {
          const nowMs = this.options.now();
          const ageMs = this.lastValidAcceptedAtMs == null
            ? Infinity : Math.max(0, nowMs - this.lastValidAcceptedAtMs);
          if (this.lastValidPresentation && ageMs <= 2000) {
            this.snapshot = this.lastValidSnapshot;
            this.presentation = {
              ...this.lastValidPresentation,
              evidenceOnly: true,
              sceneContractFallback: true,
              hazard: { level: "UNKNOWN", threshold_m: DANGER_RANGE_M },
              plannedPath: {
                status: "unavailable", points: [], length_m: 0,
              },
              sceneVerification: null,
            };
          } else {
            this.snapshot = snapshot;
            this.presentation = {
              blocked: true,
              reason: "invalid_scene",
              error,
            };
          }
        }
      }
      this.updateText(this.presentation);
      this.updateDiagnostics(this.presentation);
      this.updateSectors(this.presentation);
      this.draw();
    }

    clearTransientSceneHold() {
      this.lastValidPresentation = null;
      this.lastValidSnapshot = null;
      this.lastValidAcceptedAtMs = null;
      this.invalidSceneCount = 0;
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
      this.clearCanvas(main.ctx, main.width, main.height);

      const presentation = this.presentation;
      if (!presentation || presentation.blocked) {
        const blockedPresentation = presentation || {
          blocked: true,
          reason: "waiting",
        };
        this.updateDiagnostics(blockedPresentation);
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
        return;
      }

      const mainTransform = this.makeViewportTransform(
        main.width,
        main.height,
        MAIN_MAX_RANGE_M,
        MAIN_HALF_WIDTH_M,
        { top: 48, right: 30, bottom: 58, left: 30 },
      );
      const showRaw = Boolean(this.rawToggle && this.rawToggle.checked);
      const frame = this.snapshot && this.snapshot.frame;
      const candidateRawPoints = showRaw && frame ? frame.points : null;
      const rawPoints = Array.isArray(candidateRawPoints)
        ? candidateRawPoints : [];
      this.drawCollisionInset(
        main.ctx,
        mainTransform,
        presentation,
        rawPoints,
        showRaw,
      );
    }

    clearCanvas(ctx, width, height) {
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#151c19";
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
      this.drawReflectivitySurface(
        ctx,
        transform,
        scene.reflectivitySurface,
        { clipShape: "circle" },
      );
      this.drawAdcAngleEvidence(
        ctx,
        transform,
        scene.adcRangeAngleSurface,
      );
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
      for (const track of Array.isArray(scene.filteredTracks)
        ? scene.filteredTracks : []) {
        if (this.operatorVisibleTrack(track)) {
          this.drawUncertaintyOutline(ctx, transform, track);
        }
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
      // Keep the steering advisory above every reflection/track layer.  It is
      // the operator's final overlay and must never disappear under evidence.
      this.drawPlannedPath(ctx, transform, scene.plannedPath);
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

    drawCollisionInset(ctx, transform, scene, rawPoints = [], showRaw = false) {
      this.drawMetricGrid(ctx, transform, {
        forwardMaxM: CLOSE_MAX_RANGE_M,
        halfWidthM: CLOSE_MAX_RANGE_M,
        rangeStepM: 0.1,
        labelUnit: "cm",
        clipShape: "fov",
      });
      ctx.save();
      this.clipToMapBoundary(ctx, transform, "fov");
      this.drawReflectivitySurface(
        ctx,
        transform,
        scene.reflectivitySurface,
        { clipShape: "fov" },
      );
      if (scene.adcRangeAngleSurface) {
        this.drawAdcAngleEvidence(
          ctx,
          transform,
          scene.adcRangeAngleSurface,
        );
      } else {
        this.drawAdcRangeEvidence(
          ctx,
          transform,
          this.snapshot?.frame?.adc_range_profile,
        );
      }
      if (scene.probabilisticLocalMap === null) {
        this.drawEvidenceGrid(ctx, transform, scene, {
          clipShape: "fov",
        });
        this.drawAccumulatedObstacles(ctx, transform, scene);
      } else {
        this.drawProbabilisticLocalMap(
          ctx,
          transform,
          scene.probabilisticLocalMap,
          { clipShape: "fov", showUncertainty: false },
        );
      }
      this.drawTracks(ctx, transform, scene.tracks, {
        scene,
        labelLimit: 3,
        maxRangeM: CLOSE_MAX_RANGE_M,
        close: true,
        clipShape: "fov",
      });
      this.drawOperatorReturns(
        ctx,
        transform,
        scene.probabilisticLocalMap,
      );
      this.drawPlannedPath(ctx, transform, scene.plannedPath);
      if (showRaw) {
        this.drawRawDebugPoints(ctx, transform, rawPoints);
      }
      ctx.restore();
      this.drawOperatorGuide(
        ctx,
        transform,
        this.deriveOperatorGuide(
          scene.probabilisticLocalMap,
          scene.plannedPath,
        ),
        scene.hazard,
        scene.plannedPath,
      );
      this.drawRobotFootprint(ctx, transform);
      this.drawRobot(ctx, transform);
    }

    drawAdcRangeEvidence(ctx, transform, profile) {
      if (!profile || profile.source !== "uart_last_chirp_pair_adc" ||
          profile.direction_semantics !== "range_only" ||
          profile.complete_frame_adc !== false ||
          !Array.isArray(profile.values)) {
        return;
      }
      const rangeStepM = Number(profile.range_step_m);
      if (!Number.isFinite(rangeStepM) || rangeStepM <= 0) return;
      ctx.save();
      ctx.setLineDash([3, 5]);
      ctx.lineWidth = 1.25;
      for (let index = 1; index < profile.values.length; index += 1) {
        const rangeM = index * rangeStepM;
        const value = Number(profile.values[index]);
        if (rangeM > transform.forwardMaxM + 1e-9 ||
            !Number.isFinite(value) || value <= 0) {
          continue;
        }
        const strength = Math.max(0, Math.min(1, value / 255));
        ctx.globalAlpha = 0.05 + 0.22 * strength;
        ctx.strokeStyle = "rgba(118, 184, 255, 0.88)";
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
      const peakRangeM = Number(profile.peak_range_m);
      if (Number.isFinite(peakRangeM) && peakRangeM > 0 &&
          peakRangeM <= transform.forwardMaxM + 1e-9) {
        ctx.globalAlpha = 0.86;
        ctx.fillStyle = "rgba(166, 210, 255, 0.94)";
        ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "left";
        ctx.fillText(
          `ADC RANGE ONLY · ${Math.round(peakRangeM * 100)}cm PEAK`,
          14,
          26,
        );
      }
      ctx.globalAlpha = 1;
      ctx.restore();
    }

    drawAdcAngleEvidence(ctx, transform, surface) {
      if (!surface || surface.source !== "uart_last_chirp_pair_adc" ||
          surface.directionSemantics !== "range_azimuth" ||
          surface.completeFrameAdc !== false ||
          !Array.isArray(surface.cells) || !Array.isArray(surface.peaks)) {
        return;
      }
      ctx.save();
      for (const cell of surface.cells) {
        const rangeM = Number(cell.range_m);
        const forwardM = Number(cell.forward_m);
        const lateralM = Number(cell.lateral_m);
        const intensity = Number(cell.intensity);
        const coherence = Number(cell.coherence);
        if (!Number.isFinite(rangeM) || rangeM <= 0 ||
            rangeM > transform.forwardMaxM + 1e-9 ||
            !Number.isFinite(forwardM) || !Number.isFinite(lateralM) ||
            !Number.isFinite(intensity) || !Number.isFinite(coherence) ||
            intensity <= 0) {
          continue;
        }
        const point = window.HanselRadarScene.projectMapPoint(
          transform, forwardM, lateralM,
        );
        const strength = Math.max(0, Math.min(1, intensity * coherence));
        const radius = Math.max(0.7, Math.min(
          2.3, 0.45 + 1.85 * strength,
        ));
        ctx.globalAlpha = 0.025 + 0.28 * strength;
        ctx.fillStyle = "rgba(99, 231, 255, 0.92)";
        ctx.beginPath();
        ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
        ctx.fill();
      }
      const verified = surface.peaks.filter((peak) =>
        peak && peak.verified === true &&
        Number.isFinite(Number(peak.range_m)) &&
        Number(peak.range_m) <= transform.forwardMaxM + 1e-9 &&
        Number.isFinite(Number(peak.azimuth_deg)) &&
        Number.isFinite(Number(peak.forward_m)) &&
        Number.isFinite(Number(peak.lateral_m)));
      for (const peak of verified) {
        const rangeM = Number(peak.range_m);
        const azimuthRad = Number(peak.azimuth_deg) * Math.PI / 180;
        const uncertaintyRad = Number(peak.angular_uncertainty_deg) *
          Math.PI / 180;
        const point = window.HanselRadarScene.projectMapPoint(
          transform, Number(peak.forward_m), Number(peak.lateral_m),
        );
        ctx.globalAlpha = 0.45 + 0.5 * Math.max(
          0, Math.min(1, Number(peak.confidence) || 0),
        );
        ctx.strokeStyle = "rgba(176, 250, 255, 0.96)";
        ctx.lineWidth = 2;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.arc(
          transform.originX,
          transform.originY,
          rangeM * transform.scale,
          -Math.PI / 2 + azimuthRad - uncertaintyRad,
          -Math.PI / 2 + azimuthRad + uncertaintyRad,
        );
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "rgba(225, 254, 255, 0.98)";
        ctx.beginPath();
        ctx.arc(point.x, point.y, 3.4, 0, Math.PI * 2);
        ctx.fill();
      }
      if (verified.length > 0) {
        const strongest = verified.reduce((best, peak) =>
          Number(peak.confidence) > Number(best.confidence) ? peak : best);
        const sign = Number(strongest.azimuth_deg) > 0 ? "+" : "";
        const calibration = surface.phaseCalibration === "measured"
          ? "CAL" : "DESIGN CAL";
        ctx.globalAlpha = 0.94;
        ctx.fillStyle = "rgba(195, 251, 255, 0.98)";
        ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "left";
        ctx.fillText(
          `ADC PHASE ${calibration} · ${sign}${Math.round(
            Number(strongest.azimuth_deg),
          )}° · ${Math.round(Number(strongest.range_m) * 100)}cm`,
          14,
          26,
        );
      }
      ctx.globalAlpha = 1;
      ctx.restore();
    }

    deriveOperatorWalls(surface, localMap = null) {
      const probabilityCells = localMap && Array.isArray(localMap.cells)
        ? localMap.cells : [];
      const hasProbabilityBoundary = probabilityCells.some(
        (cell) => Number(cell && cell.probability) > 0.5,
      );
      const calibratedPolar = surface && surface.source ===
        "full_range_azimuth_heatmap" && surface.calibrated === true &&
        surface.normalization ===
          "current_range_common_mode_physical_beam" &&
        Array.isArray(surface.cells);
      if (calibratedPolar && !hasProbabilityBoundary) {
        const physicalResolutionDeg = Number(surface.angularResolutionDeg);
        const beamWidthDeg = Number.isFinite(physicalResolutionDeg) &&
          physicalResolutionDeg > 0 ? physicalResolutionDeg : 29;
        const beamCenters = [-48, -24, 0, 24, 48];
        const points = [];
        for (const centreDeg of beamCenters) {
          if (Math.abs(centreDeg) < 1e-9) continue;
          const candidates = [];
          for (const cell of surface.cells) {
            const azimuthDeg = Number(cell && cell.azimuth_deg);
            const rangeM = Number(cell && cell.range_m);
            const forwardM = Number(cell && cell.forward_m);
            const lateralM = Number(cell && cell.lateral_m);
            const residualDb = Number(
              cell && cell.calibrated_residual_db,
            );
            const confidence = Number(cell && cell.confidence);
            const persistence = Number(cell && cell.persistence);
            if (!Number.isFinite(azimuthDeg) ||
                !Number.isFinite(rangeM) || rangeM < 0.07 ||
                rangeM > OPERATOR_EVIDENCE_RADIUS_M + 1e-9 ||
                !Number.isFinite(forwardM) || forwardM < 0 ||
                forwardM > CLOSE_MAX_RANGE_M + 1e-9 ||
                !Number.isFinite(lateralM) ||
                Math.abs(lateralM) > MAIN_HALF_WIDTH_M + 1e-9 ||
                !Number.isFinite(residualDb) ||
                !Number.isFinite(confidence) ||
                !Number.isFinite(persistence)) continue;
            const deltaDeg = (azimuthDeg - centreDeg) /
              (beamWidthDeg / 2.355);
            const angularWeight = Math.exp(-0.5 * deltaDeg * deltaDeg);
            if (angularWeight < 0.03) continue;
            candidates.push({
              cell,
              rangeM,
              residualDb,
              confidence,
              persistence,
              angularWeight,
            });
          }
          const ranges = [...new Set(candidates.map((value) => value.rangeM))]
            .sort((left, right) => left - right);
          let best = null;
          let visibility = 1;
          for (const rangeM of ranges) {
            const row = candidates.filter((value) => value.rangeM === rangeM);
            const weight = row.reduce(
              (sum, value) => sum + value.angularWeight, 0,
            );
            if (weight <= 0) continue;
            const residualDb = row.reduce(
              (sum, value) => sum + value.residualDb * value.angularWeight,
              0,
            ) / weight;
            const confidence = row.reduce(
              (sum, value) => sum + value.confidence * value.angularWeight,
              0,
            ) / weight;
            const persistence = row.reduce(
              (sum, value) => sum + value.persistence * value.angularWeight,
              0,
            ) / weight;
            // Residual is converted continuously into a posterior.  No cell
            // is dropped for weak signal; the largest positive posterior is
            // the beam's measured boundary and weak beams stay translucent.
            const occupancy = 1 / (1 + Math.exp(
              -Math.max(-12, Math.min(12, residualDb / 1.5)),
            ));
            const sceneChange = row.reduce(
              (sum, value) => sum + Number(
                value.cell.scene_change_score ?? 1,
              ) * value.angularWeight,
              0,
            ) / weight;
            const strength = Math.max(0, Math.min(0.995,
              (occupancy - 0.5) * 2 * confidence * persistence,
            )) * (0.55 + 0.45 * Math.max(0, Math.min(1, sceneChange)));
            const firstSurfaceProbability = visibility * strength;
            if (!best || firstSurfaceProbability > best.strength) {
              best = {
                rangeM, residualDb,
                strength: firstSurfaceProbability, confidence,
              };
            }
            visibility *= 1 - strength;
          }
          if (!best || best.strength <= 0) continue;
          const azimuthRad = centreDeg * Math.PI / 180;
          const forwardM = best.rangeM * Math.cos(azimuthRad);
          const lateralM = best.rangeM * Math.sin(azimuthRad);
          if (forwardM > CLOSE_MAX_RANGE_M + 1e-9 ||
              Math.abs(lateralM) > MAIN_HALF_WIDTH_M + 1e-9) continue;
          points.push({
            azimuthIndex: centreDeg,
            azimuthDeg: centreDeg,
            rangeM: best.rangeM,
            forwardM,
            lateralM,
            strength: best.strength,
            confidence: best.confidence,
            angularUncertaintyDeg: beamWidthDeg / 2,
          });
        }
        return {
          left: points.filter((point) => point.lateralM < 0),
          right: points.filter((point) => point.lateralM > 0),
          calibrated: true,
        };
      }
      if (probabilityCells.length > 0) {
        const resolutionM = Number(localMap.resolutionM);
        const rows = new Map();
        for (const cell of probabilityCells) {
          const forwardM = Number(cell && cell.forward_m);
          const lateralM = Number(cell && cell.lateral_m);
          const probability = Number(cell && cell.probability);
          if (!Number.isFinite(forwardM) || forwardM <= 0 ||
              forwardM > CLOSE_MAX_RANGE_M + 1e-9 ||
              !Number.isFinite(lateralM) || Math.abs(lateralM) < 1e-6 ||
              Math.abs(lateralM) > MAIN_HALF_WIDTH_M + 1e-9 ||
              !Number.isFinite(probability) || probability <= 0.5) {
            continue;
          }
          const side = lateralM < 0 ? "left" : "right";
          const rowIndex = Number.isFinite(resolutionM) && resolutionM > 0
            ? Math.round(forwardM / resolutionM)
            : Math.round(forwardM * 100);
          const key = `${side}:${rowIndex}`;
          const strength = Math.max(0, Math.min(1,
            (probability - 0.5) * 2,
          ));
          const point = {
            azimuthIndex: rowIndex,
            azimuthDeg: Math.atan2(lateralM, forwardM) * 180 / Math.PI,
            rangeM: Math.hypot(forwardM, lateralM),
            forwardM,
            lateralM,
            strength,
            confidence: strength,
          };
          const previous = rows.get(key);
          if (!previous || point.strength > previous.strength ||
              (point.strength === previous.strength &&
                Math.abs(point.lateralM) < Math.abs(previous.lateralM))) {
            rows.set(key, point);
          }
        }
        const left = [];
        const right = [];
        for (const [key, point] of rows) {
          (key.startsWith("left:") ? left : right).push(point);
        }
        left.sort((a, b) => a.forwardM - b.forwardM);
        right.sort((a, b) => a.forwardM - b.forwardM);
        if (left.length + right.length >= 1) {
          return { left, right };
        }
      }
      const cells = surface && surface.source ===
        "full_range_azimuth_heatmap" && Array.isArray(surface.cells)
        ? surface.cells : [];
      const columns = new Map();
      for (const cell of cells) {
        const azimuthIndex = Number(cell && cell.azimuth_index);
        const azimuthDeg = Number(cell && cell.azimuth_deg);
        const rangeM = Number(cell && cell.range_m);
        const forwardM = Number(cell && cell.forward_m);
        const lateralM = Number(cell && cell.lateral_m);
        const intensity = Number(cell && cell.intensity);
        const confidence = Number(cell && cell.confidence);
        const persistence = Number(cell && cell.persistence);
        if (!Number.isInteger(azimuthIndex) ||
            !Number.isFinite(azimuthDeg) ||
            !Number.isFinite(rangeM) || rangeM <= 0 ||
            rangeM > OPERATOR_EVIDENCE_RADIUS_M + 1e-9 ||
            !Number.isFinite(forwardM) || forwardM < 0 ||
            forwardM > CLOSE_MAX_RANGE_M + 1e-9 ||
            !Number.isFinite(lateralM) ||
            Math.abs(lateralM) > MAIN_HALF_WIDTH_M + 1e-9 ||
            !Number.isFinite(intensity) ||
            !Number.isFinite(confidence) ||
            !Number.isFinite(persistence)) {
          continue;
        }
        const strength = Math.max(0, Math.min(1, intensity)) *
          (0.65 + 0.35 * Math.max(0, Math.min(1, confidence))) *
          (0.50 + 0.50 * Math.max(0, Math.min(1, persistence)));
        const point = {
          azimuthIndex,
          azimuthDeg,
          rangeM,
          forwardM,
          lateralM,
          strength,
          confidence: Math.max(0, Math.min(1, confidence)),
        };
        const previous = columns.get(azimuthIndex);
        if (!previous || point.strength > previous.strength ||
            (point.strength === previous.strength &&
              point.rangeM < previous.rangeM)) {
          columns.set(azimuthIndex, point);
        }
      }
      const ordered = [...columns.values()].sort(
        (left, right) => left.azimuthDeg - right.azimuthDeg,
      );
      return {
        left: ordered.filter((point) => point.lateralM < -1e-6),
        right: ordered.filter((point) => point.lateralM > 1e-6),
      };
    }

    operatorWallSegments(points) {
      const segments = [];
      let segment = [];
      for (const point of Array.isArray(points) ? points : []) {
        const previous = segment.at(-1);
        const gap = previous ? Math.hypot(
          point.forwardM - previous.forwardM,
          point.lateralM - previous.lateralM,
        ) : 0;
        const angularGapDeg = previous
          ? Math.abs(Number(point.azimuthDeg) - Number(previous.azimuthDeg))
          : 0;
        const radialGapM = previous
          ? Math.abs(Number(point.rangeM) - Number(previous.rangeM))
          : 0;
        const physicallyAdjacent = previous &&
          Number.isFinite(angularGapDeg) && angularGapDeg <= 29.0 + 1e-9 &&
          Number.isFinite(radialGapM) && radialGapM <= 0.09 + 1e-9;
        if (previous && gap > 0.12 && !physicallyAdjacent) {
          if (segment.length > 0) segments.push(segment);
          segment = [];
        }
        segment.push(point);
      }
      if (segment.length > 0) segments.push(segment);
      return segments;
    }

    drawOperatorReturns(ctx, transform, localMap = null) {
      const cells = localMap && Array.isArray(localMap.cells)
        ? localMap.cells : [];
      ctx.save();
      for (const cell of cells) {
        const forwardM = Number(cell && cell.forward_m);
        const lateralM = Number(cell && cell.lateral_m);
        const probability = Number(cell && cell.probability);
        const radialUncertainty = Number(cell && cell.range_uncertainty_m);
        const lateralUncertainty = Number(cell && cell.lateral_uncertainty_m);
        if (!Number.isFinite(forwardM) || forwardM < 0 ||
            forwardM > transform.forwardMaxM + 1e-9 ||
            !Number.isFinite(lateralM) ||
            Math.abs(lateralM) > transform.halfWidthM + 1e-9 ||
            !Number.isFinite(probability) || probability <= 0.5 ||
            Math.abs(Math.atan2(lateralM, Math.max(forwardM, 1e-9))) >
              Math.PI / 3 + 1e-9) {
          continue;
        }
        const projected = window.HanselRadarScene.projectMapPoint(
          transform, forwardM, lateralM,
        );
        const strength = Math.max(0, Math.min(1,
          (probability - 0.5) / 0.47,
        ));
        const radiusPx = Math.max(
          4,
          Math.min(28, Math.max(
            Number.isFinite(radialUncertainty) ? radialUncertainty : 0,
            Number.isFinite(lateralUncertainty) ? lateralUncertainty : 0,
            0.025,
          ) * transform.scale),
        );
        ctx.beginPath();
        ctx.arc(projected.x, projected.y, radiusPx, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(92, 228, 208, ${
          (0.08 + 0.34 * strength).toFixed(3)})`;
        ctx.fill();
        ctx.beginPath();
        ctx.arc(projected.x, projected.y, 2.4 + 2.6 * strength,
          0, Math.PI * 2);
        ctx.fillStyle = `rgba(224, 255, 252, ${
          (0.50 + 0.48 * strength).toFixed(3)})`;
        ctx.fill();
      }
      ctx.restore();
    }

    deriveOperatorGuide(localMap, plannedPath = null) {
      if (plannedPath && ["blocked_near_field", "stopped_no_path"].includes(plannedPath.status)) {
        return {points: [], direction: "unknown", advisory: true,
          evidenceState: "blocked", passageVerified: false, confidence: 0};
      }
      const evidence = [];
      const mapCells = localMap && Array.isArray(localMap.cells)
        ? localMap.cells : [];
      for (const cell of mapCells) {
        const forwardM = Number(cell && cell.forward_m);
        const lateralM = Number(cell && cell.lateral_m);
        const probability = Number(cell && cell.probability);
        const rangeUncertainty = Number(cell && cell.range_uncertainty_m);
        const lateralUncertainty = Number(cell && cell.lateral_uncertainty_m);
        if (!Number.isFinite(forwardM) || forwardM < 0 || forwardM > 0.30 ||
            !Number.isFinite(lateralM) ||
            !Number.isFinite(probability) || probability <= 0.5) continue;
        if (Math.abs(lateralM) > 0.30 ||
            Math.abs(Math.atan2(lateralM, Math.max(1e-9, forwardM))) >
              Math.PI / 3 + 1e-9) continue;
        evidence.push({
          forwardM,
          lateralM,
          strength: Math.max(0, Math.min(1, (probability - 0.5) * 2)),
          uncertaintyM: Math.max(
            0,
            Number.isFinite(rangeUncertainty) ? rangeUncertainty : 0,
            Number.isFinite(lateralUncertainty) ? lateralUncertainty : 0,
          ),
        });
      }
      const occupiedEvidence = evidence.filter(
        (obstacle) => obstacle.strength > 0,
      );
      const negativeLateral = occupiedEvidence
        .filter((obstacle) => obstacle.lateralM < -1e-6)
        .map((obstacle) => obstacle.lateralM);
      const positiveLateral = occupiedEvidence
        .filter((obstacle) => obstacle.lateralM > 1e-6)
        .map((obstacle) => obstacle.lateralM);
      const leftBoundaryM = negativeLateral.length > 0
        ? Math.max(...negativeLateral) : null;
      const rightBoundaryM = positiveLateral.length > 0
        ? Math.min(...positiveLateral) : null;
      const twoSided = leftBoundaryM !== null && rightBoundaryM !== null;
      const corridorCenterM = twoSided
        ? (leftBoundaryM + rightBoundaryM) / 2 : null;
      const corridorWidthM = twoSided
        ? rightBoundaryM - leftBoundaryM : null;
      const requiredPassageWidthM = 0.21;
      const boundedPassage = twoSided &&
        corridorWidthM >= requiredPassageWidthM;
      const evidenceState = boundedPassage ? "two_sided" :
        (twoSided ? "too_narrow" :
          (occupiedEvidence.length > 0 ? "one_sided" : "unobserved"));

      if (plannedPath && plannedPath.status === "turn_side_unresolved") {
        return {
          points: [],
          direction: "unknown",
          advisory: true,
          clearanceM: requiredPassageWidthM,
          evidenceState: "side_unresolved",
          passageVerified: false,
          corridorWidthM,
          confidence: 0,
        };
      }

      if (plannedPath && Array.isArray(plannedPath.points) &&
          plannedPath.points.length >= 2 &&
          ["clear", "limited"].includes(plannedPath.status)) {
        const points = plannedPath.points.map((point) => [point[0], point[1]]);
        const destination = points.at(-1)[1];
        return {
          points,
          direction: destination > 0.025 ? "right" :
            (destination < -0.025 ? "left" : "straight"),
          advisory: true,
          clearanceM: Number.isFinite(plannedPath.minimum_passage_width_m)
            ? plannedPath.minimum_passage_width_m : requiredPassageWidthM,
          evidenceState,
          passageVerified: boundedPassage,
          corridorWidthM,
          confidence: boundedPassage
            ? Math.max(0, Math.min(
                1, (corridorWidthM - requiredPassageWidthM) / 0.09,
              ))
            : 0,
        };
      }

      if (!boundedPassage) {
        return {
          points: [],
          direction: "unknown",
          advisory: true,
          clearanceM: requiredPassageWidthM,
          evidenceState,
          passageVerified: false,
          corridorWidthM,
          confidence: 0,
        };
      }

      const lateralValues = [];
      for (let index = -18; index <= 18; index += 1) {
        lateralValues.push(index / 100);
      }
      const rows = [];
      let previous = [{ lateralM: 0, cost: 0, previousIndex: -1 }];
      const robotInfluenceM = 0.075 + 0.03;
      for (let step = 1; step <= 10; step += 1) {
        const forwardM = step * 0.03;
        const limitM = Math.min(0.18, 0.02 + forwardM * 0.80);
        const candidates = lateralValues.filter(
          (lateralM) => Math.abs(lateralM) <= limitM + 1e-9,
        );
        const row = candidates.map((lateralM) => {
          let obstacleCost = 0;
          for (const obstacle of evidence) {
            if (obstacle.strength <= 0) continue;
            const influenceM = robotInfluenceM +
              Math.min(0.05, obstacle.uncertaintyM);
            const distanceM = Math.hypot(
              forwardM - obstacle.forwardM,
              lateralM - obstacle.lateralM,
            );
            const ratio = distanceM / Math.max(0.02, influenceM);
            obstacleCost += obstacle.strength *
              Math.exp(-0.5 * ratio * ratio) * 9.0;
          }
          let bestCost = Infinity;
          let bestIndex = -1;
          previous.forEach((prior, priorIndex) => {
            const deltaM = lateralM - prior.lateralM;
            if (Math.abs(deltaM) > 0.055 + 1e-9) return;
            const transitionCost = 1.8 * Math.pow(deltaM / 0.03, 2);
            const preferredLateralM = corridorCenterM === null
              ? 0 : corridorCenterM;
            const centreCost = 0.045 * Math.pow(
              (lateralM - preferredLateralM) / 0.01,
              2,
            );
            const cost = prior.cost + obstacleCost +
              transitionCost + centreCost;
            if (cost < bestCost) {
              bestCost = cost;
              bestIndex = priorIndex;
            }
          });
          return { lateralM, cost: bestCost, previousIndex: bestIndex };
        });
        rows.push(row);
        previous = row;
      }
      let bestIndex = 0;
      for (let index = 1; index < previous.length; index += 1) {
        if (previous[index].cost < previous[bestIndex].cost) {
          bestIndex = index;
        }
      }
      const points = [];
      for (let rowIndex = rows.length - 1; rowIndex >= 0; rowIndex -= 1) {
        const node = rows[rowIndex][bestIndex];
        points.push([(rowIndex + 1) * 0.03, node.lateralM]);
        bestIndex = node.previousIndex;
      }
      points.reverse();
      points.unshift([0, 0]);
      const destination = points.at(-1)[1];
      return {
        points,
        direction: destination > 0.025 ? "right" :
          (destination < -0.025 ? "left" : "straight"),
        advisory: true,
        clearanceM: robotInfluenceM * 2,
        evidenceState: "two_sided",
        passageVerified: true,
        corridorWidthM,
        confidence: Math.max(0, Math.min(
          1, (corridorWidthM - requiredPassageWidthM) / 0.09,
        )),
      };
    }

    drawOperatorGuide(ctx, transform, guide, hazard, plannedPath = null) {
      const points = guide && Array.isArray(guide.points)
        ? guide.points : [];
      if (points.length < 2) {
        const evidenceState = guide && guide.evidenceState;
        const reason = evidenceState === "blocked"
          ? "STOP · NO VERIFIED PASSAGE"
          : evidenceState === "side_unresolved"
          ? "TURN REQUIRED · SIDE UNRESOLVED"
          : evidenceState === "too_narrow"
          ? "PASSAGE TOO NARROW · DIRECTION WITHHELD"
          : evidenceState === "one_sided"
            ? "ONE-SIDED EVIDENCE · DIRECTION WITHHELD"
            : "COVERAGE INCOMPLETE · DIRECTION WITHHELD";
        ctx.save();
        ctx.fillStyle = "rgba(255, 213, 107, 0.94)";
        ctx.font = "800 11px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "center";
        const lines = reason.split(" · ");
        lines.forEach((line, index) => ctx.fillText(line, transform.width / 2, 30 + index * 15));
        ctx.restore();
        return;
      }
      const projected = points.map(([forwardM, lateralM]) =>
        window.HanselRadarScene.projectMapPoint(
          transform,
          forwardM,
          lateralM,
        ));
      const trace = () => {
        ctx.beginPath();
        ctx.moveTo(projected[0].x, projected[0].y);
        for (const point of projected.slice(1)) {
          ctx.lineTo(point.x, point.y);
        }
      };
      ctx.save();
      ctx.globalAlpha = 1;
      ctx.globalCompositeOperation = "source-over";
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      trace();
      ctx.strokeStyle = "rgba(255, 206, 74, 0.07)";
      ctx.lineWidth = Math.max(18, guide.clearanceM * transform.scale);
      ctx.stroke();
      trace();
      ctx.strokeStyle = "rgba(255, 197, 46, 0.18)";
      ctx.lineWidth = 12;
      ctx.stroke();
      trace();
      ctx.strokeStyle = "rgba(255, 225, 101, 0.98)";
      ctx.lineWidth = 3.5;
      ctx.stroke();
      const endpoint = projected.at(-1);
      ctx.beginPath();
      ctx.arc(endpoint.x, endpoint.y, 6, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(255, 225, 101, 0.98)";
      ctx.fill();
      ctx.fillStyle = "rgba(255, 235, 156, 0.98)";
      ctx.font = "800 12px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      const backendBlocked = plannedPath &&
        (plannedPath.status === "blocked_near_field" ||
          plannedPath.status === "stopped_no_path");
      let direction;
      if (backendBlocked) {
        direction = "STOP · NO VERIFIED PASSAGE";
      } else if (guide.passageVerified === false) {
        const heading = guide.direction === "left" ? "LEFT" :
          (guide.direction === "right" ? "RIGHT" : "STRAIGHT");
        direction = guide.evidenceState === "too_narrow"
          ? `GUIDE ${heading} · WIDTH UNVERIFIED`
          : `GUIDE ${heading} · UNKNOWN ≠ FREE`;
      } else {
        direction = guide.direction === "left" ? "STEER LEFT" :
          (guide.direction === "right" ? "STEER RIGHT" : "GO STRAIGHT");
      }
      const labelPoint = projected[Math.min(7, projected.length - 1)];
      const lines = ctx.measureText(direction).width > transform.width - 40
        ? direction.split(" · ") : [direction];
      lines.forEach((line, index) => {
        const half = ctx.measureText(line).width / 2 + 10;
        ctx.fillText(line, Math.max(half, Math.min(transform.width - half, labelPoint.x)),
          Math.max(70, labelPoint.y - 13 - (lines.length - 1 - index) * 15));
      });
      if (hazard && hazard.level === "DANGER") {
        ctx.fillStyle = "rgba(255, 76, 80, 0.98)";
        ctx.fillText("STOP · OBSTACLE WITHIN 10cm", transform.originX, 28);
      }
      ctx.restore();
    }

    drawReflectivitySurface(ctx, transform, surface, options) {
      if (surface && surface.source === "full_range_azimuth_heatmap") {
        this.drawFullHeatmapSurface(ctx, transform, surface, options);
        return;
      }
      const resolution = Number(surface && surface.resolutionM);
      const cells = surface && Array.isArray(surface.cells)
        ? surface.cells : [];
      if (!Number.isFinite(resolution) || resolution <= 0 || cells.length === 0) {
        ctx.globalAlpha = 1;
        return;
      }
      const size = resolution * transform.scale;
      const keys = new Set(cells.map((cell) => [
        Math.round(Number(cell.forward_m) / resolution),
        Math.round(Number(cell.lateral_m) / resolution),
      ].join(":")));
      for (const cell of cells) {
        const forward = Number(cell && cell.forward_m);
        const lateral = Number(cell && cell.lateral_m);
        const intensity = Number(cell && cell.intensity);
        const persistence = Number(cell && cell.persistence);
        if (!Number.isFinite(forward) || !Number.isFinite(lateral) ||
            !Number.isFinite(intensity) || !Number.isFinite(persistence)) {
          continue;
        }
        if (options && options.clipShape === "radial" &&
            Math.hypot(forward, lateral) > transform.forwardMaxM + resolution) {
          continue;
        }
        const forwardIndex = Math.round(forward / resolution);
        const lateralIndex = Math.round(lateral / resolution);
        const point = window.HanselRadarScene.projectMapPoint(
          transform,
          forward,
          lateral,
        );
        const left = point.x - size / 2;
        const right = point.x + size / 2;
        const top = point.y - size / 2;
        const bottom = point.y + size / 2;
        ctx.shadowBlur = 0;
        ctx.fillStyle = "rgb(224, 232, 231)";
        ctx.globalAlpha = 0.08 + Math.max(0, Math.min(1,
          0.55 * intensity + 0.45 * persistence)) * 0.52;
        ctx.fillRect(left, top, size, size);
        ctx.globalAlpha = 0.72;
        ctx.strokeStyle = "rgba(245, 250, 248, 0.78)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        if (!keys.has(`${forwardIndex + 1}:${lateralIndex}`)) {
          ctx.moveTo(left, top);
          ctx.lineTo(right, top);
        }
        if (!keys.has(`${forwardIndex - 1}:${lateralIndex}`)) {
          ctx.moveTo(left, bottom);
          ctx.lineTo(right, bottom);
        }
        if (!keys.has(`${forwardIndex}:${lateralIndex - 1}`)) {
          ctx.moveTo(left, top);
          ctx.lineTo(left, bottom);
        }
        if (!keys.has(`${forwardIndex}:${lateralIndex + 1}`)) {
          ctx.moveTo(right, top);
          ctx.lineTo(right, bottom);
        }
        ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }

    drawFullHeatmapSurface(ctx, transform, surface, options) {
      const cells = Array.isArray(surface && surface.cells)
        ? surface.cells : [];
      if (cells.length === 0) return;
      ctx.save();
      this.clipToMapBoundary(
        ctx,
        transform,
        options && options.clipShape ? options.clipShape : "circle",
      );
      const physicalResolutionDeg = Number(surface.angularResolutionDeg);
      const beamWidthDeg = Number.isFinite(physicalResolutionDeg) &&
        physicalResolutionDeg > 0 ? physicalResolutionDeg : 29;
      const beamCenters = [-48, -24, 0, 24, 48];
      const rangeRows = [...new Set(cells.map((cell) =>
        Number(cell && cell.range_m),
      ).filter(Number.isFinite))];
      const displayCells = [];
      for (const rangeM of rangeRows) {
        const rowCells = cells.filter((cell) =>
          Math.abs(Number(cell && cell.range_m) - rangeM) < 1e-9,
        );
        for (const centreDeg of beamCenters) {
          let weightedResidual = 0;
          let weightedConfidence = 0;
          let weightedPersistence = 0;
          let weightSum = 0;
          let rangeUncertainty = 0;
          for (const cell of rowCells) {
            const azimuthDeg = Number(cell && cell.azimuth_deg);
            const residualDb = Number(cell && cell.relative_db);
            const confidence = Number(cell && cell.confidence);
            const persistence = Number(cell && cell.persistence);
            if (!Number.isFinite(azimuthDeg) ||
                !Number.isFinite(residualDb) ||
                !Number.isFinite(confidence) ||
                !Number.isFinite(persistence)) continue;
            const sigmaDeg = beamWidthDeg / 2.355;
            const delta = (azimuthDeg - centreDeg) / sigmaDeg;
            const weight = Math.exp(-0.5 * delta * delta);
            weightedResidual += residualDb * weight;
            weightedConfidence += confidence * weight;
            weightedPersistence += persistence * weight;
            weightSum += weight;
            rangeUncertainty = Math.max(
              rangeUncertainty,
              Number(cell && cell.range_uncertainty_m) || 0,
            );
          }
          if (weightSum <= 0 || rangeUncertainty <= 0) continue;
          const relativeDb = weightedResidual / weightSum;
          displayCells.push({
            range_m: rangeM,
            azimuth_deg: centreDeg,
            intensity: 1 / (1 + Math.exp(
              -Math.max(-12, Math.min(12, relativeDb / 1.5)),
            )),
            confidence: weightedConfidence / weightSum,
            persistence: weightedPersistence / weightSum,
            relative_db: relativeDb,
            range_uncertainty_m: rangeUncertainty,
            angular_uncertainty_deg: beamWidthDeg / 2,
          });
        }
      }
      for (const cell of displayCells) {
        const rangeM = Number(cell && cell.range_m);
        const azimuthDeg = Number(cell && cell.azimuth_deg);
        const intensity = Number(cell && cell.intensity);
        const confidence = Number(cell && cell.confidence);
        const rangeUncertainty = Number(cell && cell.range_uncertainty_m);
        const angularUncertainty = Number(
          cell && cell.angular_uncertainty_deg,
        );
        if (!Number.isFinite(rangeM) || rangeM <= 0 ||
            !Number.isFinite(azimuthDeg) || !Number.isFinite(intensity) ||
            !Number.isFinite(confidence) ||
            !Number.isFinite(rangeUncertainty) || rangeUncertainty <= 0 ||
            !Number.isFinite(angularUncertainty) || angularUncertainty <= 0) {
          continue;
        }
        const centre = azimuthDeg * Math.PI / 180 - Math.PI / 2;
        const halfWidth = angularUncertainty * Math.PI / 180;
        const innerRadius = Math.max(
          0,
          (rangeM - rangeUncertainty) * transform.scale,
        );
        const outerRadius = Math.max(
          innerRadius + 1,
          (rangeM + rangeUncertainty) * transform.scale,
        );
        const strength = Math.max(0, Math.min(1, intensity));
        const trust = Math.max(0, Math.min(1, confidence));
        const visibleStrength = Math.max(0, Math.min(1,
          (strength - 0.34) / 0.66,
        ));
        // Full heatmap energy is diagnostic context, not a wall.  Keep it
        // visible without letting a bright later multipath lobe obscure the
        // first-surface probability map and steering overlay.
        const alpha = 0.004 + Math.pow(visibleStrength, 2.2) *
          (0.025 + 0.045 * trust);
        const shade = Math.round(76 + visibleStrength * 150);
        ctx.globalAlpha = alpha;
        ctx.fillStyle = `rgb(${shade}, ${shade}, ${shade})`;
        ctx.beginPath();
        ctx.arc(
          transform.originX,
          transform.originY,
          outerRadius,
          centre - halfWidth,
          centre + halfWidth,
        );
        ctx.arc(
          transform.originX,
          transform.originY,
          innerRadius,
          centre + halfWidth,
          centre - halfWidth,
          true,
        );
        ctx.closePath();
        ctx.fill();
      }
      ctx.globalAlpha = 1;
      ctx.restore();
    }

    drawReflectivityContours(ctx, transform, contours) {
      const segments = contours && contours.status === "confirmed" &&
        Array.isArray(contours.segments) ? contours.segments : [];
      if (segments.length === 0) return;
      ctx.save();
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.shadowBlur = 0;
      for (const segment of segments) {
        // Prominence is local (same bearing, neighbouring ranges), so it is a
        // useful display weight without becoming a scene-specific detection
        // cutoff.  Keep weak measured fragments visible, while making the
        // temporally persistent, spatially supported wall ridges dominant.
        const prominenceWeight = Math.max(
          0,
          Math.min(1, segment.meanProminenceDb / 2.0),
        );
        const spanWeight = Math.max(0, Math.min(1, segment.spanM / 0.10));
        const evidenceWeight = Math.max(
          0.18,
          Math.min(
            1,
            0.45 * segment.confidence +
              0.35 * prominenceWeight +
              0.20 * spanWeight,
          ),
        );
        const start = window.HanselRadarScene.projectMapPoint(
          transform, segment.start[0], segment.start[1],
        );
        const end = window.HanselRadarScene.projectMapPoint(
          transform, segment.end[0], segment.end[1],
        );
        const trace = (color, width) => {
          ctx.beginPath();
          ctx.moveTo(start.x, start.y);
          ctx.lineTo(end.x, end.y);
          ctx.strokeStyle = color;
          ctx.lineWidth = width;
          ctx.stroke();
        };
        trace(`rgba(80, 235, 224, ${0.07 + 0.25 * evidenceWeight})`,
          6 + 8 * evidenceWeight);
        trace(`rgba(190, 255, 250, ${0.28 + 0.70 * evidenceWeight})`,
          1.4 + 3.1 * evidenceWeight);
        for (const point of [start, end]) {
          ctx.beginPath();
          ctx.arc(point.x, point.y, 2.0 + 1.8 * evidenceWeight,
            0, Math.PI * 2);
          ctx.fillStyle = `rgba(224, 255, 252, ${
            0.35 + 0.63 * evidenceWeight})`;
          ctx.fill();
        }
        const midpoint = {
          x: (start.x + end.x) / 2,
          y: (start.y + end.y) / 2,
        };
        const nearestM = Math.min(
          Math.hypot(...segment.start),
          Math.hypot(...segment.end),
        );
        ctx.fillStyle = `rgba(220, 255, 252, ${
          0.30 + 0.66 * evidenceWeight})`;
        ctx.font = "800 10px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "center";
        ctx.fillText(
          `${Math.round(nearestM * 100)}cm · ${Math.round(segment.headingDeg)}°`,
          midpoint.x,
          midpoint.y - 8,
        );
      }
      ctx.restore();
    }

    drawReflectivityCorridor(ctx, transform, corridor) {
      const walls = corridor && Array.isArray(corridor.walls)
        ? corridor.walls : [];
      if (!corridor || corridor.status === "unavailable" || walls.length !== 2) {
        return;
      }
      const projectedWalls = walls.map((wall) => ({
        start: window.HanselRadarScene.projectMapPoint(
          transform,
          wall.start[0],
          wall.start[1],
        ),
        end: window.HanselRadarScene.projectMapPoint(
          transform,
          wall.end[0],
          wall.end[1],
        ),
      }));
      const candidate = corridor.status === "candidate";
      ctx.save();

      ctx.fillStyle = "rgba(92, 228, 208, 0.08)";
      ctx.beginPath();
      ctx.moveTo(projectedWalls[0].start.x, projectedWalls[0].start.y);
      ctx.lineTo(projectedWalls[0].end.x, projectedWalls[0].end.y);
      ctx.lineTo(projectedWalls[1].end.x, projectedWalls[1].end.y);
      ctx.lineTo(projectedWalls[1].start.x, projectedWalls[1].start.y);
      ctx.closePath();
      ctx.fill();

      ctx.setLineDash(candidate ? [7, 5] : []);
      const strokeWalls = (color, width) => {
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineCap = "round";
        for (const wall of projectedWalls) {
          ctx.beginPath();
          ctx.moveTo(wall.start.x, wall.start.y);
          ctx.lineTo(wall.end.x, wall.end.y);
          ctx.stroke();
        }
      };
      strokeWalls("rgba(1, 8, 12, 0.88)", 5.4);
      strokeWalls("rgba(177, 247, 242, 0.96)", 2.4);
      ctx.setLineDash([]);

      const labelX = (
        projectedWalls[0].start.x + projectedWalls[0].end.x +
        projectedWalls[1].start.x + projectedWalls[1].end.x
      ) / 4;
      const labelY = (
        projectedWalls[0].start.y + projectedWalls[0].end.y +
        projectedWalls[1].start.y + projectedWalls[1].end.y
      ) / 4;
      const widthM = Number(corridor.widthM);
      const widthLabel = Number.isFinite(widthM)
        ? `${Math.round(widthM * 100)}cm`
        : "--cm";
      const geometryLabel = corridor.geometry === "converging"
        ? `BOTTLENECK ${widthLabel}` : widthLabel;
      const narrowLabel = Number.isFinite(widthM) && widthM < 0.21
        ? " · NARROW" : "";
      ctx.fillStyle = "rgba(205, 255, 249, 0.96)";
      ctx.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "bottom";
      ctx.fillText(
        `${candidate ? "CANDIDATE" : "CONFIRMED"} · ${geometryLabel}${narrowLabel}`,
        labelX,
        labelY - 6,
      );
      ctx.restore();
    }

    drawReflectivityBoundaries(ctx, transform, boundaries, corridor) {
      const walls = boundaries && Array.isArray(boundaries.walls)
        ? boundaries.walls : [];
      if (!boundaries || boundaries.status !== "confirmed" ||
          walls.length === 0 || (corridor && corridor.status === "confirmed" &&
          Array.isArray(corridor.walls) && corridor.walls.length === 2)) {
        return;
      }
      ctx.save();
      ctx.lineCap = "round";
      for (const wall of walls) {
        const start = window.HanselRadarScene.projectMapPoint(
          transform, wall.start[0], wall.start[1],
        );
        const end = window.HanselRadarScene.projectMapPoint(
          transform, wall.end[0], wall.end[1],
        );
        const stroke = (color, width) => {
          ctx.strokeStyle = color;
          ctx.lineWidth = width;
          ctx.beginPath();
          ctx.moveTo(start.x, start.y);
          ctx.lineTo(end.x, end.y);
          ctx.stroke();
        };
        stroke("rgba(1, 8, 12, 0.90)", 6.0);
        stroke("rgba(122, 238, 230, 0.98)", 2.8);
        ctx.fillStyle = "rgba(201, 255, 249, 0.98)";
        ctx.font = "750 10px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "center";
        ctx.textBaseline = "bottom";
        ctx.fillText(
          `MEASURED ${String(wall.side).toUpperCase()} WALL · ` +
            `${Math.round(Number(wall.headingDeg))}°`,
          (start.x + end.x) / 2,
          (start.y + end.y) / 2 - 6,
        );
      }
      ctx.restore();
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
        ctx.globalAlpha = certainty * 0.46;
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
      if (!options || options.showUncertainty !== false) {
        for (const cell of cells) {
          this.drawUncertaintyOutline(ctx, transform, cell);
        }
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
      const rangeM = 0.30;
      const azimuthLimit = Math.PI / 3;
      const tracks = (Array.isArray(filteredTracks) ? filteredTracks : [])
        .filter((track) => {
          if (!this.operatorVisibleTrack(track)) {
            return false;
          }
          const forward = Number(track && track.forward_m);
          const lateral = Number(track && track.lateral_m);
          const distance = Number.isFinite(track && track.distance_m)
            ? track.distance_m
            : Math.hypot(forward, lateral);
          return Number.isFinite(forward) && forward > 0 &&
            Number.isFinite(lateral) && Number.isFinite(distance) &&
            distance >= 0 && distance <= rangeM + 1e-9 &&
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
            Math.hypot(point.forward, point.lateral) <= rangeM + 1e-9 &&
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
      for (const shell of [0.10, 0.20, 0.30]) {
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
      const strokePath = (points, color, width, dash = []) => {
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
        ctx.setLineDash(dash);
        ctx.stroke();
        ctx.setLineDash([]);
      };
      ctx.save();
      const candidates = Array.isArray(plannedPath.turn_candidates)
        ? plannedPath.turn_candidates : [];
      const unresolvedTurn = plannedPath.status === "turn_side_unresolved" ||
        (plannedPath.length_m === 0 && candidates.length === 2);
      if (unresolvedTurn) {
        for (const candidate of candidates) {
          strokePath(
            candidate.points,
            "rgba(255, 166, 42, 0.40)",
            18,
            [7, 7],
          );
          strokePath(
            candidate.points,
            "rgba(255, 231, 128, 1.0)",
            4.5,
            [7, 7],
          );
          if (Array.isArray(candidate.points) && candidate.points.length > 1) {
            const endCoordinates = candidate.points.at(-1);
            const endPoint = window.HanselRadarScene.projectMapPoint(
              transform,
              endCoordinates[0],
              endCoordinates[1],
            );
            ctx.beginPath();
            ctx.arc(endPoint.x, endPoint.y, 7, 0, Math.PI * 2);
            ctx.fillStyle = "rgba(255, 231, 128, 1.0)";
            ctx.fill();
            ctx.fillStyle = "rgba(255, 235, 156, 0.98)";
            ctx.font = "900 12px ui-monospace, SFMono-Regular, Consolas, monospace";
            ctx.textAlign = "center";
            ctx.fillText(
              candidate.direction === "left" ? "LEFT ?" : "RIGHT ?",
              endPoint.x,
              endPoint.y - 13,
            );
          }
        }
        ctx.fillStyle = "rgba(255, 213, 107, 0.98)";
        ctx.font = "900 13px ui-monospace, SFMono-Regular, Consolas, monospace";
        ctx.textAlign = "center";
        ctx.fillText(
          "TURN REQUIRED · SIDE UNRESOLVED",
          transform.originX,
          Math.max(36, transform.originY - 0.24 * transform.scale),
        );
        ctx.restore();
        return;
      }
      const passageWidthM = Number(plannedPath.minimum_passage_width_m);
      const trendGuided = plannedPath.planner_mode ===
        "full_reflectivity_opening";
      const underlayWidth = trendGuided
        ? 12
        : Number.isFinite(passageWidthM) && passageWidthM > 0
          ? passageWidthM * transform.scale
          : 7;
      strokePath(
        plannedPath.points,
        trendGuided ? "rgba(28, 91, 140, 0.44)" : "rgba(5, 9, 12, 0.42)",
        underlayWidth,
      );
      strokePath(plannedPath.points, "rgba(255, 215, 64, 0.96)", 3.2);
      const blockedPrevious = plannedPath.blocked_previous_path;
      const showBlockedTransition =
        plannedPath.stability_status == null ||
        plannedPath.stability_status === "route_changed_blocked";
      if (showBlockedTransition && blockedPrevious &&
          Array.isArray(blockedPrevious.points) &&
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
      } else if (clipShape === "fov") {
        const edgeLateral = transform.halfWidthM;
        const sideReachForward = Math.min(
          transform.forwardMaxM,
          edgeLateral / Math.tan(Math.PI / 3),
        );
        ctx.moveTo(transform.originX, transform.originY);
        ctx.lineTo(
          transform.originX - edgeLateral * transform.scale,
          transform.originY - sideReachForward * transform.scale,
        );
        ctx.lineTo(
          transform.originX - edgeLateral * transform.scale,
          transform.originY - transform.forwardMaxM * transform.scale,
        );
        ctx.lineTo(
          transform.originX + edgeLateral * transform.scale,
          transform.originY - transform.forwardMaxM * transform.scale,
        );
        ctx.lineTo(
          transform.originX + edgeLateral * transform.scale,
          transform.originY - sideReachForward * transform.scale,
        );
        ctx.closePath();
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
      } else if (clipShape === "fov") {
        const edgeLateral = transform.halfWidthM;
        const sideReachForward = Math.min(
          transform.forwardMaxM,
          edgeLateral / Math.tan(Math.PI / 3),
        );
        ctx.moveTo(transform.originX, transform.originY);
        ctx.lineTo(
          transform.originX - edgeLateral * transform.scale,
          transform.originY - sideReachForward * transform.scale,
        );
        ctx.lineTo(
          transform.originX - edgeLateral * transform.scale,
          transform.originY - transform.forwardMaxM * transform.scale,
        );
        ctx.lineTo(
          transform.originX + edgeLateral * transform.scale,
          transform.originY - transform.forwardMaxM * transform.scale,
        );
        ctx.lineTo(
          transform.originX + edgeLateral * transform.scale,
          transform.originY - sideReachForward * transform.scale,
        );
        ctx.closePath();
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
          this.operatorVisibleTrack(track) &&
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

    operatorVisibleTrack(track) {
      return Boolean(track) && (
        track.source !== "heatmap" || track.heatmap_confirmed === true
      );
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
      if (options.clipShape === "fov") {
        return Math.abs(Math.atan2(
          track.lateral_m,
          Math.max(track.forward_m, 1e-9),
        )) <= Math.PI / 3 + 1e-9;
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
      ctx.fillStyle = "#151c19";
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
      const sceneVerification = presentation && !blocked
        ? presentation.sceneVerification : null;
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
      const status = sceneVerification
        ? "verifying"
        : presentationStatus || (this.fetchFailed
          ? "fault"
          : state
            ? state.status
            : "waiting");
      const badge = document.querySelector("#radar-status");
      badge.dataset.status = status;
      badge.textContent = sceneVerification
        ? `SCENE VERIFYING ${sceneVerification.attempt}/${sceneVerification.limit}`
        : STATUS_LABELS[status] || String(status).toUpperCase();
      const warningStrip = document.querySelector("#warning-strip");
      if (warningStrip) {
        warningStrip.dataset.blocked = blocked ? "true" : "false";
      }
      const warning = document.querySelector("#warning-text");
      if (sceneVerification) {
        warning.textContent =
          `SCENE VERIFYING ${sceneVerification.attempt}/${sceneVerification.limit}` +
          " · LAST VALID MAP RETAINED";
      } else if (presentation && presentation.blocked) {
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
        ? sceneVerification
          ? "CALIBRATED SCENE · VERIFYING"
          : "CALIBRATED SCENE"
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

      const nearestPoint = this.nearestConfirmedPoint(presentation);
      const surfaceDistances = presentation &&
        presentation.probabilisticLocalMap &&
        Array.isArray(presentation.probabilisticLocalMap.cells)
        ? presentation.probabilisticLocalMap.cells
          .filter((cell) => Number(cell && cell.probability) > 0.5)
          .map((cell) => Math.hypot(
            Number(cell.forward_m), Number(cell.lateral_m),
          )).filter(Number.isFinite)
        : [];
      const nearestSurface = surfaceDistances.length > 0
        ? Math.min(...surfaceDistances) : null;
      const nearest = [nearestPoint, nearestSurface]
        .filter(Number.isFinite)
        .sort((left, right) => left - right)[0] ?? null;
      const collisionNearest =
        this.nearestConfirmedPoint(presentation, CLOSE_MAX_RANGE_M);
      this.setMetric(
        "metric-nearest",
        nearest === null ? "--" : `${nearest.toFixed(2)} m`,
        "filtered point / first surface",
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
      const adcStatus = document.querySelector("#adc-status-value");
      if (adcStatus) {
        adcStatus.textContent = frame && frame.adc_status === "available" &&
            scene?.adc_range_angle_surface?.direction_semantics ===
              "range_azimuth"
          ? `AVAILABLE · PHASE ANGLE · ${
            scene.adc_range_angle_surface.phase_calibration === "measured"
              ? "CALIBRATED" : "DESIGN CAL"
          }`
          : frame && frame.adc_status === "available"
            ? "AVAILABLE · RANGE ONLY"
          : frame && frame.adc_status
            ? String(frame.adc_status).toUpperCase()
            : "--";
      }
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
      setDiagnosticValue("#ground-rejected-value", "ground_rejected_count");
      setDiagnosticValue(
        "#ground-fringe-rejected-value",
        "ground_fringe_rejected_count",
      );
      setDiagnosticValue(
        "#reflectivity-cell-value",
        "reflectivity_surface_cell_count",
      );
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
      const groundFilterTag = document.querySelector("#ground-filter-tag");
      if (groundFilterTag) {
        const filter = visible && presentation && presentation.groundFilter;
        groundFilterTag.textContent = filter &&
            Number.isFinite(filter.sensorHeightM) &&
            Number.isFinite(filter.sensorPitchDeg) &&
            filter.status === "configured"
          ? `SENSOR ${(filter.sensorHeightM * 100).toFixed(1)}cm / ` +
            `PITCH ${filter.sensorPitchDeg.toFixed(1)}deg / GROUND HIDDEN`
          : "GROUND FILTER --";
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
        memoryElement.textContent = count === "--" ? "--" : `${count} · IN 30cm`;
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
          ["ok", "synthetic", "spi_raw"].includes(scene.calibration_status),
        );
      const pointCloudStatus = document.querySelector("#pointcloud-3d-status");
      if (pointCloudStatus && !visible) {
        pointCloudStatus.textContent = "SENSOR FAULT";
      }
    }

    updateSectors(presentation) {
      const sectors = Array.from({ length: 5 }, () => ({
        tracks: [],
        reflections: [],
        danger: false,
      }));
      if (!presentation || presentation.blocked) {
        sectors.forEach((sector, index) => {
          this.setSector(index, "invalid", "차단");
        });
        return;
      }
      presentation.tracks.forEach((track) => {
        const distance = Number.isFinite(track.distance_m)
          ? track.distance_m
          : Math.hypot(track.forward_m, track.lateral_m);
        if (
          !Number.isFinite(distance) || distance < 0 ||
          distance > MAIN_MAX_RANGE_M + 1e-9
        ) {
          return;
        }
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
      const probabilityCells = presentation.probabilisticLocalMap &&
        Array.isArray(presentation.probabilisticLocalMap.cells)
        ? presentation.probabilisticLocalMap.cells : [];
      probabilityCells.forEach((cell) => {
        const forwardM = Number(cell && cell.forward_m);
        const lateralM = Number(cell && cell.lateral_m);
        const probability = Number(cell && cell.probability);
        const distance = Math.hypot(forwardM, lateralM);
        const rangeUncertaintyM = Number(cell && cell.range_uncertainty_m);
        const lateralUncertaintyM = Number(
          cell && cell.lateral_uncertainty_m,
        );
        const displayUncertaintyM = Math.max(
          0,
          Number.isFinite(rangeUncertaintyM) ? rangeUncertaintyM : 0,
          Number.isFinite(lateralUncertaintyM) ? lateralUncertaintyM : 0,
        );
        const angle = (Math.atan2(lateralM, forwardM) * 180) / Math.PI;
        if (!Number.isFinite(forwardM) || forwardM < 0 ||
            !Number.isFinite(lateralM) || !Number.isFinite(probability) ||
            probability <= 0.5 || !Number.isFinite(distance) ||
            distance - displayUncertaintyM > MAIN_MAX_RANGE_M + 1e-9 ||
            !Number.isFinite(angle) || angle < -70 || angle > 70) {
          return;
        }
        const index = Math.min(
          4,
          Math.max(0, Math.floor(((angle + 70) / 140) * 5)),
        );
        sectors[index].reflections.push({
          distance: Math.min(distance, MAIN_MAX_RANGE_M), probability,
        });
      });
      sectors.forEach((sector, index) => {
        if (sector.tracks.length === 0 && sector.reflections.length === 0) {
          this.setSector(index, "unknown", "UNKNOWN");
          return;
        }
        const nearestTrack = sector.tracks.length > 0
          ? Math.min(...sector.tracks.map((track) =>
            Number.isFinite(track.distance_m)
              ? track.distance_m
              : Math.hypot(track.forward_m, track.lateral_m)))
          : Infinity;
        const nearestReflection = sector.reflections.length > 0
          ? Math.min(...sector.reflections.map((cell) => cell.distance))
          : Infinity;
        const nearest = Math.min(nearestTrack, nearestReflection);
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
          sector.tracks.length === 0
            ? `${Math.round(nearest * 100)}cm REFLECTION`
            : `${nearest < 1 ? Math.round(nearest * 100) + "cm" : nearest.toFixed(1) + "m"} 반사`,
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
