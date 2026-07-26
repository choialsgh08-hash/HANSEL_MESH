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

  class RadarPanel {
    constructor(root, options) {
      this.root = root;
      this.canvas = root.querySelector("canvas");
      this.ctx = this.canvas.getContext("2d");
      this.rangeSelect = document.querySelector("#range-select");
      this.snrInput = document.querySelector("#snr-input");
      this.snrValue = document.querySelector("#snr-value");
      this.state = null;
      this.fetchFailed = false;
      this.options = Object.assign({ endpoint: "/api/radar" }, options || {});
      this.resizeObserver = new ResizeObserver(() => this.draw());
      this.resizeObserver.observe(this.canvas);
      this.rangeSelect.addEventListener("change", () => this.draw());
      this.snrInput.addEventListener("input", () => {
        this.snrValue.textContent =
          Number(this.snrInput.value) < 0
            ? "끄기"
            : `${this.snrInput.value} dB`;
        this.draw();
      });
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
          this.state = await response.json();
          this.fetchFailed = false;
          this.updateText();
          this.draw();
        } catch (error) {
          this.fetchFailed = true;
          this.updateText(error);
          this.draw();
        } finally {
          window.clearTimeout(timeout);
        }
        await new Promise((resolve) => window.setTimeout(resolve, 100));
      }
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
        : STATUS_LABELS[status] || status.toUpperCase();

      document.querySelector("#warning-text").textContent = this.fetchFailed
        ? `화면 데이터 연결 실패 — ${error ? error.message : "unknown"}`
        : state
          ? state.warning
          : "레이더 화면 초기화 중";

      const frame = state && state.frame;
      const counters = state && state.counters;
      this.setMetric(
        "metric-frame",
        frame ? String(frame.number) : "—",
        frame ? frame.transition : "프레임 없음",
      );
      this.setMetric(
        "metric-points",
        frame ? `${frame.display_point_count}` : "—",
        frame
          ? `${frame.display_point_count}/${frame.source_point_count} 표시`
          : "수신 대기",
      );
      this.setMetric(
        "metric-fps",
        state ? state.fps.toFixed(1) : "—",
        "viewer frame/s",
      );
      this.setMetric(
        "metric-age",
        state && state.age_ms !== null ? `${state.age_ms}` : "—",
        "ms since frame",
      );
      this.setMetric(
        "metric-gap",
        counters
          ? String(
              counters.frame_gaps_total +
                Math.max(
                  counters.sensor_sequence_gaps_total || 0,
                  counters.writer_drops_total || 0,
                ),
            )
          : "—",
        counters
          ? `센서 ${counters.sensor_sequence_gaps_total || 0} · writer ${
              counters.writer_drops_total || 0
            } · 불완전 ${counters.incomplete_frames} · 오류 ${
              counters.parse_errors_total +
              (counters.sensor_sequence_errors_total || 0) +
              (counters.log_sequence_errors_total || 0)
            }`
          : "진단 대기",
      );
      const nearest =
        frame && frame.nearest_corridor_m !== null
          ? `${frame.nearest_corridor_m.toFixed(2)} m`
          : "—";
      this.setMetric(
        "metric-nearest",
        nearest,
        "중앙 ±0.6 m 참고값",
      );

      document.querySelector("#profile-value").textContent =
        frame && frame.profile_id ? frame.profile_id : "—";
      document.querySelector("#calibration-value").textContent =
        frame && frame.calibration_id
          ? frame.calibration_id
          : "uncalibrated";
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
      const height = Math.max(360, Math.floor(rect.height));
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

      const robotX = width / 2;
      const robotY = height - 42;
      const maxRange = Number(this.rangeSelect.value);
      const scale = (height - 82) / maxRange;
      this.drawBackground(ctx, width, height, robotX, robotY, scale, maxRange);
      this.drawPoints(ctx, robotX, robotY, scale, maxRange);
      this.drawRobot(ctx, robotX, robotY);
      this.drawOverlay(ctx, width, height);
    }

    drawBackground(ctx, width, height, robotX, robotY, scale, maxRange) {
      const gradient = ctx.createRadialGradient(
        robotX,
        robotY,
        4,
        robotX,
        robotY,
        height,
      );
      gradient.addColorStop(0, "rgba(14, 43, 48, 0.92)");
      gradient.addColorStop(0.48, "rgba(7, 21, 27, 0.98)");
      gradient.addColorStop(1, "#04090d");
      ctx.fillStyle = gradient;
      ctx.fillRect(0, 0, width, height);

      ctx.save();
      ctx.strokeStyle = "rgba(96, 221, 194, 0.14)";
      ctx.fillStyle = "rgba(153, 232, 215, 0.54)";
      ctx.lineWidth = 1;
      ctx.font = "11px ui-monospace, SFMono-Regular, Consolas, monospace";
      ctx.textAlign = "center";
      for (let meters = 1; meters <= maxRange; meters += 1) {
        const radius = meters * scale;
        ctx.beginPath();
        ctx.arc(robotX, robotY, radius, Math.PI, 2 * Math.PI);
        ctx.stroke();
        if (meters === 1 || meters % 2 === 0 || maxRange <= 5) {
          ctx.fillText(`${meters} m`, robotX + 3, robotY - radius + 13);
        }
      }

      ctx.setLineDash([5, 7]);
      ctx.strokeStyle = "rgba(96, 221, 194, 0.22)";
      [-60, -30, 0, 30, 60].forEach((degrees) => {
        const radians = (degrees * Math.PI) / 180;
        ctx.beginPath();
        ctx.moveTo(robotX, robotY);
        ctx.lineTo(
          robotX + Math.sin(radians) * maxRange * scale,
          robotY - Math.cos(radians) * maxRange * scale,
        );
        ctx.stroke();
      });
      ctx.setLineDash([]);

      const corridor = 0.6 * scale;
      ctx.fillStyle = "rgba(255, 190, 92, 0.045)";
      ctx.fillRect(robotX - corridor, 0, corridor * 2, robotY);
      ctx.strokeStyle = "rgba(255, 190, 92, 0.18)";
      ctx.beginPath();
      ctx.moveTo(robotX - corridor, robotY);
      ctx.lineTo(robotX - corridor, 0);
      ctx.moveTo(robotX + corridor, robotY);
      ctx.lineTo(robotX + corridor, 0);
      ctx.stroke();

      ctx.fillStyle = "rgba(214, 239, 234, 0.62)";
      ctx.font = "600 11px system-ui, sans-serif";
      ctx.textAlign = "left";
      ctx.fillText("← 좌측", 15, robotY - 8);
      ctx.textAlign = "right";
      ctx.fillText("우측 →", width - 15, robotY - 8);
      ctx.restore();
    }

    drawPoints(ctx, robotX, robotY, scale, maxRange) {
      const frame = this.state && this.state.frame;
      if (!frame || !Array.isArray(frame.points)) {
        return;
      }
      const minSnr = Number(this.snrInput.value);
      frame.points.forEach((point) => {
        const forward = point[0];
        const lateral = point[1];
        const velocity = point[3];
        const snrValue = point[4];
        if (forward < 0 || forward > maxRange) {
          return;
        }
        if (minSnr >= 0 && snrValue !== null && snrValue < minSnr) {
          return;
        }
        const x = robotX + lateral * scale;
        const y = robotY - forward * scale;
        let color;
        if (velocity < -0.15) {
          color = [255, 101, 72];
        } else if (velocity > 0.15) {
          color = [70, 189, 255];
        } else {
          color = [224, 238, 232];
        }
        const snr = snrValue === null ? 14 : snrValue;
        const strength = Math.max(0, Math.min(1, (snr - 5) / 25));
        const radius = 2.2 + strength * 4.8;
        const alpha = 0.42 + strength * 0.55;

        ctx.beginPath();
        ctx.fillStyle = `rgba(${color[0]}, ${color[1]}, ${color[2]}, ${alpha * 0.16})`;
        ctx.arc(x, y, radius * 2.2, 0, Math.PI * 2);
        ctx.fill();
        ctx.beginPath();
        ctx.fillStyle = `rgba(${color[0]}, ${color[1]}, ${color[2]}, ${alpha})`;
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    drawRobot(ctx, robotX, robotY) {
      ctx.save();
      ctx.translate(robotX, robotY);
      ctx.fillStyle = "#ffbd62";
      ctx.strokeStyle = "#fff0ce";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, -23);
      ctx.lineTo(15, 9);
      ctx.lineTo(8, 16);
      ctx.lineTo(-8, 16);
      ctx.lineTo(-15, 9);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#071015";
      ctx.fillRect(-5, 1, 10, 11);
      ctx.restore();
    }

    drawOverlay(ctx, width, height) {
      const status = this.fetchFailed
        ? "fault"
        : this.state
          ? this.state.status
          : "waiting";
      if (status === "live") {
        const frame = this.state.frame;
        if (frame && frame.display_point_count === 0) {
          this.overlayMessage(
            ctx,
            width,
            height,
            "NO RETURNS",
            "빈 공간 보장 아님",
            "rgba(255, 189, 98, 0.88)",
          );
        }
        return;
      }
      const messages = {
        waiting: ["RADAR WAITING", "프레임 수신 전 주행 금지"],
        degraded: [
          "RADAR DEGRADED",
          this.state &&
          this.state.warning &&
          this.state.warning.includes("TLV")
            ? "point-cloud TLV 없음 · cfg 확인"
            : "누락·불완전 프레임 확인",
        ],
        stale: ["RADAR STALE", "오래된 화면 — 즉시 정지"],
        fault: ["RADAR FAULT", "데이터 연결 없음 — 즉시 정지"],
        replay_end: ["REPLAY END", "마지막 프레임 고정"],
      };
      const message = messages[status] || messages.fault;
      const color =
        status === "degraded" || status === "replay_end"
          ? "rgba(255, 189, 98, 0.94)"
          : "rgba(255, 83, 80, 0.96)";
      this.overlayMessage(ctx, width, height, message[0], message[1], color);
    }

    overlayMessage(ctx, width, height, title, subtitle, color) {
      const boxWidth = Math.min(430, width - 36);
      const boxHeight = 92;
      const x = (width - boxWidth) / 2;
      const y = Math.max(24, height * 0.36);
      ctx.fillStyle = "rgba(3, 8, 12, 0.88)";
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.fillRect(x, y, boxWidth, boxHeight);
      ctx.strokeRect(x, y, boxWidth, boxHeight);
      ctx.fillStyle = color;
      ctx.textAlign = "center";
      ctx.font = "800 23px system-ui, sans-serif";
      ctx.fillText(title, width / 2, y + 38);
      ctx.fillStyle = "rgba(241, 247, 245, 0.9)";
      ctx.font = "600 14px system-ui, sans-serif";
      ctx.fillText(subtitle, width / 2, y + 66);
    }
  }

  window.HanselRadarPanel = RadarPanel;
})();
