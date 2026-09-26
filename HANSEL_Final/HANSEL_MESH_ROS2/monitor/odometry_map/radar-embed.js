"use strict";

// Read the existing renderer's source mode without changing its scene or planner.
(() => {
  const radarHeader = document.querySelector(".header-status");
  if (!radarHeader || document.querySelector(".radar-source-label")) return;
  const label = document.createElement("span");
  label.className = "radar-source-label";
  radarHeader.prepend(label);
  const updateSourceLabel = () => {
    let source;
    try { source = panel.snapshot?.source?.mode; } catch (_) { source = undefined; }
    label.hidden = source === "follow" || source === "spi";
    label.textContent = source === "demo" ? "SYNTHETIC DEMO" : source === "replay" ? "RECORDED REPLAY" : "SOURCE UNKNOWN";
  };
  updateSourceLabel();
  setInterval(updateSourceLabel, 500);
})();
