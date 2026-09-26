"use strict";

const $ = (id) => document.getElementById(id);
const canvas = $("map");
const ctx = canvas.getContext("2d");
const schema = "hansel.odometry-map.v1";
const view = {x: 0, y: 0, zoom: 100, width: 1, height: 1, follow: false};
let data = null;
let received = 0;
let connected = false;
let attempted = false;
let snapshotMode = false;
let firstFit = true;
let previousRevision = null;
let relaySignature = "";
let toastTimer;
let pathDistance = 0;
const pointers = new Map();

const finite = (v) => typeof v === "number" && Number.isFinite(v);
const coord = (v) => finite(v) ? v.toFixed(3) : "--";
const ageText = (v) => finite(v) ? `${v.toFixed(2)} s` : "unknown";
const localAge = () => (performance.now() - received) / 1000;
// Rotate only the display: ROS +X is up and ROS +Y is left.
const screen = (p) => ({x: view.width / 2 - (p.y - view.y) * view.zoom, y: view.height / 2 - (p.x - view.x) * view.zoom});
const world = (x, y) => ({x: (view.height / 2 - y) / view.zoom + view.x, y: (view.width / 2 - x) / view.zoom + view.y});
const pan = (dx, dy) => { view.x += dy / view.zoom; view.y += dx / view.zoom; };

function toast(message) {
  $("toast").textContent = message;
  $("toast").hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $("toast").hidden = true; }, 5000);
}

function validate(input) {
  if (!input || input.schema !== schema || typeof input.frame_id !== "string" || input.frame_id.length > 100) throw Error("Not a HANSEL map snapshot.");
  if (!["live", "demo"].includes(input.mode)) throw Error("Invalid map mode.");
  const point = (p) => p && finite(p.x) && finite(p.y) && Math.abs(p.x) <= 1e7 && Math.abs(p.y) <= 1e7;
  if (input.pose !== null && (!point(input.pose) || !finite(input.pose.yaw))) throw Error("Invalid Head pose.");
  if (!Array.isArray(input.path) || input.path.length > 1000 || !input.path.every(point)) throw Error("Invalid or oversized path.");
  if (!Array.isArray(input.relays) || input.relays.length > 32 || !input.relays.every((p) => point(p) && typeof p.id === "string" && p.id.length <= 100 && typeof p.label === "string" && p.label.length <= 100)) throw Error("Invalid relay positions.");
  if (input.pose_age_s !== null && (!finite(input.pose_age_s) || input.pose_age_s < 0)) throw Error("Invalid sample age.");
  if (!Number.isInteger(input.origin_revision) || input.origin_revision < 0) throw Error("Invalid origin revision.");
  const diagnostics = {};
  for (const key of ["odometry", "unit"]) {
    const d = input.diagnostics?.[key];
    if (!d) continue;
    if (!finite(d.age_s) || d.age_s < 0 || !Number.isInteger(d.level) || typeof d.message !== "string" || d.message.length > 240) throw Error("Invalid diagnostics.");
    const values = {};
    for (const name of ["imu_age_s", "wheel_age_s", "state_age_s", "provisional"]) {
      if (typeof d.values?.[name] === "string" && d.values[name].length <= 160) values[name] = d.values[name];
    }
    diagnostics[key] = {...d, values};
  }
  return {...input, diagnostics};
}

function accept(next) {
  const changedOrigin = previousRevision !== null && next.origin_revision !== previousRevision;
  data = next;
  received = performance.now();
  connected = true;
  previousRevision = next.origin_revision;
  pathDistance = next.path.slice(1).reduce((sum, p, i) => sum + Math.hypot(p.x - next.path[i].x, p.y - next.path[i].y), 0);
  if (changedOrigin) {
    firstFit = true;
    toast("Odometry origin reset. Previous trail cleared.");
  }
  if (firstFit && (data.pose || data.path.length || data.relays.length)) {
    fitView();
    firstFit = false;
  }
  const signature = JSON.stringify(data.relays);
  if (signature !== relaySignature) {
    relaySignature = signature;
    $("relays").replaceChildren();
    if (!data.relays.length) {
      const empty = document.createElement("p");
      empty.className = "muted";
      empty.textContent = "No relay positions received.";
      $("relays").append(empty);
    }
    for (const relay of data.relays) {
      const row = document.createElement("div");
      row.className = "relay-row";
      const name = document.createElement("strong");
      name.textContent = relay.label;
      const position = document.createElement("span");
      position.textContent = `X ${coord(relay.x)} m / Y ${coord(relay.y)} m`;
      row.append(name, position);
      $("relays").append(row);
    }
  }
  $("export").disabled = false;
  $("frame").textContent = data.frame_id;
  $("relay-count").textContent = data.relays.length;
  $("samples").textContent = `${data.path.length} / 1000`;
  $("rejected").textContent = finite(data.invalid_messages) ? data.invalid_messages : "--";
  $("x").textContent = coord(data.pose?.x);
  $("y").textContent = coord(data.pose?.y);
  $("yaw").textContent = data.pose ? (data.pose.yaw * 180 / Math.PI).toFixed(1) : "--";
  $("distance").textContent = data.path.length ? pathDistance.toFixed(3) : "--";
}

function health() {
  const elapsed = snapshotMode ? 0 : localAge();
  const age = data?.pose_age_s == null ? null : data.pose_age_s + elapsed;
  const online = connected && elapsed < 1.5;
  const odom = data?.diagnostics?.odometry;
  const unit = data?.diagnostics?.unit;
  const nanoAge = unit?.values?.state_age_s;
  const unitAlert = unit && unit.age_s + elapsed < 3 && (unit.level >= 2 || (nanoAge !== undefined && Number(nanoAge) > 1));
  let status = "Connecting", kind = "waiting";
  if (snapshotMode) { status = "Snapshot"; kind = "snapshot"; }
  else if (!online && attempted) { status = "Disconnected"; kind = "stale"; }
  else if (data?.mode === "demo") { status = "Demo"; kind = "demo"; }
  else if (data && !data.pose) { status = "Waiting for pose"; }
  else if (age !== null && age > 1) { status = "Pose stale"; kind = "stale"; }
  else if (unitAlert || (odom && odom.age_s + elapsed < 3 && odom.level >= 2)) { status = "Sensor alert"; kind = "stale"; }
  else if (data?.pose) { status = "ROS live"; kind = "live"; }
  $("status").textContent = status;
  $("status").className = `status ${kind}`;
  $("pose-age").textContent = ageText(age);
  const sensorAge = (diagnostic, key) => {
    if (!diagnostic || diagnostic.age_s + elapsed > 3) return "unknown";
    const text = diagnostic.values?.[key];
    if (text === "never") return "never";
    const value = text === undefined || text.trim() === "" ? NaN : Number(text);
    return ageText(Number.isFinite(value) && value >= 0 ? value + diagnostic.age_s + elapsed : null);
  };
  $("imu-age").textContent = sensorAge(odom, "imu_age_s");
  $("wheel-age").textContent = sensorAge(odom, "wheel_age_s");
  $("nano-age").textContent = sensorAge(unit, "state_age_s");
  const freshDiagnostic = odom && odom.age_s + elapsed < 3;
  $("diagnostic").textContent = data?.mode === "demo" ? "Synthetic samples. No robot connection." : unitAlert ? `Nano/controller: ${unit.message}` : freshDiagnostic ? odom.message : "Odometry diagnostics unavailable or stale. Sensor health unknown.";
  $("diagnostic").classList.toggle("error", Boolean(unitAlert || (freshDiagnostic && odom.level >= 2)));
  $("empty").hidden = Boolean(data?.pose || data?.path.length || data?.relays.length);
  $("empty").querySelector("strong").textContent = !online && attempted && !snapshotMode ? "Map connection lost" : "Waiting for odometry";
  $("mode-banner").hidden = !(snapshotMode || data?.mode === "demo");
  $("mode-banner").textContent = snapshotMode ? (data?.mode === "demo" ? "SNAPSHOT / SYNTHETIC DEMO" : "SAVED SNAPSHOT / NOT LIVE") : "SYNTHETIC DEMO";
  return !snapshotMode && (age === null || age > 1 || !online);
}

function setFollow(enabled) {
  view.follow = enabled;
  $("follow").setAttribute("aria-pressed", String(enabled));
}

function fitView() {
  const points = [{x: 0, y: 0}, ...(data?.path || []), ...(data?.relays || [])];
  if (data?.pose) points.push(data.pose);
  const xs = points.map((p) => p.x), ys = points.map((p) => p.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
  view.x = (minX + maxX) / 2;
  view.y = (minY + maxY) / 2;
  view.zoom = Math.max(0.00001, Math.min(1200, Math.max(80, view.width - 160) / Math.max(2, maxY - minY), Math.max(80, view.height - 140) / Math.max(2, maxX - minX)));
  view.y -= 30 / view.zoom;
  setFollow(false);
}

function zoom(factor, x = view.width / 2, y = view.height / 2) {
  const before = world(x, y);
  view.zoom = Math.max(0.00001, Math.min(3000, view.zoom * factor));
  const after = world(x, y);
  view.x += before.x - after.x;
  view.y += before.y - after.y;
}

function niceStep(value) {
  const base = 10 ** Math.floor(Math.log10(value));
  return [1, 2, 5, 10].find((n) => n * base >= value) * base;
}

function draw() {
  const stale = health();
  if (view.follow && data?.pose) { view.x = data.pose.x; view.y = data.pose.y; }
  const {width: w, height: h} = view;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#f5f7f7";
  ctx.fillRect(0, 0, w, h);
  const step = niceStep(55 / view.zoom);
  const low = world(w, h), high = world(0, 0);
  ctx.lineWidth = 1;
  ctx.strokeStyle = "#e1e7e6";
  ctx.beginPath();
  for (let x = Math.ceil(low.x / step) * step; x <= high.x; x += step) { const p = screen({x, y: 0}); ctx.moveTo(0, p.y); ctx.lineTo(w, p.y); }
  for (let y = Math.ceil(low.y / step) * step; y <= high.y; y += step) { const p = screen({x: 0, y}); ctx.moveTo(p.x, 0); ctx.lineTo(p.x, h); }
  ctx.stroke();
  const origin = screen({x: 0, y: 0});
  ctx.strokeStyle = "#bbc9c7";
  ctx.beginPath(); ctx.moveTo(origin.x, 0); ctx.lineTo(origin.x, h); ctx.moveTo(0, origin.y); ctx.lineTo(w, origin.y); ctx.stroke();
  ctx.font = "11px system-ui";
  ctx.fillStyle = "#7c8c8b";
  ctx.textAlign = "left";
  const labelX = Math.max(8, Math.min(w - 90, origin.x + 8));
  for (let x = Math.ceil(low.x / step) * step; x <= high.x; x += step) { const p = screen({x, y: 0}); if (Math.abs(x) > step / 10 && p.y > 55 && p.y < h - 55) ctx.fillText(`X ${Number(x.toPrecision(5))}`, labelX, p.y - 5); }
  const labelY = Math.max(55, Math.min(h - 55, origin.y + 16));
  for (let y = Math.ceil(low.y / step) * step; y <= high.y; y += step) { const p = screen({x: 0, y}); if (Math.abs(y) > step / 10 && p.x > 10 && p.x < w - 70) ctx.fillText(`Y ${Number(y.toPrecision(5))}`, p.x + 4, labelY); }
  if (data?.path.length) {
    ctx.strokeStyle = "#258c70"; ctx.lineWidth = 2.5; ctx.lineJoin = "round"; ctx.lineCap = "round";
    ctx.beginPath();
    data.path.forEach((p, i) => { const s = screen(p); if (i === 0) ctx.moveTo(s.x, s.y); else ctx.lineTo(s.x, s.y); });
    ctx.stroke();
    const start = screen(data.path[0]);
    ctx.fillStyle = "#258c70"; ctx.beginPath(); ctx.arc(start.x, start.y, 3, 0, Math.PI * 2); ctx.fill();
  }
  ctx.strokeStyle = "#627571"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(origin.x, origin.y, 5, 0, Math.PI * 2); ctx.stroke();
  ctx.fillStyle = "#667a76"; ctx.fillText("0, 0", origin.x + 10, origin.y + 22);
  for (const relay of data?.relays || []) {
    const p = screen(relay);
    ctx.fillStyle = "#b56298"; ctx.fillRect(p.x - 5, p.y - 5, 10, 10);
    ctx.font = "600 12px system-ui";
    const label = relay.label.length > 28 ? relay.label.slice(0, 25) + "..." : relay.label;
    const textWidth = ctx.measureText(label).width;
    ctx.fillStyle = "#fffffff0"; ctx.fillRect(p.x + 9, p.y - 13, textWidth + 8, 21);
    ctx.fillStyle = "#874976"; ctx.fillText(label, p.x + 13, p.y + 2);
  }
  if (data?.pose) {
    const p = screen(data.pose);
    ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(-Math.PI / 2 - data.pose.yaw);
    ctx.fillStyle = stale ? "#899796" : "#187eb8"; ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(15, 0); ctx.lineTo(-10, -10); ctx.lineTo(-6, 0); ctx.lineTo(-10, 10); ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore(); ctx.fillStyle = stale ? "#778583" : "#226c96"; ctx.font = "600 12px system-ui"; ctx.fillText("Head", p.x + 18, p.y - 16);
  }
  const bar = niceStep(65 / view.zoom);
  const length = bar * view.zoom;
  ctx.strokeStyle = "#6e8280"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(20, h - 44); ctx.lineTo(20, h - 38); ctx.lineTo(20 + length, h - 38); ctx.lineTo(20 + length, h - 44); ctx.stroke();
  $("scale").textContent = bar < 1 ? `${Number((bar * 100).toPrecision(4))} cm` : `${Number(bar.toPrecision(4))} m`;
  requestAnimationFrame(draw);
}

new ResizeObserver(() => {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  view.width = rect.width; view.height = rect.height;
  canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}).observe(canvas);

$("zoom-in").onclick = () => zoom(1.4);
$("zoom-out").onclick = () => zoom(1 / 1.4);
$("fit-view").onclick = fitView;
$("follow").onclick = () => setFollow(!view.follow);
canvas.addEventListener("wheel", (e) => {
  e.preventDefault(); setFollow(false);
  const r = canvas.getBoundingClientRect();
  zoom(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top);
}, {passive: false});
canvas.addEventListener("pointerdown", (e) => {
  if (e.pointerType === "mouse" && e.button !== 0) return;
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, {x: e.clientX, y: e.clientY});
  setFollow(false);
});
canvas.addEventListener("pointermove", (e) => {
  const old = pointers.get(e.pointerId);
  if (!old) return;
  if (pointers.size === 2) {
    const other = [...pointers.entries()].find(([id]) => id !== e.pointerId)[1];
    const before = Math.hypot(old.x - other.x, old.y - other.y);
    const after = Math.hypot(e.clientX - other.x, e.clientY - other.y);
    const r = canvas.getBoundingClientRect();
    if (before > 5 && after > 5) zoom(after / before, (old.x + other.x) / 2 - r.left, (old.y + other.y) / 2 - r.top);
    pan((e.clientX - old.x) / 2, (e.clientY - old.y) / 2);
  } else {
    pan(e.clientX - old.x, e.clientY - old.y);
  }
  pointers.set(e.pointerId, {x: e.clientX, y: e.clientY});
});
for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) canvas.addEventListener(type, (e) => pointers.delete(e.pointerId));
canvas.addEventListener("keydown", (e) => {
  if (e.key === "+" || e.key === "=") zoom(1.4);
  else if (e.key === "-") zoom(1 / 1.4);
  else if (e.key === "Home") fitView();
  else if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.key)) {
    setFollow(false);
    pan((e.key === "ArrowRight" ? -1 : e.key === "ArrowLeft" ? 1 : 0) * 40,
      (e.key === "ArrowUp" ? 1 : e.key === "ArrowDown" ? -1 : 0) * 40);
  } else return;
  e.preventDefault();
});

const radarFrame = $("radar-frame");
function styleRadarFrame() {
  const doc = radarFrame.contentDocument;
  if (doc?.readyState !== "complete" || !doc.head || doc.getElementById("combined-map-style")) return;
  const style = doc.createElement("link");
  style.id = "combined-map-style";
  style.rel = "stylesheet";
  style.href = "/radar-embed.css";
  doc.head.append(style);
  const script = doc.createElement("script");
  script.src = "/radar-embed.js";
  doc.head.append(script);
}
radarFrame.addEventListener("load", styleRadarFrame);
styleRadarFrame();
$("radar-reload").onclick = () => { radarFrame.src = "/radar/"; };

$("import").onclick = () => $("file").click();
$("file").onchange = async (e) => {
  const file = e.target.files[0];
  e.target.value = "";
  if (!file) return;
  try {
    if (file.size > 2_000_000) throw Error("Snapshot exceeds 2 MB.");
    const parsed = validate(JSON.parse(await file.text()));
    snapshotMode = true;
    firstFit = true;
    previousRevision = null;
    accept(parsed);
    $("live").hidden = false;
    toast("Odometry snapshot opened. Radar remains independent.");
  } catch (error) { toast(error.message); }
};
$("live").onclick = () => {
  snapshotMode = false; firstFit = true; previousRevision = null;
  data = null; connected = false; received = 0;
  relaySignature = "";
  for (const id of ["x", "y", "yaw", "distance"]) $(id).textContent = "--";
  $("relays").replaceChildren();
  $("relay-count").textContent = "0";
  $("samples").textContent = "0 / 1000";
  $("rejected").textContent = "0";
  $("live").hidden = true; $("export").disabled = true;
};
$("export").onclick = () => {
  if (!data) return;
  const saved = structuredClone(data);
  if (!snapshotMode) {
    saved.pose_age_s = data.pose_age_s === null ? null : data.pose_age_s + localAge();
    for (const d of Object.values(saved.diagnostics)) d.age_s += localAge();
  }
  saved.saved_at = new Date().toISOString();
  const url = URL.createObjectURL(new Blob([JSON.stringify(saved, null, 2)], {type: "application/json"}));
  const a = document.createElement("a");
  a.href = url; a.download = `hansel-map-${snapshotMode ? "snapshot-" : ""}${saved.saved_at.replace(/[:.]/g, "-")}.json`;
  a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
};

async function poll() {
  if (!snapshotMode) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 1800);
    try {
      const response = await fetch("/api/state", {cache: "no-store", signal: controller.signal});
      if (!response.ok) throw Error("Map server unavailable");
      const next = validate(await response.json());
      if (!snapshotMode) accept(next);
    } catch (error) { if (!snapshotMode) connected = false; }
    finally { attempted = true; clearTimeout(timer); }
  }
  setTimeout(poll, 250);
}
poll();
requestAnimationFrame(draw);
