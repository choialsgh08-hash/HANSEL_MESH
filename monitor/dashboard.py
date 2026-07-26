#!/usr/bin/env python3
"""Mesh metrics collector + live web dashboard.

Runs on Base (or the operator laptop). Three threads:

  1. UDP listener (:7100) ingests JSON snapshots from each node's metrics_agent.
  2. A sampler appends a merged history record every --interval seconds, so the
     time-series charts have evenly spaced points regardless of agent timing.
  3. An HTTP server (:8080) serves the dashboard page and /api/state.

The frontend merges every node's *direct* links into one mesh topology graph
(edges carry RSSI + BATMAN TQ) and plots RTT / RSSI over time.

No Python third-party packages are required (stdlib only). The normal chart /
graph libraries load from a CDN when internet is available, and the web page has
small built-in fallbacks so field tests still show basic topology and trends
when the laptop is offline.

Run it now without any hardware:
    python monitor/dashboard.py --demo
    # open http://localhost:8080
"""

from __future__ import annotations  # 3.9(Bullseye) 호환

import argparse
import datetime
import json
import os
import random
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from metrics_agent import NODES, build_snapshot


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    from controller.quality_supervisor import QualityConfig, score_quality
except ImportError:
    QualityConfig = None
    score_quality = None

# Shared state, guarded by LOCK.
LOCK = threading.Lock()
LATEST: dict = {}                 # node_name -> {"snap": snapshot, "recv": ts}
VIDEO: dict = {}                  # {"stats": <sample>, "recv": ts} from video_probe
HISTORY: deque = deque(maxlen=180)  # merged records for the charts
EVENTS: deque = deque(maxlen=60)    # reconnect/route-change events from agents
NODE_TIMEOUT_S = 12.0             # mark a node offline after this much silence
RECONNECT_WINDOW_S = 6.0         # keep the "reconnecting" banner up this long

QUALITY_LEVELS = {
    "UNKNOWN": 0,
    "GOOD": 1,
    "TRANSIENT": 2,
    "WARN": 3,
    "DANGER": 4,
}


def _edge_key(a: str, b: str) -> str:
    return "|".join(sorted((a, b)))


def ingest(snapshot: dict, now: float) -> None:
    """Store one agent snapshot keyed by its node name, plus its reconnect events."""
    node = snapshot.get("node", "unknown")
    with LOCK:
        LATEST[node] = {"snap": snapshot, "recv": now}
        for ev in snapshot.get("events", []):
            record = dict(ev)
            record["node"] = node
            record["ts"] = round(now, 1)
            record["time"] = datetime.datetime.fromtimestamp(now).strftime("%H:%M:%S")
            EVENTS.append(record)


def ingest_video(stats: dict, now: float) -> None:
    """Store the latest video-quality sample from video_probe."""
    with LOCK:
        VIDEO["stats"] = stats
        VIDEO["recv"] = now


def _quality_from_status(status: str):
    if status == "WARN":
        return 0.35, 1
    if status == "DANGER":
        return 0.0, 2
    return None, 0


def _worst_ping(e2e: dict) -> dict:
    losses = []
    rtts = []
    for stats in e2e.values():
        if stats.get("loss_pct") is not None:
            losses.append(float(stats["loss_pct"]))
        if stats.get("rtt_avg_ms") is not None:
            rtts.append(float(stats["rtt_avg_ms"]))

    out = {}
    if losses:
        out["loss_pct"] = max(losses)
    if rtts:
        out["rtt_avg_ms"] = max(rtts)
    return out


def _quality_thresholds(target_fps: float) -> dict:
    cfg = QualityConfig(target_fps=target_fps) if QualityConfig else None
    fps_warn_ratio = cfg.fps_warn_ratio if cfg else 0.85
    fps_danger_ratio = cfg.fps_danger_ratio if cfg else 0.60
    err_warn_rate = cfg.err_warn_rate if cfg else 0.20
    err_danger_rate = cfg.err_danger_rate if cfg else 1.00
    drop_warn_rate = cfg.drop_warn_rate if cfg else 1.00
    drop_danger_rate = cfg.drop_danger_rate if cfg else 3.00
    rtt_warn_ms = cfg.rtt_warn_ms if cfg else 120.0
    rtt_danger_ms = cfg.rtt_danger_ms if cfg else 180.0
    tq_warn = cfg.tq_warn if cfg else 200
    tq_danger = cfg.tq_danger if cfg else 180
    return {
        "target_fps": target_fps,
        "fps_warn": round(target_fps * fps_warn_ratio, 2),
        "fps_danger": round(target_fps * fps_danger_ratio, 2),
        "err_warn_rate": err_warn_rate,
        "err_danger_rate": err_danger_rate,
        "drop_warn_rate": drop_warn_rate,
        "drop_danger_rate": drop_danger_rate,
        "rtt_warn_ms": rtt_warn_ms,
        "rtt_danger_ms": rtt_danger_ms,
        "tq_warn": tq_warn,
        "tq_danger": tq_danger,
    }


def evaluate_quality(video, e2e: dict, edges: list, now: float,
                     link_health: dict = None) -> dict:
    """Mirror the controller's video-first quality judgment for the dashboard.

    link_health (weakest direct link) is scored here for operator awareness.
    The motor-stopping control client still omits it until its thresholds are
    validated against real hardware.
    """
    video = video or {}
    target_fps = float(video.get("target_fps") or 15.0)
    thresholds = _quality_thresholds(target_fps)
    ping = _worst_ping(e2e)
    edge_tqs = [int(e["tq"]) for e in edges if e.get("tq") is not None]
    batman = {"selected_tq_min": min(edge_tqs)} if edge_tqs else {}

    if score_quality and QualityConfig:
        cfg = QualityConfig(target_fps=target_fps)
        raw_status, reasons = score_quality(
            video, ping, batman, cfg, now, link=link_health or None)
    else:
        raw_status = "UNKNOWN" if not video else "GOOD"
        reasons = ["quality_supervisor unavailable"] if not video else ["healthy"]

    speed_cap, camera_profile = _quality_from_status(raw_status)
    sample_ts = float(video.get("ts") or 0.0)
    video_age_s = round(now - sample_ts, 2) if sample_ts else None
    return {
        "status": raw_status,
        "level": QUALITY_LEVELS.get(raw_status, 0),
        "speed_cap": speed_cap,
        "camera_profile": camera_profile,
        "reasons": reasons,
        "video_age_s": video_age_s,
        "ping": ping,
        "batman": batman,
        "link": link_health or {},
        "thresholds": thresholds,
    }


def merge_state(now: float) -> dict:
    """Fold every node's latest snapshot into one mesh view."""
    with LOCK:
        latest = {k: dict(v) for k, v in LATEST.items()}

    # Nodes: every known node, online if we heard from it recently.
    online = {
        name: (now - info["recv"]) < NODE_TIMEOUT_S
        for name, info in latest.items()
    }
    nodes = []
    for name, ip in NODES.items():
        if name in latest or name in online:
            nodes.append({"id": name, "ip": ip, "online": online.get(name, False)})

    # Edges: union of all *direct* radio links, deduped undirected.
    # Also track the weakest link (worst signal, longest peer silence) so the
    # quality judgment can warn of a reconnect before video/RTT degrade.
    edges_acc: dict = {}
    link_signals: list = []
    link_inactives: list = []
    for name, info in latest.items():
        for link in info["snap"].get("links", []):
            if not link.get("direct"):
                continue
            peer = link.get("peer")
            if not peer or peer == name:
                continue
            key = _edge_key(name, peer)
            acc = edges_acc.setdefault(key, {"rssi": [], "tq": []})
            sig = link.get("signal_avg_dbm", link.get("signal_dbm"))
            if sig is not None:
                acc["rssi"].append(sig)
                link_signals.append(sig)
            if "tq" in link:
                acc["tq"].append(link["tq"])
            if link.get("inactive_ms") is not None:
                link_inactives.append(link["inactive_ms"])

    edges = []
    for key, acc in edges_acc.items():
        a, b = key.split("|")
        rssi = round(sum(acc["rssi"]) / len(acc["rssi"])) if acc["rssi"] else None
        tq = round(sum(acc["tq"]) / len(acc["tq"])) if acc["tq"] else None
        edges.append({"from": a, "to": b, "rssi": rssi, "tq": tq,
                      "directions": len(acc["rssi"]) or len(acc["tq"])})

    # End-to-end: each node's ping results -> "from->to": stats.
    e2e = {}
    for name, info in latest.items():
        for target, stats in info["snap"].get("end_to_end", {}).items():
            e2e[f"{name}->{target}"] = stats

    # Single network scalar for the video<->comms correlation chart: the
    # weakest (worst) link RSSI is what bottlenecks the video stream.
    rssis = [e["rssi"] for e in edges if e["rssi"] is not None]
    net_rssi_worst = min(rssis) if rssis else None

    with LOCK:
        video = dict(VIDEO.get("stats") or {}) if (
            VIDEO.get("recv") and (now - VIDEO["recv"]) < NODE_TIMEOUT_S) else None

    link_health: dict = {}
    if link_signals:
        link_health["signal_worst_dbm"] = min(link_signals)
    if link_inactives:
        link_health["inactive_worst_ms"] = max(link_inactives)

    quality = evaluate_quality(video, e2e, edges, now, link_health)

    with LOCK:
        all_events = list(EVENTS)
    recent = [e for e in all_events if now - e.get("ts", 0.0) < RECONNECT_WINDOW_S]

    return {"nodes": nodes, "edges": edges, "e2e": e2e,
            "video": video, "quality": quality,
            "link_health": link_health,
            "events": all_events[-15:],
            "reconnect_active": bool(recent),
            "reconnect_latest": recent[-1] if recent else None,
            "net_rssi_worst": net_rssi_worst}


def sampler_loop(interval: float, log_path: str = None) -> None:
    """Append a compact history record at a steady cadence for the charts.

    If log_path is given, also append each record as one JSON line so the full
    comms + video data (with a human-readable receive time) is saved to disk.
    """
    log_fh = open(log_path, "a", encoding="utf-8") if log_path else None
    if log_fh:
        print(f"[collector] logging combined metrics to {log_path}")
    while True:
        now = time.time()
        state = merge_state(now)
        vid = state.get("video") or {}
        quality = state.get("quality") or {}
        rec = {
            "ts": round(now, 1),
            # Human-readable receive time (local clock of the collector).
            "time": datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S"),
            "rssi": {_edge_key(e["from"], e["to"]): e["rssi"]
                     for e in state["edges"] if e["rssi"] is not None},
            "tq": {_edge_key(e["from"], e["to"]): e["tq"]
                   for e in state["edges"] if e["tq"] is not None},
            "rtt": {k: v.get("rtt_avg_ms")
                    for k, v in state["e2e"].items() if "rtt_avg_ms" in v},
            # Video quality + the network scalar, on the same time axis.
            "vid_fps": vid.get("fps"),
            "vid_fps_ratio": vid.get("fps_ratio"),
            "vid_err": vid.get("err_rate"),
            "vid_drop": vid.get("drop_rate"),
            "vid_bitrate": vid.get("bitrate_kbps"),
            "net_rssi": state.get("net_rssi_worst"),
            "quality_level": quality.get("level"),
            "quality_status": quality.get("status"),
        }
        with LOCK:
            HISTORY.append(rec)
        if log_fh:
            log_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
            log_fh.flush()
        time.sleep(interval)


def udp_listener(host: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    print(f"[collector] UDP listening on {host}:{port}")
    while True:
        try:
            data, peer = sock.recvfrom(65535)
            snap = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            print(f"[collector] bad packet: {exc}")
            continue
        if "video" in snap:               # from video_probe.py
            ingest_video(snap["video"], time.time())
        else:                             # from metrics_agent.py
            ingest(snap, time.time())


# --------------------------------------------------------------------------- #
# Demo data generator (no hardware): a base-node1-node2-head chain.           #
# --------------------------------------------------------------------------- #

DEMO_CHAIN = ["base", "node1", "node2", "head"]
# Keys normalized through _edge_key so lookups match regardless of order.
DEMO_RSSI = {
    _edge_key("base", "node1"): -50,
    _edge_key("node1", "node2"): -64,
    _edge_key("node2", "head"): -71,
}


def _tq_from_rssi(rssi: float) -> int:
    tq = int(255 + (rssi + 40) * 3.0)
    return max(0, min(255, tq))


def demo_loop(interval: float) -> None:
    """Synthesize realistic per-node snapshots and feed the real ingest path."""
    print("[collector] DEMO mode: generating a 4-node chain mesh")
    tick = 0
    while True:
        now = time.time()
        tick += 1
        jitter = {k: v + random.uniform(-4, 4) for k, v in DEMO_RSSI.items()}
        for idx, node in enumerate(DEMO_CHAIN):
            neighbors = []
            if idx > 0:
                neighbors.append(DEMO_CHAIN[idx - 1])
            if idx < len(DEMO_CHAIN) - 1:
                neighbors.append(DEMO_CHAIN[idx + 1])

            station_lines = []
            neigh_lines = []
            o_lines = ["   Originator   last-seen (#/255)   Nexthop [outIF]"]
            neigh_table = []
            for peer in neighbors:
                rssi = jitter[_edge_key(node, peer)]
                tq = _tq_from_rssi(rssi)
                mac = f"02:aa:bb:00:00:{NODES[peer].split('.')[-1].zfill(2)}"
                station_lines.append(
                    f"Station {mac} (on wlan0)\n"
                    f"\tinactive time:\t100 ms\n"
                    f"\tsignal:  \t{round(rssi)} dBm\n"
                    f"\tsignal avg:\t{round(rssi)} dBm\n"
                    f"\ttx bitrate:\t{max(6, 54 + (rssi + 50)):.1f} MBit/s\n"
                    f"\trx bitrate:\t{max(6, 48 + (rssi + 50)):.1f} MBit/s\n"
                )
                neigh_table.append(f"     wlan0\t{mac}\t   0.500s")
                o_lines.append(f" * {mac}    0.500s   ({tq}) {mac} [ wlan0]")

            ip_lines = [f"{NODES[p]} lladdr 02:aa:bb:00:00:"
                        f"{NODES[p].split('.')[-1].zfill(2)} REACHABLE"
                        for p in neighbors]
            links_text = {
                "station": "\n".join(station_lines),
                "batctl_o": "\n".join(o_lines),
                "batctl_n": "IF Neighbor last-seen\n" + "\n".join(neigh_table),
                "ip_neigh": "\n".join(ip_lines),
            }

            # End-to-end RTT: ping base from each node, ~3ms per hop.
            hops = abs(idx - 0)
            rtt = max(0.5, hops * 3.0 + random.uniform(-1, 1))
            loss = 0.0 if rtt < 14 else round(random.uniform(0, 20), 1)
            ping_text = {
                "base": "3 packets transmitted, 3 received, "
                        f"{loss}% packet loss, time 2002ms\n"
                        f"rtt min/avg/max/mdev = {rtt:.3f}/{rtt:.3f}/"
                        f"{rtt:.3f}/0.100 ms"
            } if node != "base" else {}

            snap = build_snapshot(node, "wlan0", links_text, ping_text, now)
            # Every ~8th cycle, simulate a reconnect on node1 so the banner and
            # event log are visible in demo mode without real hardware.
            if node == "node1" and tick % 8 == 0:
                kind = ["route_changed", "neighbor_lost",
                        "neighbor_gained"][(tick // 8) % 3]
                ev = {"type": kind, "peer": "node2", "mac": "02:aa:bb:00:00:0c"}
                if kind == "route_changed":
                    ev["from"] = "02:aa:bb:00:00:0c"
                    ev["to"] = "02:aa:bb:00:00:0a"
                snap["events"] = [ev]
            ingest(snap, now)

        # Synthesize video quality that tracks the weakest link: as the worst
        # RSSI drops below ~-65 dBm, decode errors climb and fps falls.
        worst = min(jitter.values())
        stress = max(0.0, -65 - worst)            # 0 when healthy, grows as link weakens
        err_rate = round(stress * 1.5 + random.uniform(0, 1), 2)
        fps = round(max(3.0, 15.0 - stress * 0.45 - random.uniform(0, 0.5)), 1)
        ingest_video({
            "fps": fps,
            "target_fps": 15.0,
            "err_rate": err_rate,
            "drop_rate": round(err_rate * 0.25, 2),
            "bitrate_kbps": round(1200 * fps / 15.0, 1),
        }, now)
        time.sleep(interval)


# --------------------------------------------------------------------------- #
# HTTP server                                                                 #
# --------------------------------------------------------------------------- #

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):  # quieter console
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            path = os.path.join(HERE, "web", "index.html")
            with open(path, "rb") as fh:
                self._send(200, fh.read(), "text/html; charset=utf-8")
        elif self.path.startswith("/api/state"):
            now = time.time()
            state = merge_state(now)
            with LOCK:
                state["history"] = list(HISTORY)
            state["updated"] = round(now, 1)
            body = json.dumps(state).encode("utf-8")
            self._send(200, body, "application/json")
        else:
            self._send(404, b"not found", "text/plain")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Mesh metrics collector + dashboard.")
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--udp-host", default="0.0.0.0")
    p.add_argument("--udp-port", type=int, default=7100)
    p.add_argument("--interval", type=float, default=2.0,
                   help="history sampling cadence (s)")
    p.add_argument("--demo", action="store_true",
                   help="generate synthetic data, no agents needed")
    p.add_argument("--log", default=None,
                   help="save combined comms+video metrics to this JSONL file")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    threading.Thread(target=sampler_loop, args=(args.interval, args.log),
                     daemon=True).start()
    if args.demo:
        threading.Thread(target=demo_loop, args=(args.interval,),
                         daemon=True).start()
    else:
        threading.Thread(target=udp_listener,
                         args=(args.udp_host, args.udp_port),
                         daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    print(f"[dashboard] open http://localhost:{args.http_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[dashboard] stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
