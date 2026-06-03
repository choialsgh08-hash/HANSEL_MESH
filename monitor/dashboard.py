#!/usr/bin/env python3
"""Mesh metrics collector + live web dashboard.

Runs on Base (or the operator laptop). Three threads:

  1. UDP listener (:7100) ingests JSON snapshots from each node's metrics_agent.
  2. A sampler appends a merged history record every --interval seconds, so the
     time-series charts have evenly spaced points regardless of agent timing.
  3. An HTTP server (:8080) serves the dashboard page and /api/state.

The frontend merges every node's *direct* links into one mesh topology graph
(edges carry RSSI + BATMAN TQ) and plots RTT / RSSI over time.

No third-party packages required (stdlib only). The chart/graph libraries load
from a CDN, so the dashboard machine needs internet for the page assets; the
metrics data itself is fully local.

Run it now without any hardware:
    python monitor/dashboard.py --demo
    # open http://localhost:8080
"""

from __future__ import annotations  # 3.9(Bullseye) 호환

import argparse
import json
import os
import random
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from metrics_agent import NODES, build_snapshot


HERE = os.path.dirname(os.path.abspath(__file__))

# Shared state, guarded by LOCK.
LOCK = threading.Lock()
LATEST: dict = {}                 # node_name -> {"snap": snapshot, "recv": ts}
HISTORY: deque = deque(maxlen=180)  # merged records for the charts
NODE_TIMEOUT_S = 12.0             # mark a node offline after this much silence


def _edge_key(a: str, b: str) -> str:
    return "|".join(sorted((a, b)))


def ingest(snapshot: dict, now: float) -> None:
    """Store one agent snapshot keyed by its node name."""
    node = snapshot.get("node", "unknown")
    with LOCK:
        LATEST[node] = {"snap": snapshot, "recv": now}


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
    edges_acc: dict = {}
    for name, info in latest.items():
        for link in info["snap"].get("links", []):
            if not link.get("direct"):
                continue
            peer = link.get("peer")
            if not peer or peer == name:
                continue
            key = _edge_key(name, peer)
            acc = edges_acc.setdefault(key, {"rssi": [], "tq": []})
            if "signal_avg_dbm" in link:
                acc["rssi"].append(link["signal_avg_dbm"])
            elif "signal_dbm" in link:
                acc["rssi"].append(link["signal_dbm"])
            if "tq" in link:
                acc["tq"].append(link["tq"])

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

    return {"nodes": nodes, "edges": edges, "e2e": e2e}


def sampler_loop(interval: float) -> None:
    """Append a compact history record at a steady cadence for the charts."""
    while True:
        now = time.time()
        state = merge_state(now)
        rec = {
            "ts": round(now, 1),
            "rssi": {_edge_key(e["from"], e["to"]): e["rssi"]
                     for e in state["edges"] if e["rssi"] is not None},
            "rtt": {k: v.get("rtt_avg_ms")
                    for k, v in state["e2e"].items() if "rtt_avg_ms" in v},
        }
        with LOCK:
            HISTORY.append(rec)
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
    while True:
        now = time.time()
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
            ingest(snap, now)
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
    return p.parse_args()


def main() -> int:
    args = parse_args()

    threading.Thread(target=sampler_loop, args=(args.interval,),
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
