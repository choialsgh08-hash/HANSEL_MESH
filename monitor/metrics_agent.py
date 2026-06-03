#!/usr/bin/env python3
"""Per-node mesh metrics agent.

Runs on each Pi. Collects link-layer metrics this node can directly observe
(RSSI / bitrate per neighbor via `iw station dump`, BATMAN TQ via `batctl o`,
neighbor last-seen via `batctl n`) plus end-to-end RTT (ping) to known nodes,
maps MAC -> node name via the bat0 ARP table, and emits one JSON snapshot.

Designed to be testable WITHOUT hardware: pass --sample <dir> to parse captured
command output (station.txt, batctl_o.txt, batctl_n.txt, ip_neigh.txt) instead
of running the real commands. This lets you validate parsing on any machine.

Typical use on a Pi:
    python3 monitor/metrics_agent.py --self node1 --loop --interval 5 \
        --send 192.168.50.1:7100
"""

from __future__ import annotations  # 3.9(Bullseye) 호환: str | None 같은 표기 지원

import argparse
import json
import re
import socket
import subprocess
import sys
import time


# Known mesh nodes: name -> bat0 IP. Matches configs/*.env.
NODES = {
    "base": "192.168.50.1",
    "head": "192.168.50.10",
    "node1": "192.168.50.11",
    "node2": "192.168.50.12",
    "node3": "192.168.50.13",
}

MAC_RE = r"[0-9a-fA-F]{2}(?::[0-9a-fA-F]{2}){5}"


# --------------------------------------------------------------------------- #
# Parsers (pure functions: text in -> dict out, no side effects)              #
# --------------------------------------------------------------------------- #

def parse_station_dump(text: str) -> dict:
    """Parse `iw dev <if> station dump`.

    Returns {mac: {signal_dbm, signal_avg_dbm, tx_mbit, rx_mbit, inactive_ms}}.
    """
    stations: dict = {}
    current = None
    for line in text.splitlines():
        m = re.match(rf"\s*Station\s+({MAC_RE})", line)
        if m:
            current = m.group(1).lower()
            stations[current] = {}
            continue
        if current is None:
            continue
        line = line.strip()
        if line.startswith("signal avg:"):
            v = _first_int(line)
            if v is not None:
                stations[current]["signal_avg_dbm"] = v
        elif line.startswith("signal:"):
            v = _first_int(line)
            if v is not None:
                stations[current]["signal_dbm"] = v
        elif line.startswith("tx bitrate:"):
            v = _first_float(line)
            if v is not None:
                stations[current]["tx_mbit"] = v
        elif line.startswith("rx bitrate:"):
            v = _first_float(line)
            if v is not None:
                stations[current]["rx_mbit"] = v
        elif line.startswith("inactive time:"):
            v = _first_int(line)
            if v is not None:
                stations[current]["inactive_ms"] = v
    return stations


def parse_batctl_o(text: str) -> dict:
    """Parse `batctl o` (originators). Returns {mac: {tq, last_seen_s, nexthop}}.

    Only the selected best path (line marked with '*') is kept per originator.
    """
    originators: dict = {}
    # Example: " * 02:11:.. 0.520s (245) 02:11:.. [ wlan0]"
    line_re = re.compile(
        rf"^\s*\*\s+({MAC_RE})\s+([\d.]+)s\s+\((\d+)\)\s+({MAC_RE})"
    )
    for line in text.splitlines():
        m = line_re.match(line)
        if not m:
            continue
        mac = m.group(1).lower()
        originators[mac] = {
            "last_seen_s": float(m.group(2)),
            "tq": int(m.group(3)),
            "nexthop": m.group(4).lower(),
        }
    return originators


def parse_batctl_n(text: str) -> dict:
    """Parse `batctl n` (direct neighbors). Returns {mac: last_seen_s}."""
    neighbors: dict = {}
    line_re = re.compile(rf"^\s*\S+\s+({MAC_RE})\s+([\d.]+)s")
    for line in text.splitlines():
        if "Neighbor" in line and "last-seen" in line:
            continue  # header
        m = line_re.match(line)
        if m:
            neighbors[m.group(1).lower()] = float(m.group(2))
    return neighbors


def parse_ip_neigh(text: str) -> dict:
    """Parse `ip neigh show dev bat0`. Returns {mac: ip}."""
    mac_to_ip: dict = {}
    line_re = re.compile(rf"^(\d+\.\d+\.\d+\.\d+)\s+lladdr\s+({MAC_RE})")
    for line in text.splitlines():
        m = line_re.match(line.strip())
        if m:
            mac_to_ip[m.group(2).lower()] = m.group(1)
    return mac_to_ip


def parse_ping(text: str) -> dict:
    """Parse Linux `ping -c N` output. Returns {rtt_avg_ms, loss_pct}."""
    result: dict = {}
    loss = re.search(r"(\d+(?:\.\d+)?)%\s*packet loss", text)
    if loss:
        result["loss_pct"] = float(loss.group(1))
    rtt = re.search(r"=\s*[\d.]+/([\d.]+)/[\d.]+/[\d.]+\s*ms", text)
    if rtt:
        result["rtt_avg_ms"] = float(rtt.group(1))
    return result


def _first_int(line: str):
    m = re.search(r"-?\d+", line)
    return int(m.group()) if m else None


def _first_float(line: str):
    m = re.search(r"-?\d+(?:\.\d+)?", line)
    return float(m.group()) if m else None


# --------------------------------------------------------------------------- #
# Command runners (real hardware)                                             #
# --------------------------------------------------------------------------- #

def _run(cmd: list) -> str:
    try:
        out = subprocess.run(
            cmd, capture_output=True, text=True, timeout=10
        )
        return out.stdout + out.stderr
    except (subprocess.SubprocessError, FileNotFoundError) as exc:
        print(f"[warn] command failed: {' '.join(cmd)} ({exc})", file=sys.stderr)
        return ""


def ip_to_node(ip: str) -> str:
    for name, node_ip in NODES.items():
        if node_ip == ip:
            return name
    return ip


# --------------------------------------------------------------------------- #
# Snapshot assembly                                                           #
# --------------------------------------------------------------------------- #

def build_snapshot(self_name: str, mesh_if: str, links_text: dict,
                   ping_targets: dict, timestamp: float) -> dict:
    """Combine parsed command outputs into one snapshot dict.

    links_text: {"station": ..., "batctl_o": ..., "batctl_n": ..., "ip_neigh": ...}
    ping_targets: {node_name: ping_output_text}
    """
    stations = parse_station_dump(links_text.get("station", ""))
    originators = parse_batctl_o(links_text.get("batctl_o", ""))
    neighbors = parse_batctl_n(links_text.get("batctl_n", ""))
    mac_to_ip = parse_ip_neigh(links_text.get("ip_neigh", ""))

    # Union of all MACs we have any data for.
    macs = set(stations) | set(originators) | set(neighbors)
    links = []
    for mac in sorted(macs):
        ip = mac_to_ip.get(mac)
        entry = {
            "mac": mac,
            "peer": ip_to_node(ip) if ip else mac,
            "ip": ip,
        }
        entry.update(stations.get(mac, {}))
        if mac in originators:
            entry["tq"] = originators[mac]["tq"]
            entry["last_seen_s"] = originators[mac]["last_seen_s"]
            entry["nexthop"] = originators[mac]["nexthop"]
        if mac in neighbors:
            entry["neighbor_last_seen_s"] = neighbors[mac]
            entry["direct"] = True
        links.append(entry)

    e2e = {}
    for name, text in ping_targets.items():
        stats = parse_ping(text)
        if stats:
            e2e[name] = stats

    return {
        "node": self_name,
        "mesh_if": mesh_if,
        "ts": round(timestamp, 3),
        "links": links,
        "end_to_end": e2e,
    }


def collect_real(self_name: str, mesh_if: str, ping_list: list,
                 timestamp: float) -> dict:
    links_text = {
        "station": _run(["iw", "dev", mesh_if, "station", "dump"]),
        "batctl_o": _run(["batctl", "o"]),
        "batctl_n": _run(["batctl", "n"]),
        "ip_neigh": _run(["ip", "neigh", "show", "dev", "bat0"]),
    }
    ping_targets = {}
    for name in ping_list:
        ip = NODES.get(name)
        if not ip:
            continue
        ping_targets[name] = _run(["ping", "-c", "3", "-W", "1", ip])
    return build_snapshot(self_name, mesh_if, links_text, ping_targets, timestamp)


def collect_sample(sample_dir: str, self_name: str, mesh_if: str,
                   timestamp: float) -> dict:
    """Read captured command output from a directory (no hardware needed)."""
    import os

    def read(name):
        path = os.path.join(sample_dir, name)
        if os.path.exists(path):
            with open(path, encoding="utf-8") as fh:
                return fh.read()
        return ""

    links_text = {
        "station": read("station.txt"),
        "batctl_o": read("batctl_o.txt"),
        "batctl_n": read("batctl_n.txt"),
        "ip_neigh": read("ip_neigh.txt"),
    }
    ping_targets = {}
    for name in NODES:
        text = read(f"ping_{name}.txt")
        if text:
            ping_targets[name] = text
    return build_snapshot(self_name, mesh_if, links_text, ping_targets, timestamp)


# --------------------------------------------------------------------------- #
# Output                                                                      #
# --------------------------------------------------------------------------- #

def emit(snapshot: dict, send_to: str | None) -> None:
    payload = json.dumps(snapshot)
    print(payload, flush=True)
    if send_to:
        host, _, port = send_to.partition(":")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(payload.encode("utf-8"), (host, int(port)))
        finally:
            sock.close()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Collect mesh link + RTT metrics.")
    p.add_argument("--self", dest="self_name", default="unknown",
                   help="this node's name (base/head/node1/...)")
    p.add_argument("--mesh-if", default="wlan0", help="wireless interface")
    p.add_argument("--ping", nargs="*", default=[],
                   help="node names to ping for RTT (e.g. --ping head base)")
    p.add_argument("--send", default=None,
                   help="forward JSON to collector as UDP host:port")
    p.add_argument("--loop", action="store_true", help="run continuously")
    p.add_argument("--interval", type=float, default=5.0,
                   help="seconds between samples in --loop mode")
    p.add_argument("--sample", default=None,
                   help="parse captured output from this dir instead of hardware")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    # Date.now equivalent: time.time() is fine at runtime (not a workflow).
    base_ts = time.time()

    def one(ts):
        if args.sample:
            snap = collect_sample(args.sample, args.self_name, args.mesh_if, ts)
        else:
            snap = collect_real(args.self_name, args.mesh_if, args.ping, ts)
        emit(snap, args.send)

    if not args.loop:
        one(base_ts)
        return 0

    while True:
        one(time.time())
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
