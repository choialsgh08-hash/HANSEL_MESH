#!/usr/bin/env python3
"""HANSEL_MESH-compatible per-node BATMAN-adv metrics agent.

Produces the same top-level UDP JSON contract consumed by
hansel_network_adapter: node, mesh_if, bat_if, ts, links, end_to_end, bat0.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

NODES = {
    "base": "192.168.50.1",
    "head": "192.168.50.10",
    "node1": "192.168.50.11",
    "node2": "192.168.50.12",
    "node3": "192.168.50.13",
}
MAC_RE = r"[0-9a-fA-F]{2}(?::[0-9a-fA-F]{2}){5}"


def _first_int(line: str):
    m = re.search(r"-?\d+", line)
    return int(m.group()) if m else None


def _first_float(line: str):
    m = re.search(r"-?\d+(?:\.\d+)?", line)
    return float(m.group()) if m else None


def parse_station_dump(text: str) -> dict:
    stations: dict = {}
    current = None
    for line in text.splitlines():
        match = re.match(rf"\s*Station\s+({MAC_RE})", line)
        if match:
            current = match.group(1).lower()
            stations[current] = {}
            continue
        if current is None:
            continue
        line = line.strip()
        mapping = {
            "signal avg:": ("signal_avg_dbm", _first_int),
            "signal:": ("signal_dbm", _first_int),
            "tx bitrate:": ("tx_mbit", _first_float),
            "rx bitrate:": ("rx_mbit", _first_float),
            "inactive time:": ("inactive_ms", _first_int),
            "tx retries:": ("tx_retries", _first_int),
            "tx failed:": ("tx_failed", _first_int),
            "expected throughput:": ("expected_mbps", _first_float),
        }
        for prefix, (key, parser) in mapping.items():
            if line.startswith(prefix):
                value = parser(line)
                if value is not None:
                    stations[current][key] = value
                break
    return stations


def parse_batctl_o(text: str) -> dict:
    result = {}
    pattern = re.compile(rf"^\s*\*\s+({MAC_RE})\s+([\d.]+)s\s+\((\d+)\)\s+({MAC_RE})")
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            result[match.group(1).lower()] = {
                "last_seen_s": float(match.group(2)),
                "tq": int(match.group(3)),
                "nexthop": match.group(4).lower(),
            }
    return result


def parse_batctl_n(text: str) -> dict:
    result = {}
    pattern = re.compile(rf"^\s*\S+\s+({MAC_RE})\s+([\d.]+)s")
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            result[match.group(1).lower()] = float(match.group(2))
    return result


def parse_ip_neigh(text: str) -> dict:
    result = {}
    pattern = re.compile(rf"^(\d+\.\d+\.\d+\.\d+)\s+.*lladdr\s+({MAC_RE})")
    for line in text.splitlines():
        match = pattern.match(line.strip())
        if match:
            result[match.group(2).lower()] = match.group(1)
    return result


def parse_ping(text: str) -> dict:
    result = {}
    loss = re.search(r"(\d+(?:\.\d+)?)%\s*packet loss", text)
    if loss:
        result["loss_pct"] = float(loss.group(1))
    rtt = re.search(r"=\s*[\d.]+/([\d.]+)/[\d.]+/[\d.]+\s*ms", text)
    if rtt:
        result["rtt_avg_ms"] = float(rtt.group(1))
    return result


def parse_bat_stats(text: str) -> dict:
    stats = {}
    lines = text.splitlines()
    for index, line in enumerate(lines):
        header = line.strip()
        if header.startswith("RX:") and index + 1 < len(lines):
            nums = lines[index + 1].split()
            if len(nums) >= 4 and all(x.isdigit() for x in nums[:4]):
                stats.update(rx_bytes=int(nums[0]), rx_packets=int(nums[1]), rx_dropped=int(nums[3]))
        elif header.startswith("TX:") and index + 1 < len(lines):
            nums = lines[index + 1].split()
            if len(nums) >= 4 and all(x.isdigit() for x in nums[:4]):
                stats.update(tx_bytes=int(nums[0]), tx_packets=int(nums[1]), tx_dropped=int(nums[3]))
    return stats


def _run(command: list[str]) -> str:
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=10, check=False)
        return completed.stdout + completed.stderr
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"[warn] command failed: {' '.join(command)} ({exc})", file=sys.stderr)
        return ""


def _node_for_ip(ip: str | None) -> str:
    if ip is None:
        return ""
    for name, value in NODES.items():
        if value == ip:
            return name
    return ip


def build_snapshot(self_name: str, mesh_if: str, bat_if: str, texts: dict, pings: dict, timestamp: float) -> dict:
    stations = parse_station_dump(texts.get("station", ""))
    originators = parse_batctl_o(texts.get("batctl_o", ""))
    neighbors = parse_batctl_n(texts.get("batctl_n", ""))
    mac_to_ip = parse_ip_neigh(texts.get("ip_neigh", ""))
    links = []
    for mac in sorted(set(stations) | set(originators) | set(neighbors)):
        ip = mac_to_ip.get(mac)
        entry = {"mac": mac, "peer": _node_for_ip(ip) or mac, "ip": ip}
        entry.update(stations.get(mac, {}))
        if mac in originators:
            entry.update(originators[mac])
        if mac in neighbors:
            entry.update(neighbor_last_seen_s=neighbors[mac], direct=True)
        links.append(entry)
    e2e = {}
    for name, output in pings.items():
        parsed = parse_ping(output)
        if parsed:
            e2e[name] = parsed
    return {
        "node": self_name,
        "mesh_if": mesh_if,
        "bat_if": bat_if,
        "ts": round(timestamp, 3),
        "links": links,
        "end_to_end": e2e,
        "bat0": parse_bat_stats(texts.get("bat_stats", "")),
    }


def collect_real(self_name: str, mesh_if: str, bat_if: str, ping_names: list[str]) -> dict:
    commands = {
        "station": ["iw", "dev", mesh_if, "station", "dump"],
        "batctl_o": ["batctl", "-m", bat_if, "o"],
        "batctl_n": ["batctl", "-m", bat_if, "n"],
        "ip_neigh": ["ip", "neigh", "show", "dev", bat_if],
        "bat_stats": ["ip", "-s", "link", "show", "dev", bat_if],
    }
    texts = {}
    pings = {}
    with ThreadPoolExecutor(max_workers=max(4, len(commands) + len(ping_names))) as executor:
        future_map = {executor.submit(_run, cmd): ("text", key) for key, cmd in commands.items()}
        for name in ping_names:
            ip = NODES.get(name, name)
            future_map[executor.submit(_run, ["ping", "-c", "2", "-W", "1", ip])] = ("ping", name)
        for future in as_completed(future_map):
            kind, key = future_map[future]
            if kind == "text":
                texts[key] = future.result()
            else:
                pings[key] = future.result()
    return build_snapshot(self_name, mesh_if, bat_if, texts, pings, time.time())


def collect_sample(self_name: str, mesh_if: str, bat_if: str, sample_dir: Path, ping_names: list[str]) -> dict:
    names = {
        "station": "station.txt",
        "batctl_o": "batctl_o.txt",
        "batctl_n": "batctl_n.txt",
        "ip_neigh": "ip_neigh.txt",
        "bat_stats": "bat_stats.txt",
    }
    texts = {key: (sample_dir / filename).read_text(errors="replace") if (sample_dir / filename).exists() else "" for key, filename in names.items()}
    pings = {}
    for name in ping_names:
        path = sample_dir / f"ping_{name}.txt"
        if path.exists():
            pings[name] = path.read_text(errors="replace")
    return build_snapshot(self_name, mesh_if, bat_if, texts, pings, time.time())


def send_udp(snapshot: dict, destination: str) -> None:
    host, port_text = destination.rsplit(":", 1)
    payload = json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")).encode()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(payload, (host, int(port_text)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self", dest="self_name", required=True, choices=sorted(NODES))
    parser.add_argument("--mesh-if", default="wlan1")
    parser.add_argument("--bat-if", default="bat0")
    parser.add_argument("--ping", nargs="*", default=list(NODES))
    parser.add_argument("--send", default="")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--sample", type=Path)
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be positive")
    while True:
        if args.sample:
            snapshot = collect_sample(args.self_name, args.mesh_if, args.bat_if, args.sample, args.ping)
        else:
            snapshot = collect_real(args.self_name, args.mesh_if, args.bat_if, args.ping)
        print(json.dumps(snapshot, ensure_ascii=False), flush=True)
        if args.send:
            send_udp(snapshot, args.send)
        if not args.loop:
            break
        time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
