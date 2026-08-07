from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "monitor" / "metrics_agent.py"
SPEC = importlib.util.spec_from_file_location("hansel_mesh_metrics_agent", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
metrics_agent = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(metrics_agent)


def test_snapshot_contract_and_parsers():
    texts = {
        "station": (
            "Station 02:11:22:33:44:55 (on wlan1)\n"
            "\tsignal: -48 dBm\n"
            "\ttx bitrate: 24.0 MBit/s\n"
        ),
        "batctl_o": (
            " * 02:11:22:33:44:55 0.520s (245) "
            "02:11:22:33:44:55 [ wlan1]\n"
        ),
        "batctl_n": "wlan1 02:11:22:33:44:55 0.120s\n",
        "ip_neigh": (
            "192.168.50.10 dev bat0 lladdr "
            "02:11:22:33:44:55 REACHABLE\n"
        ),
        "bat_stats": (
            "RX: bytes packets errors dropped overrun mcast\n"
            "1000 10 0 1 0 0\n"
            "TX: bytes packets errors dropped carrier collsns\n"
            "2000 20 0 2 0 0\n"
        ),
    }
    pings = {
        "head": (
            "2 packets transmitted, 2 received, 0% packet loss\n"
            "rtt min/avg/max/mdev = 1.000/2.500/4.000/1.500 ms\n"
        )
    }

    snapshot = metrics_agent.build_snapshot(
        "base", "wlan1", "bat0", texts, pings, 123.456
    )

    assert {
        "node", "mesh_if", "bat_if", "ts", "links", "end_to_end", "bat0"
    } <= set(snapshot)
    assert snapshot["node"] == "base"
    assert snapshot["links"][0]["peer"] == "head"
    assert snapshot["links"][0]["tq"] == 245
    assert snapshot["end_to_end"]["head"]["rtt_avg_ms"] == 2.5
    assert snapshot["bat0"]["tx_dropped"] == 2
