import unittest
from unittest import mock

from monitor import metrics_agent


class MetricsInterfaceTests(unittest.TestCase):
    def test_collect_real_uses_configured_batman_interface(self) -> None:
        commands = []

        def fake_run(command):
            commands.append(command)
            return ""

        with mock.patch.object(metrics_agent, "_run", side_effect=fake_run):
            snapshot = metrics_agent.collect_real(
                "head",
                "mesh9",
                "bat9",
                [],
                123.0,
            )

        self.assertIn(
            ["ip", "neigh", "show", "dev", "bat9"],
            commands,
        )
        self.assertIn(["batctl", "-m", "bat9", "o"], commands)
        self.assertIn(["batctl", "-m", "bat9", "n"], commands)
        self.assertEqual(snapshot["mesh_if"], "mesh9")
        self.assertEqual(snapshot["bat_if"], "bat9")

    def test_collect_real_uses_bounded_parallel_ping_commands(self) -> None:
        commands = []

        def fake_run(command):
            commands.append(command)
            return ""

        with mock.patch.object(metrics_agent, "_run", side_effect=fake_run):
            metrics_agent.collect_real(
                "head",
                "wlan0",
                "bat0",
                ["base", "node1", "unknown"],
                123.0,
            )

        self.assertIn(
            ["ping", "-c", "3", "-W", "1", "192.168.50.1"],
            commands,
        )
        self.assertIn(
            ["ping", "-c", "3", "-W", "1", "192.168.50.11"],
            commands,
        )
        self.assertFalse(
            any(command[-1] == "unknown" for command in commands)
        )

    def test_collect_real_reads_bat0_stats(self) -> None:
        commands = []

        with mock.patch.object(
            metrics_agent, "_run", side_effect=lambda c: commands.append(c) or ""
        ):
            metrics_agent.collect_real("head", "wlan1", "bat0", [], 1.0)

        self.assertIn(["ip", "-s", "link", "show", "dev", "bat0"], commands)


class StationParseTests(unittest.TestCase):
    STATION = (
        "Station 02:aa:bb:00:00:01 (on wlan1)\n"
        "\tinactive time:\t120 ms\n"
        "\ttx retries:\t12\n"
        "\ttx failed:\t3\n"
        "\tsignal:  \t-52 dBm\n"
        "\ttx bitrate:\t54.0 MBit/s\n"
        "\texpected throughput:\t20.000Mbps\n"
    )

    def test_parses_retries_failed_and_expected_throughput(self) -> None:
        stations = metrics_agent.parse_station_dump(self.STATION)
        entry = stations["02:aa:bb:00:00:01"]
        self.assertEqual(entry["tx_retries"], 12)
        self.assertEqual(entry["tx_failed"], 3)
        self.assertEqual(entry["expected_mbps"], 20.0)
        self.assertEqual(entry["inactive_ms"], 120)


class BatStatsParseTests(unittest.TestCase):
    TEXT = (
        "5: bat0: <UP> mtu 1468 qdisc noqueue state UNKNOWN\n"
        "    link/ether 02:aa:bb:00:00:10 brd ff:ff:ff:ff:ff:ff\n"
        "    RX: bytes  packets  errors  dropped overrun mcast\n"
        "    10485760   82000    0       15      0       120\n"
        "    TX: bytes  packets  errors  dropped carrier collsns\n"
        "    5242880    41000    0       7       0       0\n"
    )

    def test_parses_rx_tx_dropped(self) -> None:
        stats = metrics_agent.parse_bat_stats(self.TEXT)
        self.assertEqual(stats["rx_bytes"], 10485760)
        self.assertEqual(stats["rx_packets"], 82000)
        self.assertEqual(stats["rx_dropped"], 15)
        self.assertEqual(stats["tx_dropped"], 7)

    def test_empty_input_returns_empty(self) -> None:
        self.assertEqual(metrics_agent.parse_bat_stats(""), {})


class DetectEventsTests(unittest.TestCase):
    def _snap(self, links):
        return {"node": "head", "links": links}

    def test_route_change_detected(self) -> None:
        prev = self._snap([
            {"mac": "aa", "peer": "base", "nexthop": "bb"},
        ])
        curr = self._snap([
            {"mac": "aa", "peer": "base", "nexthop": "cc"},
        ])
        events = metrics_agent.detect_events(prev, curr)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["type"], "route_changed")
        self.assertEqual(events[0]["from"], "bb")
        self.assertEqual(events[0]["to"], "cc")
        self.assertEqual(events[0]["peer"], "base")

    def test_neighbor_lost_and_gained(self) -> None:
        prev = self._snap([
            {"mac": "aa", "peer": "node1", "direct": True},
            {"mac": "bb", "peer": "node2", "direct": True},
        ])
        curr = self._snap([
            {"mac": "bb", "peer": "node2", "direct": True},
            {"mac": "cc", "peer": "node3", "direct": True},
        ])
        events = metrics_agent.detect_events(prev, curr)
        types = {(e["type"], e["peer"]) for e in events}
        self.assertIn(("neighbor_lost", "node1"), types)
        self.assertIn(("neighbor_gained", "node3"), types)

    def test_no_change_yields_no_events(self) -> None:
        snap = self._snap([
            {"mac": "aa", "peer": "base", "nexthop": "bb", "direct": True},
        ])
        self.assertEqual(metrics_agent.detect_events(snap, dict(snap)), [])


if __name__ == "__main__":
    unittest.main()
