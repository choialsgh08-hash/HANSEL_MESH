import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "monitor"))
import dashboard
from metrics_agent import build_snapshot, parse_local_macs
from route_state import selected_path, video_routes

MACS = {"head": "02:00:00:00:00:10", "node1": "02:00:00:00:00:11",
        "node2": "02:00:00:00:00:12", "base": "02:00:00:00:00:01"}


def reports(forward=("head", "node1", "node2", "base"),
            reverse=("base", "node2", "node1", "head")):
    latest = {name: {"recv": 100, "snap": {"node": name,
              "mesh_macs": [mac], "links": []}}
              for name, mac in MACS.items()}
    for path in (forward, reverse):
        destination = path[-1]
        for source, next_node in zip(path, path[1:]):
            latest[source]["snap"]["links"].append({
                "mac": MACS[destination], "peer": MACS[destination],
                "nexthop": MACS[next_node], "tq": 150, "last_seen_s": 0.5})
    return latest


class VideoRouteTests(unittest.TestCase):
    def test_relay_resolves_radio_mac_without_arp_names(self):
        route = selected_path(reports(), "head", "base", 101)
        self.assertEqual(route["status"], "complete")
        self.assertEqual(route["path"], ["head", "node1", "node2", "base"])
        self.assertEqual(route["hops"], 3)
        self.assertEqual(route["mode"], "relay")
        self.assertEqual(route["age_s"], 1.5)

    def test_direct(self):
        route = selected_path(reports(("head", "base")), "head", "base", 101)
        self.assertEqual((route["mode"], route["hops"]), ("direct", 1))

    def test_reverse_is_independently_reconstructed(self):
        routes = video_routes(reports(reverse=("base", "head")), 101)
        self.assertEqual(routes["forward"]["hops"], 3)
        self.assertEqual(routes["reverse"]["hops"], 1)

    def test_stale_intermediate_report_is_not_a_complete_route(self):
        latest = reports()
        latest["node1"]["recv"] = 80
        route = selected_path(latest, "head", "base", 101)
        self.assertEqual(route["status"], "stale")
        self.assertIsNone(route["hops"])
        self.assertIsNone(route["mode"])

    def test_aged_ogm_plus_receive_age_expires(self):
        latest = reports()
        latest["head"]["snap"]["links"][0]["last_seen_s"] = 11.5
        self.assertEqual(selected_path(latest, "head", "base", 101)["status"], "stale")

    def test_unknown_mac_is_partial(self):
        latest = reports()
        latest["head"]["snap"]["links"][0]["nexthop"] = "02:ff:ff:ff:ff:ff"
        self.assertEqual(selected_path(latest, "head", "base", 101)["status"], "partial")

    def test_missing_destination_report(self):
        latest = reports()
        del latest["base"]
        route = selected_path(latest, "head", "base", 101)
        self.assertNotEqual(route["status"], "complete")

    def test_loop_during_route_convergence(self):
        latest = reports()
        latest["node1"]["snap"]["links"][0]["nexthop"] = MACS["head"]
        self.assertEqual(selected_path(latest, "head", "base", 101)["status"], "loop")

    def test_duplicate_identity_is_not_guessed(self):
        latest = reports()
        latest["node2"]["snap"]["mesh_macs"].append(MACS["node1"])
        self.assertEqual(selected_path(latest, "head", "base", 101)["status"], "partial")

    def test_missing_route_age_is_not_complete(self):
        latest = reports()
        del latest["head"]["snap"]["links"][0]["last_seen_s"]
        self.assertEqual(selected_path(latest, "head", "base", 101)["status"], "partial")

    def test_local_interface_and_primary_macs(self):
        macs = parse_local_macs(
            "3: wlan1: link/ether 02:00:00:00:00:10 brd ff:ff:ff:ff:ff:ff",
            "[MainIF/MAC: wlan1/02:00:00:00:00:10 (bat0/aa:bb:cc:dd:ee:ff BATMAN_IV)]")
        self.assertEqual(macs, [MACS["head"]])

    def test_build_snapshot_keeps_own_identity(self):
        snap = build_snapshot("head", "wlan1",
                              {"local_link": "link/ether " + MACS["head"]}, {}, 100)
        self.assertEqual(snap["mesh_macs"], [MACS["head"]])

    def test_dashboard_api_contains_routes(self):
        with mock.patch.dict(dashboard.LATEST, reports(), clear=True):
            state = dashboard.merge_state(101)
        self.assertEqual(state["video_routes"]["forward"]["hops"], 3)


if __name__ == "__main__":
    unittest.main()
