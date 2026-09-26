import os
import sys
import unittest

# dashboard.py imports metrics_agent as a top-level module (it adds monitor/ to
# sys.path at runtime after that import), so mirror how it is actually launched.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "monitor"))
import dashboard  # noqa: E402


class DashboardEventTests(unittest.TestCase):
    def setUp(self) -> None:
        dashboard.EVENTS.clear()
        dashboard.LATEST.clear()

    def _ingest_event(self, event, now):
        dashboard.ingest(
            {"node": "node1", "links": [], "end_to_end": {}, "events": [event]},
            now,
        )

    def test_event_activates_reconnect_banner(self) -> None:
        now = 1000.0
        self._ingest_event(
            {"type": "route_changed", "peer": "node2", "to": "bb"}, now
        )
        state = dashboard.merge_state(now)
        self.assertTrue(state["reconnect_active"])
        self.assertEqual(state["reconnect_latest"]["type"], "route_changed")
        self.assertEqual(len(state["events"]), 1)
        self.assertIn("time", state["events"][0])

    def test_banner_clears_after_window_but_log_keeps_event(self) -> None:
        now = 1000.0
        self._ingest_event({"type": "neighbor_lost", "peer": "node2"}, now)
        later = now + dashboard.RECONNECT_WINDOW_S + 1
        state = dashboard.merge_state(later)
        self.assertFalse(state["reconnect_active"])
        self.assertEqual(len(state["events"]), 1)  # still visible in the log

    def test_snapshot_without_events_keeps_banner_off(self) -> None:
        dashboard.ingest({"node": "node1", "links": [], "end_to_end": {}}, 1000.0)
        state = dashboard.merge_state(1000.0)
        self.assertFalse(state["reconnect_active"])
        self.assertEqual(state["events"], [])


if __name__ == "__main__":
    unittest.main()
