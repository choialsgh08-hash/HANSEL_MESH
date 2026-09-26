import unittest

from monitor.radar_path import PathSelectionHysteresis


def path(curvature, lateral=0.0):
    return {
        "status": "clear",
        "curvature_per_m": curvature,
        "resolution_m": 0.01,
        "length_m": 0.02,
        "points": [[0.0, 0.0], [0.01, lateral], [0.02, lateral]],
    }


class PathSelectionHysteresisTests(unittest.TestCase):
    def test_unblocked_route_switches_after_three_equal_candidates(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        initial = selector.select(path(0.0), False, [], 1_000_000_000)
        self.assertFalse(initial["switch_accepted"])
        self.assertEqual(selector.diagnostics(), {
            "path_switch_pending_count": 0,
            "path_switch_accepted_count": 0,
        })
        first = selector.select(path(1.0, 0.01), False, [], 1_125_000_000)
        second = selector.select(path(1.0, 0.01), False, [], 1_250_000_000)
        third = selector.select(path(1.0, 0.01), False, [], 1_375_000_000)
        self.assertEqual(first["path"]["curvature_per_m"], 0.0)
        self.assertEqual(second["pending_switch_frames"], 2)
        self.assertEqual(third["path"]["curvature_per_m"], 1.0)
        self.assertTrue(third["switch_accepted"])
        self.assertEqual(third["pending_switch_frames"], 0)
        self.assertEqual(selector.diagnostics(), {
            "path_switch_pending_count": 0,
            "path_switch_accepted_count": 1,
        })

    def test_pending_switch_resets_when_candidate_changes(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        selector.select(path(1.0), False, [], 1_125_000_000)
        result = selector.select(path(-1.0), False, [], 1_250_000_000)
        self.assertEqual(result["pending_switch_frames"], 1)
        self.assertEqual(result["path"]["curvature_per_m"], 0.0)

    def test_return_to_selected_curvature_breaks_alternate_consecutive_run(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        selector.select(path(1.0), False, [], 1_125_000_000)
        returned = selector.select(path(0.0), False, [], 1_250_000_000)
        self.assertEqual(returned["pending_switch_frames"], 0)
        selector.select(path(1.0), False, [], 1_375_000_000)
        result = selector.select(path(1.0), False, [], 1_500_000_000)
        self.assertEqual(result["pending_switch_frames"], 2)
        self.assertEqual(result["path"]["curvature_per_m"], 0.0)

    def test_blocked_selected_path_switches_immediately_and_keeps_blocked_overlay(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        result = selector.select(
            path(1.0), True,
            [{"start_index": 1, "end_index": 2, "reason": "confirmed_obstacle"}],
            1_125_000_000,
        )
        self.assertEqual(result["path"]["curvature_per_m"], 1.0)
        self.assertEqual(result["status"], "route_changed_blocked")
        self.assertEqual(result["blocked_previous_path"]["points"][1], [0.01, 0.0])

    def test_blocked_path_preserves_candidate_geometry_status(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        limited = dict(path(1.0), status="limited")
        result = selector.select(
            limited, True,
            [{"start_index": 1, "end_index": 2, "reason": "confirmed_obstacle"}],
            1_125_000_000,
        )
        self.assertEqual(result["status"], "route_changed_blocked")
        self.assertEqual(result["path"]["status"], "limited")
        self.assertIsNotNone(result["blocked_previous_path"])

    def test_blocked_overlay_is_retained_while_new_route_stays_selected(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        changed = selector.select(
            path(1.0), True,
            [{"start_index": 1, "end_index": 2, "reason": "confirmed_obstacle"}],
            1_125_000_000,
        )

        held = selector.select(path(1.0, 0.01), False, [], 1_250_000_000)

        self.assertEqual(
            held["blocked_previous_path"], changed["blocked_previous_path"],
        )

    def test_repeated_near_field_stop_is_idempotent_and_resume_needs_three_frames(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        clear = path(0.0)
        selector.select(clear, False, [], 1_000_000_000)
        stopped = dict(
            path(0.0), status="blocked_near_field",
            points=[[0.0, 0.0]], length_m=0.0,
        )
        first = selector.select(
            stopped, True,
            [{"start_index": 0, "end_index": 1, "reason": "near_field_overlap"}],
            1_125_000_000,
        )
        held = selector.select(
            stopped, True,
            [{"start_index": 0, "end_index": 0, "reason": "near_field_overlap"}],
            1_250_000_000,
        )
        self.assertTrue(first["switch_accepted"])
        self.assertFalse(held["switch_accepted"])
        self.assertEqual(selector.diagnostics()["path_switch_accepted_count"], 1)
        self.assertEqual(held["path"]["status"], "blocked_near_field")
        self.assertEqual(held["blocked_previous_path"], first["blocked_previous_path"])
        one = selector.select(clear, False, [], 1_375_000_000)
        two = selector.select(clear, False, [], 1_500_000_000)
        three = selector.select(clear, False, [], 1_625_000_000)
        self.assertEqual(one["path"]["status"], "blocked_near_field")
        self.assertEqual(two["pending_switch_frames"], 2)
        self.assertEqual(three["path"]["status"], "clear")

    def test_repeated_stop_refreshes_guidance_without_counting_a_route_switch(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        stopped_straight = dict(
            path(0.0),
            status="blocked_near_field",
            points=[[0.0, 0.0]],
            length_m=0.0,
            goal_angle_deg=0.0,
            goal_lateral_m=0.0,
        )
        first = selector.select(
            stopped_straight,
            True,
            [{"start_index": 0, "end_index": 0, "reason": "near_field_overlap"}],
            1_125_000_000,
        )
        stopped_right = dict(
            stopped_straight,
            goal_angle_deg=55.0,
            goal_lateral_m=0.41,
            goal_selection_mode="reflectivity_corridor_guided",
        )

        refreshed = selector.select(
            stopped_right,
            True,
            [{"start_index": 0, "end_index": 0, "reason": "near_field_overlap"}],
            1_250_000_000,
        )

        self.assertEqual(refreshed["path"]["goal_angle_deg"], 55.0)
        self.assertEqual(refreshed["path"]["goal_lateral_m"], 0.41)
        self.assertFalse(refreshed["switch_accepted"])
        self.assertEqual(
            refreshed["selected_since_ns"], first["selected_since_ns"]
        )
        self.assertEqual(
            selector.diagnostics()["path_switch_accepted_count"], 1
        )
        self.assertEqual(
            refreshed["blocked_previous_path"], first["blocked_previous_path"]
        )

    def test_unresolved_turn_withholds_straight_immediately_and_clear_needs_confirmation(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        clear = path(0.0)
        selector.select(clear, False, [], 1_000_000_000)
        unresolved = dict(
            clear,
            status="turn_side_unresolved",
            points=[[0.0, 0.0]],
            length_m=0.0,
            turn_candidates=[{"direction": "left"}, {"direction": "right"}],
        )
        blocked = [{
            "start_index": 0,
            "end_index": 2,
            "reason": "turn_side_unresolved",
        }]

        first = selector.select(
            unresolved, True, blocked, 1_125_000_000,
        )
        held = selector.select(
            unresolved, True, blocked, 1_250_000_000,
        )
        clear_one = selector.select(clear, False, [], 1_375_000_000)
        clear_two = selector.select(clear, False, [], 1_500_000_000)
        clear_three = selector.select(clear, False, [], 1_625_000_000)

        self.assertEqual(first["path"]["status"], "turn_side_unresolved")
        self.assertTrue(first["switch_accepted"])
        self.assertEqual(held["path"]["status"], "turn_side_unresolved")
        self.assertFalse(held["switch_accepted"])
        self.assertEqual(clear_one["path"]["status"], "turn_side_unresolved")
        self.assertEqual(clear_two["pending_switch_frames"], 2)
        self.assertEqual(clear_three["path"]["status"], "clear")

    def test_reset_clears_selected_pending_overlay_and_epoch_counters(self):
        selector = PathSelectionHysteresis(confirm_frames=3)
        selector.select(path(0.0), False, [], 1_000_000_000)
        selector.select(path(1.0), False, [], 1_125_000_000)
        self.assertEqual(selector.diagnostics()["path_switch_pending_count"], 1)
        selector.reset()
        self.assertEqual(selector.diagnostics(), {
            "path_switch_pending_count": 0,
            "path_switch_accepted_count": 0,
        })
        fresh = selector.select(path(-1.0), False, [], 1_250_000_000)
        self.assertFalse(fresh["switch_accepted"])
        self.assertEqual(fresh["pending_switch_frames"], 0)
        self.assertIsNone(fresh.get("blocked_previous_path"))


if __name__ == "__main__":
    unittest.main()
