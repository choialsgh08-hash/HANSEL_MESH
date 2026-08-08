from copy import deepcopy


class PathSelectionHysteresis:
    def __init__(self, confirm_frames: int = 3):
        if isinstance(confirm_frames, bool) or not isinstance(confirm_frames, int):
            raise TypeError("confirm_frames must be an integer")
        if confirm_frames < 1:
            raise ValueError("confirm_frames must be positive")
        self._confirm_frames = confirm_frames
        self.reset()

    def reset(self) -> None:
        self._selected = None
        self._selected_since_ns = None
        self._pending = None
        self._pending_frames = 0
        self._accepted_count = 0
        self._blocked_overlay = None

    @staticmethod
    def _same_route(left, right) -> bool:
        if left is None or right is None:
            return False
        left_stopped = left.get("status") == "blocked_near_field"
        right_stopped = right.get("status") == "blocked_near_field"
        return left_stopped == right_stopped and abs(
            float(left.get("curvature_per_m", 0.0))
            - float(right.get("curvature_per_m", 0.0))
        ) <= 1e-9

    def _result(self, status: str, switch_accepted: bool) -> dict:
        result = {
            "path": deepcopy(self._selected),
            "status": status,
            "selected_since_ns": self._selected_since_ns,
            "pending_switch_frames": self._pending_frames,
            "switch_accepted": switch_accepted,
            "blocked_previous_path": deepcopy(self._blocked_overlay),
        }
        return result

    def select(
        self,
        candidate,
        current_blocked,
        blocked_segments,
        now_ns,
    ) -> dict:
        candidate = deepcopy(candidate)
        if self._selected is None:
            self._selected = candidate
            self._selected_since_ns = now_ns
            return self._result("initialized", False)

        repeated_stop = (
            current_blocked
            and self._selected.get("status") == "blocked_near_field"
            and self._same_route(self._selected, candidate)
        )
        if repeated_stop:
            self._pending = None
            self._pending_frames = 0
            return self._result("stable", False)

        if current_blocked:
            old_path = deepcopy(self._selected)
            self._selected = candidate
            self._selected_since_ns = now_ns
            self._pending = None
            self._pending_frames = 0
            self._accepted_count += 1
            self._blocked_overlay = {
                "points": old_path.get("points", []),
                "segments": deepcopy(blocked_segments),
            }
            return self._result("route_changed_blocked", True)

        if self._same_route(self._selected, candidate):
            self._selected = candidate
            self._pending = None
            self._pending_frames = 0
            return self._result("stable", False)

        if self._same_route(self._pending, candidate):
            self._pending_frames += 1
        else:
            self._pending = candidate
            self._pending_frames = 1

        if self._pending_frames >= self._confirm_frames:
            self._selected = candidate
            self._selected_since_ns = now_ns
            self._pending = None
            self._pending_frames = 0
            self._accepted_count += 1
            self._blocked_overlay = None
            return self._result("route_changed_confirmed", True)

        return self._result("holding_candidate", False)

    def diagnostics(self) -> dict:
        return {
            "path_switch_pending_count": self._pending_frames,
            "path_switch_accepted_count": self._accepted_count,
        }
