import threading
import time
import unittest

from controller.quality_supervisor import (
    AsyncQualitySupervisor,
    QualityConfig,
    QualityDecision,
)


def decision(status: str, reason: str) -> QualityDecision:
    return QualityDecision(
        status=status,
        raw_status=status,
        speed_cap=None,
        camera_profile=0,
        reasons=[reason],
        video={},
        network={},
    )


def wait_until(predicate, timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


class SlowSupervisor:
    def __init__(self) -> None:
        self.config = QualityConfig(interval=0.01)
        self.last_decision = decision("UNKNOWN", "initial")
        self.entered = threading.Event()
        self.release = threading.Event()

    def update(self) -> QualityDecision:
        self.entered.set()
        self.release.wait(2.0)
        return decision("GOOD", "updated")


class ControlledSequenceSupervisor:
    def __init__(self) -> None:
        self.config = QualityConfig(interval=0.01)
        self.last_decision = decision("UNKNOWN", "initial")
        self.calls = 0
        self.recovery_entered = threading.Event()
        self.allow_recovery = threading.Event()

    def update(self) -> QualityDecision:
        self.calls += 1
        if self.calls == 1:
            return decision("GOOD", "first success")
        if self.calls == 2:
            raise RuntimeError("probe failed")
        self.recovery_entered.set()
        self.allow_recovery.wait(1.0)
        return decision("WARN", "recovered")


class OneSuccessSupervisor:
    def __init__(self) -> None:
        self.config = QualityConfig(interval=1.0, async_stale_s=0.03)
        self.last_decision = decision("UNKNOWN", "initial")

    def update(self) -> QualityDecision:
        return decision("GOOD", "fresh")


class AsyncQualitySupervisorTests(unittest.TestCase):
    def test_latest_does_not_wait_for_slow_update(self) -> None:
        slow = SlowSupervisor()
        async_supervisor = AsyncQualitySupervisor(slow)
        async_supervisor.start()
        self.assertTrue(slow.entered.wait(0.5))

        try:
            started = time.monotonic()
            snapshot = async_supervisor.latest()
            elapsed = time.monotonic() - started

            self.assertLess(elapsed, 0.1)
            self.assertEqual(snapshot.status, "NOT_READY")
            self.assertEqual(snapshot.speed_cap, 0.0)
            self.assertEqual(snapshot.camera_profile, 2)
            self.assertFalse(async_supervisor.stop(timeout=0.01))
        finally:
            slow.release.set()
            self.assertTrue(async_supervisor.stop(timeout=0.5))

    def test_error_preserves_last_successful_decision_and_reports_reason(self) -> None:
        sequence = ControlledSequenceSupervisor()
        async_supervisor = AsyncQualitySupervisor(sequence)
        async_supervisor.start()
        try:
            self.assertTrue(sequence.recovery_entered.wait(0.5))
            self.assertTrue(
                wait_until(
                    lambda: (
                        async_supervisor.latest().status == "ERROR"
                        and async_supervisor.last_error is not None
                    )
                )
            )
            snapshot = async_supervisor.latest()
            self.assertEqual(snapshot.status, "ERROR")
            self.assertEqual(snapshot.speed_cap, 0.0)
            self.assertEqual(snapshot.camera_profile, 2)
            self.assertIn("first success", snapshot.reasons)
            self.assertTrue(
                any("RuntimeError: probe failed" in reason for reason in snapshot.reasons)
            )

            sequence.allow_recovery.set()
            self.assertTrue(
                wait_until(lambda: async_supervisor.latest().status == "WARN")
            )
            self.assertIsNone(async_supervisor.last_error)
        finally:
            sequence.allow_recovery.set()
            self.assertTrue(async_supervisor.stop(timeout=0.5))

    def test_successful_update_becomes_fail_closed_when_stale(self) -> None:
        one_success = OneSuccessSupervisor()
        async_supervisor = AsyncQualitySupervisor(one_success)
        async_supervisor.start()
        try:
            self.assertTrue(
                wait_until(lambda: async_supervisor.latest().status == "GOOD")
            )
            self.assertTrue(
                wait_until(
                    lambda: async_supervisor.latest().status == "STALE",
                    timeout=0.5,
                )
            )
            snapshot = async_supervisor.latest()
            self.assertEqual(snapshot.speed_cap, 0.0)
            self.assertEqual(snapshot.camera_profile, 2)
            self.assertTrue(
                any("quality update stale" in reason for reason in snapshot.reasons)
            )
        finally:
            self.assertTrue(async_supervisor.stop(timeout=0.5))

    def test_context_manager_starts_and_stops_worker(self) -> None:
        slow = SlowSupervisor()
        with AsyncQualitySupervisor(slow) as async_supervisor:
            self.assertTrue(slow.entered.wait(0.5))
            self.assertTrue(async_supervisor.is_running)
            slow.release.set()

        self.assertFalse(async_supervisor.is_running)


if __name__ == "__main__":
    unittest.main()
