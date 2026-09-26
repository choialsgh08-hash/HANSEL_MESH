import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from common.control_protocol import build_command
from robot.mesh_control_server import (
    DEFAULT_DRIVE_STATE_DIR,
    ControlServerCore,
    DriveLatchStore,
    parse_args,
    restore_persisted_drive_latch,
)


class RecordingController:
    def __init__(self, events=None, enable_result=True, enable_raises=False):
        self.events = events if events is not None else []
        self.enable_result = enable_result
        self.enable_raises = enable_raises
        self.drive_enabled = False

    def handle_command(self, command, message=None):
        self.events.append(f"controller:{command}")
        if command in {"relay_hold", "drive_disable"}:
            self.drive_enabled = False
            return True, "drive_disabled"
        if command == "drive_enable":
            if self.enable_raises:
                raise RuntimeError("simulated enable failure")
            if not self.enable_result:
                return False, "simulated_enable_rejection"
            self.drive_enabled = True
            return True, "drive_enabled"
        return True, "applied"


class RecordingStore:
    def __init__(self, events=None, initial=False, fail_values=()):
        self.events = events if events is not None else []
        self.drive_enabled = initial
        self.enable_pending = False
        self.fail_values = set(fail_values)

    def load_drive_enabled(self):
        return self.drive_enabled and not self.enable_pending

    def begin_drive_enable(self):
        self.events.append("store:begin")
        self.enable_pending = True

    def commit_drive_enable(self):
        self.events.append("store:commit")
        self.enable_pending = False

    def save_drive_enabled(self, drive_enabled):
        self.events.append(f"store:{drive_enabled}")
        if drive_enabled in self.fail_values:
            raise OSError("simulated persistence failure")
        self.drive_enabled = drive_enabled


class DriveLatchStoreTests(unittest.TestCase):
    def test_missing_state_means_attached_and_enabled(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            store = DriveLatchStore(temporary_dir, "node1")
            self.assertTrue(store.load_drive_enabled())

    def test_round_trip_uses_physical_pi_atomic_json(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            store = DriveLatchStore(temporary_dir, "node2")
            store.save_drive_enabled(False)

            self.assertFalse(store.load_drive_enabled())
            self.assertEqual(store.path.name, "drive-latch.json")
            state = json.loads(store.path.read_text(encoding="utf-8"))
            self.assertEqual(state["role"], "node2")
            self.assertFalse(state["drive_enabled"])
            self.assertEqual(
                list(Path(temporary_dir).glob("*.tmp")),
                [],
            )

            store.save_drive_enabled(True)
            self.assertTrue(store.load_drive_enabled())

    def test_unfinished_enable_transaction_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            store = DriveLatchStore(temporary_dir, "node1")
            store.save_drive_enabled(True)
            store.begin_drive_enable()

            self.assertFalse(store.load_drive_enabled())

    def test_role_change_cannot_bypass_a_disabled_physical_latch(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            node1_store = DriveLatchStore(temporary_dir, "node1")
            node1_store.save_drive_enabled(False)

            node2_store = DriveLatchStore(temporary_dir, "node2")
            self.assertFalse(node2_store.load_drive_enabled())

    def test_disabled_legacy_role_state_migrates_fail_closed(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            legacy_path = (
                Path(temporary_dir) / "drive-latch-node1.json"
            )
            legacy_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "role": "node1",
                        "drive_enabled": False,
                    }
                ),
                encoding="utf-8",
            )

            switched_store = DriveLatchStore(temporary_dir, "node2")
            self.assertFalse(switched_store.load_drive_enabled())

    def test_corrupt_state_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            store = DriveLatchStore(temporary_dir, "head")
            store.path.write_text("{not-json", encoding="utf-8")
            self.assertFalse(store.load_drive_enabled())

    def test_unreadable_state_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            store = DriveLatchStore(temporary_dir, "node3")
            store.path.mkdir()
            self.assertFalse(store.load_drive_enabled())


class PersistentControlCoreTests(unittest.TestCase):
    def setUp(self):
        self.peer = ("192.168.60.2", 42000)
        self.sender_clock = 1_000_000_000
        self.receiver_clock = 5_000_000_000

    def packet(self, seq, command):
        message = build_command(
            session_id="persistence-test",
            seq=seq,
            target="node1",
            command=command,
            ttl_ms=750,
            sent_monotonic_ns=self.sender_clock + seq * 1_000_000,
        )
        return json.dumps(message).encode("utf-8")

    def process(self, core, seq, command):
        return core.process_datagram(
            self.packet(seq, command),
            self.peer,
            now_monotonic_ns=self.receiver_clock + seq * 1_000_000,
        )

    def build_core(self, controller, store):
        core = ControlServerCore(
            controller=controller,
            role="node1",
            drive_latch_store=store,
        )
        initial_stop = self.process(core, 1, "stop")
        self.assertTrue(initial_stop.applied)
        controller.events.clear()
        store.events.clear()
        return core

    def test_disable_applies_hardware_before_persisting_false(self):
        events = []
        controller = RecordingController(events)
        store = RecordingStore(events, initial=True)
        core = self.build_core(controller, store)

        result = self.process(core, 2, "relay_hold")

        self.assertTrue(result.applied)
        self.assertEqual(
            events,
            ["controller:relay_hold", "store:False"],
        )
        self.assertFalse(controller.drive_enabled)
        self.assertFalse(store.drive_enabled)

    def test_disable_persistence_failure_is_rejected_but_hardware_stays_off(self):
        events = []
        controller = RecordingController(events)
        controller.drive_enabled = True
        store = RecordingStore(events, initial=True, fail_values=(False,))
        core = self.build_core(controller, store)

        result = self.process(core, 2, "drive_disable")

        self.assertFalse(result.applied)
        self.assertEqual(result.ack["status"], "rejected")
        self.assertEqual(
            result.ack["reason"],
            "drive_state_persist_failed",
        )
        self.assertFalse(controller.drive_enabled)
        self.assertEqual(
            events,
            ["controller:drive_disable", "store:False"],
        )

    def test_enable_persists_true_before_enabling_hardware(self):
        events = []
        controller = RecordingController(events)
        store = RecordingStore(events, initial=False)
        core = self.build_core(controller, store)

        result = self.process(core, 2, "drive_enable")

        self.assertTrue(result.applied)
        self.assertEqual(
            events,
            [
                "store:begin",
                "store:True",
                "controller:drive_enable",
                "store:commit",
            ],
        )
        self.assertTrue(controller.drive_enabled)
        self.assertTrue(store.drive_enabled)

    def test_enable_persistence_failure_never_reaches_hardware(self):
        events = []
        controller = RecordingController(events)
        store = RecordingStore(events, initial=False, fail_values=(True,))
        core = self.build_core(controller, store)

        result = self.process(core, 2, "drive_enable")

        self.assertFalse(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "drive_state_persist_failed",
        )
        self.assertEqual(
            events,
            [
                "store:begin",
                "store:True",
                "controller:relay_hold",
                "store:begin",
                "store:False",
            ],
        )
        self.assertFalse(controller.drive_enabled)
        self.assertFalse(store.drive_enabled)

    def test_post_replace_enable_failure_rolls_disk_back_to_false(self):
        events = []
        controller = RecordingController(events)
        with tempfile.TemporaryDirectory() as temporary_dir:
            store = DriveLatchStore(temporary_dir, "node1")
            store.save_drive_enabled(False)
            core = ControlServerCore(
                controller=controller,
                role="node1",
                drive_latch_store=store,
            )
            initial_stop = self.process(core, 1, "stop")
            self.assertTrue(initial_stop.applied)
            events.clear()

            with mock.patch.object(
                store,
                "_fsync_directory",
                side_effect=[
                    None,
                    OSError("simulated directory fsync failure"),
                    None,
                    None,
                ],
            ):
                result = self.process(core, 2, "drive_enable")

            self.assertFalse(result.applied)
            self.assertEqual(
                result.ack["reason"],
                "drive_state_persist_failed",
            )
            self.assertFalse(controller.drive_enabled)
            self.assertFalse(store.load_drive_enabled())

    def test_pending_marker_survives_double_persistence_failure(self):
        events = []
        controller = RecordingController(events)
        with tempfile.TemporaryDirectory() as temporary_dir:
            store = DriveLatchStore(temporary_dir, "node1")
            store.save_drive_enabled(False)
            core = ControlServerCore(
                controller=controller,
                role="node1",
                drive_latch_store=store,
            )
            initial_stop = self.process(core, 1, "stop")
            self.assertTrue(initial_stop.applied)
            original_save = store.save_drive_enabled

            def fail_false_before_replace(drive_enabled):
                if drive_enabled is False:
                    raise OSError("simulated rollback pre-write failure")
                return original_save(drive_enabled)

            with mock.patch.object(
                store,
                "_fsync_directory",
                side_effect=[
                    None,
                    OSError("simulated enabled-state fsync failure"),
                    None,
                ],
            ), mock.patch.object(
                store,
                "save_drive_enabled",
                side_effect=fail_false_before_replace,
            ):
                result = self.process(core, 2, "drive_enable")

            self.assertFalse(result.applied)
            self.assertFalse(controller.drive_enabled)
            self.assertTrue(store.enable_pending_path.exists())
            self.assertFalse(store.load_drive_enabled())

    def test_rejected_enable_rolls_back_hardware_and_persistence(self):
        events = []
        controller = RecordingController(events, enable_result=False)
        store = RecordingStore(events, initial=False)
        core = self.build_core(controller, store)

        result = self.process(core, 2, "drive_enable")

        self.assertFalse(result.applied)
        self.assertEqual(
            result.ack["reason"],
            "simulated_enable_rejection",
        )
        self.assertEqual(
            events,
            [
                "store:begin",
                "store:True",
                "controller:drive_enable",
                "controller:relay_hold",
                "store:begin",
                "store:False",
            ],
        )
        self.assertFalse(controller.drive_enabled)
        self.assertFalse(store.drive_enabled)

    def test_exception_during_enable_also_rolls_back(self):
        events = []
        controller = RecordingController(events, enable_raises=True)
        store = RecordingStore(events, initial=False)
        core = self.build_core(controller, store)

        result = self.process(core, 2, "drive_enable")

        self.assertFalse(result.applied)
        self.assertEqual(result.ack["reason"], "command_apply_failed")
        self.assertEqual(
            events,
            [
                "store:begin",
                "store:True",
                "controller:drive_enable",
                "controller:relay_hold",
                "store:begin",
                "store:False",
            ],
        )
        self.assertFalse(controller.drive_enabled)
        self.assertFalse(store.drive_enabled)


class StartupRestoreTests(unittest.TestCase):
    def test_disabled_state_is_restored_before_serving(self):
        events = []
        controller = RecordingController(events)
        store = RecordingStore(events, initial=False)

        self.assertTrue(
            restore_persisted_drive_latch(controller, "node1", store)
        )
        self.assertEqual(
            events,
            ["controller:relay_hold", "store:False"],
        )
        self.assertFalse(controller.drive_enabled)

    def test_enabled_or_missing_state_needs_no_hardware_action(self):
        events = []
        controller = RecordingController(events)
        store = RecordingStore(events, initial=True)

        self.assertTrue(
            restore_persisted_drive_latch(controller, "node1", store)
        )
        self.assertEqual(events, [])

    def test_cli_defaults_to_production_state_directory(self):
        args = parse_args(["--role", "node1"])
        self.assertEqual(args.drive_state_dir, DEFAULT_DRIVE_STATE_DIR)
        self.assertFalse(args.unsafe_no_drive_state_persistence)


if __name__ == "__main__":
    unittest.main()
