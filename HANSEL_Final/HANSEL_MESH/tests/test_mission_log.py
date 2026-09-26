from dataclasses import replace
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from common.sensor_contract import (
    DropEvent,
    RadarFrame,
    SensorHeader,
    SensorHealth,
    WheelState,
)
from sensors.mission_log import (
    MissionLogError,
    MissionLogWriter,
    encode_log_entry,
    inspect_mission_log,
    iter_mission_log,
    iter_replay,
)


def wheel(seq, monotonic_ns=None):
    return WheelState(
        header=SensorHeader(
            mission_id="mission-1",
            unit_id="head",
            boot_id="boot-1",
            producer_id="producer-1",
            stream_id="wheel/drive",
            seq=seq,
            monotonic_ns=(
                seq * 100_000_000 if monotonic_ns is None else monotonic_ns
            ),
            frame_id="base_link",
        ),
        left_ticks=seq * 2,
        right_ticks=seq * 3,
        sample_period_ns=100_000_000,
    )


class MissionLogTests(unittest.TestCase):
    def test_writer_reader_and_inspector_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                self.assertTrue(writer.submit(wheel(1)))
                self.assertTrue(writer.submit(wheel(2)))
                critical = DropEvent(
                    header=SensorHeader(
                        mission_id="mission-1",
                        unit_id="head",
                        boot_id="boot-1",
                        producer_id="producer-1",
                        stream_id="drop/events",
                        seq=1,
                        monotonic_ns=250_000_000,
                    ),
                    event_id="drop-1",
                    released_unit_id="node1",
                    actuator_unit_id="head",
                    phase="requested",
                )
                self.assertEqual(writer.write_critical(critical), 3)

            entries = list(iter_mission_log(path))
            self.assertEqual([entry.log_seq for entry in entries], [1, 2, 3])
            self.assertEqual(entries[0].record, wheel(1))
            report = inspect_mission_log(path)
            self.assertEqual(report["records"], 3)
            self.assertEqual(report["record_counts"]["wheel_state"], 2)
            self.assertTrue(report["healthy"])

    def test_critical_waiter_reencodes_sequence_after_normal_submitter(self):
        critical = DropEvent(
            header=SensorHeader(
                mission_id="mission-1",
                unit_id="head",
                boot_id="boot-1",
                producer_id="producer-1",
                stream_id="drop/events",
                seq=1,
                monotonic_ns=250_000_000,
            ),
            event_id="drop-1",
            released_unit_id="node1",
            actuator_unit_id="head",
            phase="requested",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                original_fits = writer._fits
                original_wait = writer._condition.wait
                injected = {"done": False}

                def fits_after_injection(data_size):
                    if not injected["done"]:
                        return False
                    return original_fits(data_size)

                def inject_normal_submit(_timeout=None):
                    if injected["done"]:
                        return original_wait(_timeout)
                    normal = wheel(1)
                    writer._enqueue(
                        writer._next_log_seq,
                        encode_log_entry(writer._next_log_seq, normal),
                        critical=False,
                    )
                    injected["done"] = True
                    return True

                with mock.patch.object(
                    writer,
                    "_fits",
                    side_effect=fits_after_injection,
                ), mock.patch.object(
                    writer._condition,
                    "wait",
                    side_effect=inject_normal_submit,
                ):
                    self.assertEqual(writer.write_critical(critical), 2)

            entries = list(iter_mission_log(path))
            self.assertEqual([entry.log_seq for entry in entries], [1, 2])

    def test_writer_never_overwrites_existing_log_by_default(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            path.write_bytes(b"existing")
            with self.assertRaises(FileExistsError):
                MissionLogWriter(path)
            self.assertEqual(path.read_bytes(), b"existing")

    def test_queue_limits_reject_new_normal_record_visibly(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            writer = MissionLogWriter(
                path,
                max_queue_records=1,
                max_queue_bytes=65536,
                max_record_bytes=65536,
            )
            try:
                with writer._condition:
                    self.assertTrue(writer.submit(wheel(1)))
                    self.assertFalse(writer.submit(wheel(2)))
                self.assertEqual(writer.stats().dropped_records, 1)
            finally:
                writer.close()

    def test_truncated_last_line_can_be_recovered_but_middle_corruption_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                writer.submit(wheel(1))
                writer.submit(wheel(2))
            with path.open("ab") as handle:
                handle.write(b'{"log_version":1')
            self.assertEqual(len(list(iter_mission_log(path))), 2)
            report = inspect_mission_log(path)
            self.assertTrue(report["trailing_partial_recovered"])
            self.assertGreater(report["trailing_partial_bytes"], 0)
            self.assertFalse(report["healthy"])
            with self.assertRaises(MissionLogError):
                list(iter_mission_log(path, recover_trailing_partial=False))

            lines = path.read_bytes().splitlines(keepends=True)
            path.write_bytes(lines[0] + b"{bad json}\n" + lines[1])
            with self.assertRaisesRegex(MissionLogError, ":2:"):
                list(iter_mission_log(path))

    def test_log_sequence_gap_is_a_corruption(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                writer.submit(wheel(1))
                writer.submit(wheel(2))
            raw = path.read_text(encoding="utf-8").replace(
                '"log_seq":2',
                '"log_seq":3',
                1,
            )
            with path.open("w", encoding="utf-8", newline="\n") as handle:
                handle.write(raw)
            with self.assertRaisesRegex(MissionLogError, "expected log_seq 2"):
                list(iter_mission_log(path))

    def test_inspector_reports_stream_gap_and_time_regression(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                writer.submit(wheel(1, 200))
                writer.submit(wheel(3, 100))
            report = inspect_mission_log(path)
            self.assertEqual(report["sequence_gaps"], 1)
            self.assertEqual(report["monotonic_regressions"], 1)
            self.assertFalse(report["healthy"])

    def test_first_stream_sequence_must_start_at_one(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                writer.submit(wheel(5))
            report = inspect_mission_log(path)
            self.assertEqual(report["sequence_gaps"], 4)
            self.assertFalse(report["healthy"])

    def test_radar_quality_fields_cannot_hide_an_unusable_frame(self):
        radar = RadarFrame(
            header=SensorHeader(
                mission_id="mission-1",
                unit_id="head",
                boot_id="boot-1",
                producer_id="radar-producer-1",
                stream_id="radar/front",
                seq=1,
                monotonic_ns=100,
            ),
            frame_number=5,
            subframe_number=0,
            complete=False,
            dropped_frames_since_previous=4,
            points=(),
            frame_transition="gap",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                writer.submit(radar)
            report = inspect_mission_log(path)
            self.assertEqual(report["radar_incomplete_frames"], 1)
            self.assertEqual(report["radar_declared_drops"], 4)
            self.assertEqual(report["radar_discontinuity_frames"], 1)
            self.assertFalse(report["healthy"])

    def test_writer_and_inspector_reject_mixed_missions(self):
        first = wheel(1)
        second = replace(
            wheel(1),
            header=replace(
                wheel(1).header,
                mission_id="mission-2",
                producer_id="producer-2",
            ),
        )
        with tempfile.TemporaryDirectory() as directory:
            writer_path = Path(directory) / "writer.jsonl"
            with MissionLogWriter(writer_path) as writer:
                self.assertTrue(writer.submit(first))
                with self.assertRaisesRegex(ValueError, "cannot mix"):
                    writer.submit(second)

            mixed_path = Path(directory) / "mixed.jsonl"
            mixed_path.write_bytes(
                encode_log_entry(1, first)
                + b"\n"
                + encode_log_entry(2, second)
                + b"\n"
            )
            report = inspect_mission_log(mixed_path)
            self.assertTrue(report["mixed_missions"])
            self.assertFalse(report["healthy"])

    def test_rejected_first_record_does_not_bind_writer_mission(self):
        oversized = DropEvent(
            header=SensorHeader(
                mission_id="mission-1",
                unit_id="head",
                boot_id="boot-1",
                producer_id="producer-1",
                stream_id="drop/events",
                seq=1,
                monotonic_ns=100,
            ),
            event_id="drop-1",
            released_unit_id="node1",
            actuator_unit_id="head",
            phase="failed",
            reason="x" * 512,
        )
        accepted = replace(
            wheel(1),
            header=replace(
                wheel(1).header,
                mission_id="mission-2",
                producer_id="producer-2",
            ),
        )
        accepted_size = len(encode_log_entry(1, accepted))
        oversized_size = len(encode_log_entry(1, oversized))
        self.assertGreater(oversized_size, accepted_size)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(
                path,
                max_record_bytes=accepted_size,
                max_queue_bytes=oversized_size + 1024,
            ) as writer:
                with self.assertRaisesRegex(ValueError, "exceeds"):
                    writer.submit(oversized)
                self.assertTrue(writer.submit(accepted))
            report = inspect_mission_log(path)
            self.assertEqual(report["mission_ids"], ["mission-2"])
            self.assertTrue(report["healthy"])

    def test_replay_speed_uses_recorded_monotonic_delta(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                writer.submit(wheel(1, 1_000_000_000))
                writer.submit(wheel(2, 1_500_000_000))
            sleeps = []
            entries = list(iter_replay(path, speed=2.0, sleep=sleeps.append))
            self.assertEqual(len(entries), 2)
            self.assertEqual(sleeps, [0.25])
            no_sleeps = mock.Mock()
            list(iter_replay(path, speed=0, sleep=no_sleeps))
            no_sleeps.assert_not_called()

    def test_replay_cursor_does_not_overcount_cross_stream_reordering(self):
        def stream_wheel(stream_id, monotonic_ns):
            return WheelState(
                header=SensorHeader(
                    mission_id="mission-1",
                    unit_id="head",
                    boot_id="boot-1",
                    producer_id=f"producer-{stream_id}",
                    stream_id=stream_id,
                    seq=1,
                    monotonic_ns=monotonic_ns,
                ),
                left_ticks=0,
                right_ticks=0,
                sample_period_ns=1,
            )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path) as writer:
                writer.submit(stream_wheel("wheel/a", 100_000_000))
                writer.submit(stream_wheel("wheel/b", 90_000_000))
                writer.submit(stream_wheel("wheel/c", 110_000_000))
            sleeps = []
            list(iter_replay(path, speed=1.0, sleep=sleeps.append))
            self.assertEqual(sleeps, [0.01])

    def test_empty_log_is_not_reported_healthy(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            with MissionLogWriter(path):
                pass
            report = inspect_mission_log(path)
            self.assertEqual(report["records"], 0)
            self.assertFalse(report["healthy"])

    def test_stale_or_nonzero_health_counters_make_log_unhealthy(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            health = SensorHealth(
                header=SensorHeader(
                    mission_id="mission-1",
                    unit_id="head",
                    boot_id="boot-1",
                    producer_id="producer-1",
                    stream_id="health/radar",
                    seq=1,
                    monotonic_ns=200_000_000,
                ),
                subject_stream_id="radar/front",
                status="stale",
                parse_errors_total=1,
            )
            with MissionLogWriter(path) as writer:
                writer.submit(wheel(1))
                writer.submit(health)
            report = inspect_mission_log(path)
            self.assertEqual(report["health_status_counts"], {"stale": 1})
            self.assertEqual(report["health_counters"]["parse_errors_total"], 1)
            self.assertEqual(report["unhealthy_health_records"], 1)
            self.assertFalse(report["healthy"])


if __name__ == "__main__":
    unittest.main()
