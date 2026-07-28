from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from common.sensor_contract import (
    RadarFrame,
    RadarHeatmap,
    SensorHeader,
    SensorHealth,
)
from sensors.mission_log import DEFAULT_MAX_LINE_BYTES, encode_log_entry
from sensors.radar_watchdog import (
    ExpectedRadarEvidence,
    RadarEpochWatchdog,
)


PROFILE_ID = (
    "lsdk-05.05.04.02-presence-near-"
    "heatmap16-elev8-cfar15-10hz-v1"
)
EXPECTED = ExpectedRadarEvidence(
    profile_id=PROFILE_ID,
    heatmap_azimuth_bins=16,
    heatmap_range_bins=128,
    heatmap_range_step_m=0.09765625,
)


def append_bytes(path: Path, data: bytes) -> None:
    with path.open("ab") as handle:
        handle.write(data)


def sensor_header(
    *,
    seq: int,
    stream_id: str = "radar/front",
) -> SensorHeader:
    return SensorHeader(
        mission_id="mission-1",
        unit_id="head",
        boot_id="boot-1",
        producer_id="radar-epoch-1",
        stream_id=stream_id,
        seq=seq,
        monotonic_ns=seq * 100_000_000,
    )


def radar_frame(
    *,
    seq: int,
    frame_number: int,
    complete: bool = True,
    profile_id: str = PROFILE_ID,
    heatmap: bool = True,
    azimuth_bins: int = 16,
    range_bins: int = 128,
    range_step_m: float = 0.09765625,
) -> RadarFrame:
    radar_heatmap = None
    if heatmap:
        radar_heatmap = RadarHeatmap(
            data=bytes(azimuth_bins * range_bins),
            range_bins=range_bins,
            azimuth_bins=azimuth_bins,
            range_step_m=range_step_m,
            tlv_type=304,
            motion_mode="major",
            floor_db=1.0,
            ceiling_db=2.0,
        )
    return RadarFrame(
        header=sensor_header(seq=seq),
        frame_number=frame_number,
        subframe_number=0,
        complete=complete,
        dropped_frames_since_previous=0,
        points=(),
        profile_id=profile_id,
        heatmap=radar_heatmap,
    )


def sensor_health(*, seq: int, status: str) -> SensorHealth:
    return SensorHealth(
        header=sensor_header(seq=seq, stream_id="health/radar"),
        subject_stream_id="radar/front",
        status=status,
    )


def encoded_record(log_seq: int, record: object) -> bytes:
    return encode_log_entry(log_seq, record) + b"\n"


def append_radar_frame(
    path: Path,
    *,
    log_seq: int,
    frame_number: int,
    **overrides: object,
) -> None:
    append_bytes(
        path,
        encoded_record(
            log_seq,
            radar_frame(
                seq=log_seq,
                frame_number=frame_number,
                **overrides,
            ),
        ),
    )


def append_sensor_health(path: Path, *, log_seq: int, status: str) -> None:
    append_bytes(
        path,
        encoded_record(
            log_seq,
            sensor_health(seq=log_seq, status=status),
        ),
    )


def make_watchdog(mission: Path, raw: Path) -> RadarEpochWatchdog:
    return RadarEpochWatchdog(
        mission_path=mission,
        raw_path=raw,
        expected=EXPECTED,
        started_at_s=0.0,
        first_frame_timeout_s=3.0,
        frame_timeout_s=2.5,
        required_consecutive_frames=5,
        verification_timeout_s=3.0,
    )


class RadarEpochWatchdogTests(unittest.TestCase):
    def test_sensor_health_does_not_refresh_radar_frame_freshness(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")

            append_radar_frame(mission, log_seq=1, frame_number=10)
            self.assertIsNone(watchdog.poll(0.1).fault_reason)
            append_sensor_health(mission, log_seq=2, status="ok")

            snapshot = watchdog.poll(2.7)

            self.assertEqual(snapshot.fault_reason, "radar_frame_timeout")
            self.assertEqual(snapshot.last_frame_observed_s, 0.1)
            self.assertEqual(snapshot.latest_frame_number, 10)

    def test_missing_epoch_files_are_not_faults_before_first_frame_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            watchdog = make_watchdog(
                root / "not-created.jsonl",
                root / "not-created.bin",
            )

            self.assertIsNone(watchdog.poll(2.999).fault_reason)
            self.assertEqual(
                watchdog.poll(3.0).fault_reason,
                "radar_frame_timeout",
            )

    def test_split_json_line_is_decoded_only_after_newline_arrives(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            line = encoded_record(
                1,
                radar_frame(seq=1, frame_number=21),
            )
            midpoint = len(line) // 2
            mission.write_bytes(line[:midpoint])

            partial = watchdog.poll(0.1)
            append_bytes(mission, line[midpoint:])
            complete = watchdog.poll(0.2)

            self.assertIsNone(partial.last_frame_observed_s)
            self.assertEqual(complete.last_frame_observed_s, 0.2)
            self.assertEqual(complete.latest_frame_number, 21)

    def test_truncated_mission_epoch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            append_radar_frame(mission, log_seq=1, frame_number=1)
            watchdog.poll(0.1)

            mission.write_bytes(b"")

            self.assertEqual(
                watchdog.poll(0.2).fault_reason,
                "mission_evidence_invalid",
            )

    def test_mission_truncate_and_regrow_past_offset_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            append_radar_frame(mission, log_seq=1, frame_number=1)
            consumed_size = mission.stat().st_size
            watchdog.poll(0.1)

            mission.write_bytes(b"x" * consumed_size)

            self.assertEqual(
                watchdog.poll(0.2).fault_reason,
                "mission_evidence_invalid",
            )

    def test_replaced_mission_epoch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            append_radar_frame(mission, log_seq=1, frame_number=1)
            watchdog.poll(0.1)
            replacement = root / "replacement.jsonl"
            replacement.write_bytes(
                encoded_record(
                    1,
                    radar_frame(seq=1, frame_number=99),
                )
            )
            replacement.replace(mission)

            snapshot = watchdog.poll(0.2)

            self.assertEqual(
                snapshot.fault_reason,
                "mission_evidence_invalid",
            )
            self.assertEqual(snapshot.latest_frame_number, 1)

    def test_over_limit_partial_mission_line_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            mission.write_bytes(b"x" * (DEFAULT_MAX_LINE_BYTES + 1))

            self.assertEqual(
                watchdog.poll(0.1).fault_reason,
                "mission_evidence_invalid",
            )

    def test_five_consecutive_qualifying_frames_verify_the_epoch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            for log_seq in range(1, 5):
                append_radar_frame(
                    mission,
                    log_seq=log_seq,
                    frame_number=100 + log_seq,
                )

            four_frames = watchdog.poll(0.4)
            append_radar_frame(mission, log_seq=5, frame_number=105)
            five_frames = watchdog.poll(0.5)

            self.assertFalse(four_frames.verified)
            self.assertEqual(four_frames.consecutive_good_frames, 4)
            self.assertTrue(five_frames.verified)
            self.assertEqual(five_frames.consecutive_good_frames, 5)

    def test_each_nonqualifying_frame_resets_consecutive_evidence(self):
        cases = {
            "incomplete": {"complete": False},
            "wrong_profile": {"profile_id": "wrong-profile"},
            "missing_heatmap": {"heatmap": False},
            "wrong_azimuth_bins": {"azimuth_bins": 15},
            "wrong_range_bins": {"range_bins": 127},
            "wrong_range_step": {"range_step_m": 0.1},
        }
        for name, overrides in cases.items():
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    mission = root / "mission.jsonl"
                    watchdog = make_watchdog(
                        mission,
                        root / "capture.bin",
                    )
                    append_radar_frame(
                        mission,
                        log_seq=1,
                        frame_number=1,
                    )
                    self.assertEqual(
                        watchdog.poll(0.1).consecutive_good_frames,
                        1,
                    )
                    append_radar_frame(
                        mission,
                        log_seq=2,
                        frame_number=2,
                        **overrides,
                    )

                    snapshot = watchdog.poll(0.2)

                    self.assertEqual(snapshot.consecutive_good_frames, 0)
                    self.assertFalse(snapshot.verified)
                    self.assertEqual(snapshot.last_frame_observed_s, 0.2)

    def test_verification_must_finish_before_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            for log_seq in range(1, 5):
                append_radar_frame(
                    mission,
                    log_seq=log_seq,
                    frame_number=log_seq,
                )

            self.assertIsNone(watchdog.poll(2.999).fault_reason)
            snapshot = watchdog.poll(3.0)

            self.assertFalse(snapshot.verified)
            self.assertEqual(
                snapshot.fault_reason,
                "radar_verification_timeout",
            )

    def test_frame_timeout_starts_at_supervisor_observation_time(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mission = root / "mission.jsonl"
            watchdog = make_watchdog(mission, root / "capture.bin")
            append_radar_frame(mission, log_seq=1, frame_number=1)
            watchdog.poll(0.1)

            self.assertIsNone(watchdog.poll(2.6).fault_reason)
            self.assertEqual(
                watchdog.poll(2.600001).fault_reason,
                "radar_frame_timeout",
            )

    def test_split_confirmed_firmware_marker_faults(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "capture.bin"
            watchdog = make_watchdog(root / "mission.jsonl", raw)
            raw.write_bytes(
                b"binary-prefixError: No Sufficient Time "
            )
            self.assertIsNone(watchdog.poll(0.1).fault_reason)

            append_bytes(
                raw,
                b"for getting into Low Power Modes.\n",
            )

            self.assertEqual(
                watchdog.poll(0.2).fault_reason,
                "firmware_low_power_timing_assert",
            )

    def test_raw_truncate_and_regrow_past_offset_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "capture.bin"
            watchdog = make_watchdog(root / "mission.jsonl", raw)
            initial_size = 96
            raw.write_bytes(b"x" * initial_size)
            self.assertIsNone(watchdog.poll(0.1).fault_reason)
            marker = (
                b"Error: No Sufficient Time for getting into "
                b"Low Power Modes."
            )

            raw.write_bytes(
                marker + b"y" * (initial_size - len(marker))
            )

            self.assertEqual(
                watchdog.poll(0.2).fault_reason,
                "raw_evidence_invalid",
            )

    def test_arbitrary_binary_and_other_error_strings_do_not_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = root / "capture.bin"
            watchdog = make_watchdog(root / "mission.jsonl", raw)
            raw.write_bytes(
                b"\x00\xffError: unrelated failure\n"
                b"No Sufficient Time for getting into Low Power Modes.\n"
                b"Error: No Sufficient Time for getting into Low Power Mode.\n"
            )

            self.assertIsNone(watchdog.poll(0.2).fault_reason)


if __name__ == "__main__":
    unittest.main()
