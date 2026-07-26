from concurrent.futures import ThreadPoolExecutor
import itertools
import tempfile
import unittest
from pathlib import Path

from sensors.header_factory import (
    SensorHeaderFactory,
    new_producer_id,
    read_linux_boot_id,
)


class SensorHeaderFactoryTests(unittest.TestCase):
    def test_factory_sequences_and_uses_explicit_capture_time(self):
        monotonic_values = iter((100, 200))
        factory = SensorHeaderFactory(
            mission_id="mission-1",
            unit_id="head",
            boot_id="boot-1",
            producer_id="producer-1",
            stream_id="radar/front",
            frame_id="radar_native",
            monotonic_ns=lambda: next(monotonic_values),
            timestamp_source="uart_read_midpoint",
        )
        first = factory.next(
            sensor_timestamp_ns=77,
            timestamp_uncertainty_ns=5,
        )
        second = factory.next(capture_monotonic_ns=150)
        self.assertEqual(first.seq, 1)
        self.assertEqual(first.monotonic_ns, 100)
        self.assertEqual(first.sensor_timestamp_ns, 77)
        self.assertEqual(first.timestamp_source, "uart_read_midpoint")
        self.assertEqual(first.timestamp_uncertainty_ns, 5)
        self.assertEqual(second.seq, 2)
        self.assertEqual(second.monotonic_ns, 150)

    def test_linux_boot_id_reader_strips_newline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "boot_id"
            path.write_text("boot-uuid-1\n", encoding="ascii")
            self.assertEqual(read_linux_boot_id(path), "boot-uuid-1")

    def test_producer_id_is_unique_and_has_prefix(self):
        first = new_producer_id("radar")
        second = new_producer_id("radar")
        self.assertTrue(first.startswith("radar-"))
        self.assertNotEqual(first, second)

    def test_concurrent_default_clock_keeps_sequence_and_time_order(self):
        clock_values = itertools.count(1_000)
        factory = SensorHeaderFactory(
            mission_id="mission-1",
            unit_id="head",
            boot_id="boot-1",
            producer_id="producer-1",
            stream_id="imu/body",
            monotonic_ns=lambda: next(clock_values),
        )
        with ThreadPoolExecutor(max_workers=8) as executor:
            headers = list(executor.map(lambda _: factory.next(), range(64)))
        ordered = sorted(headers, key=lambda item: item.seq)
        self.assertEqual(
            [item.monotonic_ns for item in ordered],
            list(range(1_000, 1_064)),
        )

    def test_explicit_capture_time_regression_is_rejected(self):
        factory = SensorHeaderFactory(
            mission_id="mission-1",
            unit_id="head",
            boot_id="boot-1",
            producer_id="producer-1",
            stream_id="radar/front",
        )
        factory.next(capture_monotonic_ns=200)
        with self.assertRaisesRegex(ValueError, "regressed"):
            factory.next(capture_monotonic_ns=199)


if __name__ == "__main__":
    unittest.main()
