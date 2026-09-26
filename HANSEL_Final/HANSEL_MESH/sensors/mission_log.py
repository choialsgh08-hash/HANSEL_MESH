"""Bounded JSONL mission recorder, validator, inspector, and replay engine."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Deque, Dict, Iterator, Mapping, Optional, Set, Tuple

from common.sensor_contract import (
    RadarFrame,
    SensorHealth,
    SensorRecord,
    record_from_dict,
    record_to_dict,
)
from common.sensor_json import (
    MAX_SENSOR_JSON_BYTES,
    canonical_json_bytes,
    strict_json_loads,
)


LOG_VERSION = 1
DEFAULT_MAX_LINE_BYTES = MAX_SENSOR_JSON_BYTES + 1024


@dataclass(frozen=True)
class LogEntry:
    log_seq: int
    record: SensorRecord
    line_number: int = 0


@dataclass(frozen=True)
class WriterStats:
    accepted_records: int
    written_records: int
    dropped_records: int
    queued_records: int
    queued_bytes: int
    failed: bool


@dataclass
class _PendingWrite:
    log_seq: int
    data: bytes
    critical: bool
    done: Optional[threading.Event] = None
    error: Optional[BaseException] = None


class MissionLogError(RuntimeError):
    pass


def encode_log_entry(log_seq: int, record: SensorRecord) -> bytes:
    if isinstance(log_seq, bool) or not isinstance(log_seq, int) or log_seq < 1:
        raise ValueError("log_seq must be a positive integer")
    return canonical_json_bytes(
        {
            "log_version": LOG_VERSION,
            "log_seq": log_seq,
            "record": record_to_dict(record),
        }
    )


def decode_log_entry(raw: bytes, line_number: int = 0) -> LogEntry:
    value = strict_json_loads(raw, max_bytes=DEFAULT_MAX_LINE_BYTES)
    if not isinstance(value, Mapping):
        raise ValueError("log line must be an object")
    version = value.get("log_version")
    if isinstance(version, bool) or not isinstance(version, int):
        raise ValueError("log_version must be an integer")
    if version != LOG_VERSION:
        raise ValueError(f"unsupported log version: {version}")
    log_seq = value.get("log_seq")
    if isinstance(log_seq, bool) or not isinstance(log_seq, int) or log_seq < 1:
        raise ValueError("log_seq must be a positive integer")
    if "record" not in value:
        raise ValueError("record is required")
    return LogEntry(
        log_seq=log_seq,
        record=record_from_dict(value["record"]),
        line_number=line_number,
    )


class MissionLogWriter:
    """Single-owner background writer with record and byte queue bounds.

    Normal sensor samples use :meth:`submit` and are explicitly rejected when
    the queue is full.  Safety- or mission-critical events use
    :meth:`write_critical`, which waits for capacity and for an ``fsync``.
    """

    def __init__(
        self,
        path: Path,
        max_queue_records: int = 256,
        max_queue_bytes: int = 16 * 1024 * 1024,
        max_record_bytes: int = DEFAULT_MAX_LINE_BYTES,
        overwrite: bool = False,
    ) -> None:
        if max_queue_records < 1:
            raise ValueError("max_queue_records must be positive")
        if max_queue_bytes < 1:
            raise ValueError("max_queue_bytes must be positive")
        if max_record_bytes < 1 or max_record_bytes > max_queue_bytes:
            raise ValueError(
                "max_record_bytes must be positive and no larger than "
                "max_queue_bytes"
            )
        self.path = Path(path)
        self.max_queue_records = max_queue_records
        self.max_queue_bytes = max_queue_bytes
        self.max_record_bytes = max_record_bytes

        self._condition = threading.Condition()
        self._queue: Deque[_PendingWrite] = deque()
        self._queued_bytes = 0
        self._outstanding_records = 0
        self._next_log_seq = 1
        self._accepted_records = 0
        self._written_records = 0
        self._dropped_records = 0
        self._closing = False
        self._closed = False
        self._failure: Optional[BaseException] = None
        self._mission_id: Optional[str] = None

        parent_existed = self.path.parent.exists()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        mode = "wb" if overwrite else "xb"
        self._file = self.path.open(mode, buffering=0)
        try:
            self._fsync_directory(self.path.parent)
            if not parent_existed:
                self._fsync_directory(self.path.parent.parent)
        except BaseException:
            self._file.close()
            raise
        self._thread = threading.Thread(
            target=self._run,
            name="hansel-mission-log-writer",
            daemon=True,
        )
        try:
            self._thread.start()
        except BaseException:
            self._file.close()
            raise

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        # Linux requires the containing directory to be synced before a newly
        # created filename is guaranteed to survive sudden power loss.
        if os.name != "posix":
            return
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        descriptor = os.open(str(path), flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def __enter__(self) -> "MissionLogWriter":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close(drain=True)

    def _raise_if_unavailable(self) -> None:
        if self._failure is not None:
            raise MissionLogError("mission log writer failed") from self._failure
        if self._closing or self._closed:
            raise MissionLogError("mission log writer is closed")

    def _fits(self, data_size: int) -> bool:
        return (
            self._outstanding_records < self.max_queue_records
            and self._queued_bytes + data_size <= self.max_queue_bytes
        )

    def submit(self, record: SensorRecord) -> bool:
        """Queue a normal sample without blocking.

        ``False`` is a visible backpressure/drop signal.  Previously accepted
        records are never evicted to make room for a newer sample.
        """

        with self._condition:
            self._raise_if_unavailable()
            log_seq = self._next_log_seq
            data = encode_log_entry(log_seq, record)
            if len(data) > self.max_record_bytes:
                raise ValueError(
                    f"log record exceeds {self.max_record_bytes} bytes"
                )
            if not self._fits(len(data)):
                self._dropped_records += 1
                return False
            self._bind_mission(record)
            self._enqueue(log_seq, data, critical=False)
            return True

    def write_critical(
        self,
        record: SensorRecord,
        timeout_s: float = 2.0,
    ) -> int:
        """Durably write one critical record and return its log sequence."""

        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        deadline = time.monotonic() + timeout_s
        done = threading.Event()
        with self._condition:
            self._raise_if_unavailable()
            while True:
                # A capacity wait releases the condition lock.  Re-read the
                # sequence and re-encode after every wake because a normal
                # submitter may have enqueued while this thread was waiting.
                log_seq = self._next_log_seq
                data = encode_log_entry(log_seq, record)
                if len(data) > self.max_record_bytes:
                    raise ValueError(
                        f"log record exceeds {self.max_record_bytes} bytes"
                    )
                if self._fits(len(data)):
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("timed out waiting for mission log capacity")
                self._condition.wait(remaining)
                self._raise_if_unavailable()
            self._bind_mission(record)
            pending = self._enqueue(
                log_seq,
                data,
                critical=True,
                done=done,
            )

        remaining = deadline - time.monotonic()
        if remaining <= 0 or not done.wait(remaining):
            raise TimeoutError(
                "critical mission record was accepted but durability "
                "confirmation timed out"
            )
        with self._condition:
            if pending.error is not None:
                raise MissionLogError(
                    "critical mission record failed"
                ) from pending.error
            if self._failure is not None:
                raise MissionLogError("mission log writer failed") from self._failure
        return log_seq

    def _bind_mission(self, record: SensorRecord) -> None:
        mission_id = record.header.mission_id
        if self._mission_id is None:
            self._mission_id = mission_id
        elif mission_id != self._mission_id:
            raise ValueError(
                "mission log cannot mix mission_id values: "
                f"{self._mission_id!r} and {mission_id!r}"
            )

    def _enqueue(
        self,
        log_seq: int,
        data: bytes,
        critical: bool,
        done: Optional[threading.Event] = None,
    ) -> _PendingWrite:
        pending = _PendingWrite(
            log_seq=log_seq,
            data=data,
            critical=critical,
            done=done,
        )
        self._queue.append(pending)
        self._queued_bytes += len(data)
        self._outstanding_records += 1
        self._next_log_seq += 1
        self._accepted_records += 1
        self._condition.notify()
        return pending

    def stats(self) -> WriterStats:
        with self._condition:
            return WriterStats(
                accepted_records=self._accepted_records,
                written_records=self._written_records,
                dropped_records=self._dropped_records,
                queued_records=self._outstanding_records,
                queued_bytes=self._queued_bytes,
                failed=self._failure is not None,
            )

    def raise_if_failed(self) -> None:
        with self._condition:
            if self._failure is not None:
                raise MissionLogError("mission log writer failed") from self._failure

    def close(self, drain: bool = True, timeout_s: float = 10.0) -> None:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        with self._condition:
            if self._closed:
                return
            self._closing = True
            if not drain:
                while self._queue:
                    pending = self._queue.popleft()
                    self._dropped_records += 1
                    self._queued_bytes -= len(pending.data)
                    self._outstanding_records -= 1
                    if pending.done is not None:
                        pending.error = MissionLogError(
                            "writer closed before critical write"
                        )
                        pending.done.set()
            self._condition.notify_all()

        self._thread.join(timeout_s)
        if self._thread.is_alive():
            raise TimeoutError("timed out closing mission log writer")
        with self._condition:
            self._closed = True
            if self._failure is not None:
                raise MissionLogError("mission log writer failed") from self._failure

    def _run(self) -> None:
        active_pending: Optional[_PendingWrite] = None
        try:
            while True:
                with self._condition:
                    while not self._queue and not self._closing:
                        self._condition.wait()
                    if not self._queue:
                        break
                    pending = self._queue.popleft()
                    active_pending = pending

                self._write_all(pending.data)
                self._write_all(b"\n")
                if pending.critical:
                    os.fsync(self._file.fileno())

                with self._condition:
                    self._written_records += 1
                    self._queued_bytes -= len(pending.data)
                    self._outstanding_records -= 1
                    if pending.done is not None:
                        pending.done.set()
                    self._condition.notify_all()
                active_pending = None

            os.fsync(self._file.fileno())
        except BaseException as exc:
            with self._condition:
                self._failure = exc
                if active_pending is not None and active_pending.done is not None:
                    active_pending.error = exc
                    active_pending.done.set()
                while self._queue:
                    queued = self._queue.popleft()
                    if queued.done is not None:
                        queued.error = exc
                        queued.done.set()
                self._queued_bytes = 0
                self._outstanding_records = 0
                self._condition.notify_all()
        finally:
            try:
                self._file.close()
            except BaseException as close_error:
                with self._condition:
                    if self._failure is None:
                        self._failure = close_error
                    self._condition.notify_all()

    def _write_all(self, data: bytes) -> None:
        view = memoryview(data)
        while view:
            written = self._file.write(view)
            if written is None or written <= 0:
                raise OSError("mission log write made no progress")
            view = view[written:]


def iter_mission_log(
    path: Path,
    recover_trailing_partial: bool = True,
    max_line_bytes: int = DEFAULT_MAX_LINE_BYTES,
) -> Iterator[LogEntry]:
    """Read and strictly validate a mission log.

    A crash-truncated final line may be ignored.  Any malformed complete line,
    duplicate key, sequence discontinuity, or invalid sensor value is fatal.
    """

    if max_line_bytes < 1:
        raise ValueError("max_line_bytes must be positive")
    expected_log_seq = 1
    with Path(path).open("rb") as handle:
        line_number = 0
        while True:
            raw = handle.readline(max_line_bytes + 2)
            if not raw:
                break
            line_number += 1
            if len(raw) > max_line_bytes:
                raise MissionLogError(
                    f"{path}:{line_number}: line exceeds {max_line_bytes} bytes"
                )
            if not raw.endswith(b"\n"):
                if recover_trailing_partial:
                    break
                raise MissionLogError(
                    f"{path}:{line_number}: incomplete final line"
                )
            line = raw[:-1]
            if line.endswith(b"\r"):
                line = line[:-1]
            try:
                entry = decode_log_entry(line, line_number=line_number)
            except (
                TypeError,
                ValueError,
                OverflowError,
                UnicodeError,
                RecursionError,
            ) as exc:
                raise MissionLogError(
                    f"{path}:{line_number}: invalid log record: {exc}"
                ) from exc
            if entry.log_seq != expected_log_seq:
                raise MissionLogError(
                    f"{path}:{line_number}: expected log_seq "
                    f"{expected_log_seq}, got {entry.log_seq}"
                )
            expected_log_seq += 1
            yield entry


def inspect_mission_log(path: Path) -> Dict[str, Any]:
    record_counts: Counter = Counter()
    stream_counts: Counter = Counter()
    missions: Set[str] = set()
    units: Set[str] = set()
    boots: Set[str] = set()
    sequence_gaps = 0
    duplicate_sequences = 0
    sequence_regressions = 0
    monotonic_regressions = 0
    health_status_counts: Counter = Counter()
    unhealthy_health_records = 0
    radar_incomplete_frames = 0
    radar_declared_drops = 0
    radar_discontinuity_frames = 0
    health_counters = {
        "seq_gaps_total": 0,
        "parse_errors_total": 0,
        "producer_drops_total": 0,
        "writer_drops_total": 0,
        "device_discontinuities_total": 0,
    }
    previous: Dict[Tuple[str, str, str, str, str], Tuple[int, int]] = {}
    total = 0
    data_records = 0
    trailing_partial_bytes = _trailing_partial_bytes(Path(path))

    for entry in iter_mission_log(path):
        total += 1
        record = entry.record
        record_type = record_to_dict(record)["record_type"]
        header = record.header
        record_counts[record_type] += 1
        stream_counts[header.stream_id] += 1
        missions.add(header.mission_id)
        units.add(header.unit_id)
        boots.add(header.boot_id)
        if isinstance(record, SensorHealth):
            health_status_counts[record.status] += 1
            counter_values = {
                "seq_gaps_total": record.seq_gaps_total,
                "parse_errors_total": record.parse_errors_total,
                "producer_drops_total": record.producer_drops_total,
                "writer_drops_total": record.writer_drops_total,
                "device_discontinuities_total": (
                    record.device_discontinuities_total
                ),
            }
            for name, value in counter_values.items():
                health_counters[name] = max(health_counters[name], value)
            if record.status != "ok" or any(counter_values.values()):
                unhealthy_health_records += 1
        else:
            data_records += 1
            if isinstance(record, RadarFrame):
                if not record.complete:
                    radar_incomplete_frames += 1
                radar_declared_drops += (
                    record.dropped_frames_since_previous
                )
                if record.frame_transition in {
                    "gap",
                    "duplicate",
                    "reset_or_out_of_order",
                }:
                    radar_discontinuity_frames += 1
        key = (
            header.mission_id,
            header.unit_id,
            header.boot_id,
            header.producer_id,
            header.stream_id,
        )
        old = previous.get(key)
        if old is None:
            if header.seq > 1:
                sequence_gaps += header.seq - 1
        else:
            old_seq, old_time = old
            if header.seq == old_seq:
                duplicate_sequences += 1
            elif header.seq < old_seq:
                sequence_regressions += 1
            elif header.seq > old_seq + 1:
                sequence_gaps += header.seq - old_seq - 1
            if header.monotonic_ns < old_time:
                monotonic_regressions += 1
        previous[key] = (header.seq, header.monotonic_ns)

    return {
        "path": str(Path(path)),
        "records": total,
        "data_records": data_records,
        "record_counts": dict(sorted(record_counts.items())),
        "stream_counts": dict(sorted(stream_counts.items())),
        "mission_ids": sorted(missions),
        "mixed_missions": len(missions) > 1,
        "unit_ids": sorted(units),
        "boot_ids": sorted(boots),
        "sequence_gaps": sequence_gaps,
        "duplicate_sequences": duplicate_sequences,
        "sequence_regressions": sequence_regressions,
        "monotonic_regressions": monotonic_regressions,
        "health_status_counts": dict(sorted(health_status_counts.items())),
        "unhealthy_health_records": unhealthy_health_records,
        "radar_incomplete_frames": radar_incomplete_frames,
        "radar_declared_drops": radar_declared_drops,
        "radar_discontinuity_frames": radar_discontinuity_frames,
        "health_counters": health_counters,
        "trailing_partial_recovered": trailing_partial_bytes > 0,
        "trailing_partial_bytes": trailing_partial_bytes,
        "healthy": (
            total > 0
            and data_records > 0
            and len(missions) == 1
            and trailing_partial_bytes == 0
            and unhealthy_health_records == 0
            and radar_incomplete_frames == 0
            and radar_declared_drops == 0
            and radar_discontinuity_frames == 0
            and sequence_gaps == 0
            and duplicate_sequences == 0
            and sequence_regressions == 0
            and monotonic_regressions == 0
        ),
    }


def _trailing_partial_bytes(path: Path) -> int:
    with path.open("rb") as handle:
        handle.seek(0, os.SEEK_END)
        size = handle.tell()
        if size == 0:
            return 0
        handle.seek(-1, os.SEEK_END)
        if handle.read(1) == b"\n":
            return 0
        position = size
        while position > 0:
            read_size = min(4096, position)
            position -= read_size
            handle.seek(position)
            chunk = handle.read(read_size)
            newline_index = chunk.rfind(b"\n")
            if newline_index >= 0:
                return size - (position + newline_index + 1)
        return size


def iter_replay(
    path: Path,
    speed: float = 0.0,
    sleep: Callable[[float], None] = time.sleep,
) -> Iterator[LogEntry]:
    """Yield records in durable log order, optionally preserving timing.

    ``speed=0`` is deterministic logical replay with no sleep.  Positive
    values divide the captured monotonic interval, so ``2`` is twice real
    time.  A boot/unit transition has no implied timing relationship.
    """

    if isinstance(speed, bool) or not isinstance(speed, (int, float)):
        raise ValueError("speed must be a non-negative number")
    speed_value = float(speed)
    if speed_value < 0 or not (speed_value < float("inf")):
        raise ValueError("speed must be finite and non-negative")

    previous_domain: Optional[Tuple[str, str]] = None
    playback_cursor_ns: Optional[int] = None
    for entry in iter_mission_log(path):
        header = entry.record.header
        domain = (header.unit_id, header.boot_id)
        if domain != previous_domain or playback_cursor_ns is None:
            playback_cursor_ns = header.monotonic_ns
        else:
            target_ns = max(playback_cursor_ns, header.monotonic_ns)
            delay_ns = target_ns - playback_cursor_ns
            if speed_value > 0 and delay_ns:
                sleep(delay_ns / 1_000_000_000.0 / speed_value)
            playback_cursor_ns = target_ns
        previous_domain = domain
        yield entry
