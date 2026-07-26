"""Strict integrity inspection for UART RAW captures and timing sidecars."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from common.sensor_contract import validate_sensor_id
from common.sensor_json import strict_json_loads


UART_CHUNK_INDEX_VERSION = 1
MAX_UART_CHUNK_INDEX_LINE_BYTES = 64 * 1024

_STABLE_METADATA_FIELDS = (
    "mission_id",
    "unit_id",
    "boot_id",
    "producer_id",
    "profile_id",
    "calibration_id",
    "baudrate",
)


def _is_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _read_integer(
    record: Mapping[str, Any],
    key: str,
    errors: List[str],
    location: str,
    minimum: int = 0,
) -> Optional[int]:
    value = record.get(key)
    if not _is_integer(value) or value < minimum:
        errors.append(
            f"{location}: {key} must be an integer >= {minimum}"
        )
        return None
    return value


def _read_metadata(
    record: Mapping[str, Any],
    errors: List[str],
    location: str,
) -> Optional[Dict[str, Any]]:
    metadata: Dict[str, Any] = {}
    for key in _STABLE_METADATA_FIELDS:
        if key not in record:
            errors.append(f"{location}: missing metadata field {key}")
            continue
        value = record[key]
        if key == "baudrate":
            if not _is_integer(value) or value <= 0:
                errors.append(
                    f"{location}: baudrate must be a positive integer"
                )
                continue
        elif key == "calibration_id":
            if value is not None:
                try:
                    validate_sensor_id(value, key)
                except ValueError as exc:
                    errors.append(f"{location}: {exc}")
                    continue
        else:
            try:
                validate_sensor_id(value, key)
            except ValueError as exc:
                errors.append(f"{location}: {exc}")
                continue
        metadata[key] = value
    if len(metadata) != len(_STABLE_METADATA_FIELDS):
        return None
    return metadata


def _drain_overlong_line(handle: object, initial: bytes) -> None:
    raw = initial
    while raw and not raw.endswith(b"\n"):
        raw = handle.readline(MAX_UART_CHUNK_INDEX_LINE_BYTES + 2)


def inspect_uart_chunk_index(
    raw_capture: Path,
    raw_index: Path,
) -> Dict[str, Any]:
    """Validate a RAW UART file against its version-1 JSONL chunk index.

    Inspection is deliberately non-recovering: an incomplete final line,
    malformed JSON, missing footer, offset gap, metadata change, or RAW-size
    mismatch makes the result unhealthy.  Expected input damage is returned in
    ``errors`` rather than raised as a decoding traceback.
    """

    raw_path = Path(raw_capture)
    index_path = Path(raw_index)
    errors: List[str] = []
    records = 0
    chunks = 0
    indexed_raw_bytes = 0
    footer_count = 0
    footer_chunks: Optional[int] = None
    footer_raw_bytes: Optional[int] = None
    footer_raw_sha256: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    previous_read_finished_ns: Optional[int] = None

    try:
        actual_raw_bytes: Optional[int] = raw_path.stat().st_size
    except OSError as exc:
        actual_raw_bytes = None
        errors.append(f"{raw_path}: cannot stat RAW capture: {exc}")
    actual_raw_sha256: Optional[str] = None
    if actual_raw_bytes is not None:
        try:
            hasher = hashlib.sha256()
            with raw_path.open("rb") as raw_handle:
                while True:
                    raw_chunk = raw_handle.read(1024 * 1024)
                    if not raw_chunk:
                        break
                    hasher.update(raw_chunk)
            actual_raw_sha256 = hasher.hexdigest()
        except OSError as exc:
            errors.append(f"{raw_path}: cannot hash RAW capture: {exc}")

    try:
        handle = index_path.open("rb")
    except OSError as exc:
        errors.append(f"{index_path}: cannot open chunk index: {exc}")
        return _summary(
            raw_path=raw_path,
            index_path=index_path,
            errors=errors,
            records=records,
            chunks=chunks,
            indexed_raw_bytes=indexed_raw_bytes,
            actual_raw_bytes=actual_raw_bytes,
            footer_count=footer_count,
            footer_chunks=footer_chunks,
            footer_raw_bytes=footer_raw_bytes,
            footer_raw_sha256=footer_raw_sha256,
            actual_raw_sha256=actual_raw_sha256,
            metadata=metadata,
        )

    with handle:
        line_number = 0
        while True:
            raw_line = handle.readline(
                MAX_UART_CHUNK_INDEX_LINE_BYTES + 2
            )
            if not raw_line:
                break
            line_number += 1
            records += 1
            location = f"{index_path}:{line_number}"

            if len(raw_line) > MAX_UART_CHUNK_INDEX_LINE_BYTES:
                errors.append(
                    f"{location}: line exceeds "
                    f"{MAX_UART_CHUNK_INDEX_LINE_BYTES} bytes"
                )
                _drain_overlong_line(handle, raw_line)
                continue

            terminated = raw_line.endswith(b"\n")
            if not terminated:
                errors.append(f"{location}: incomplete final line")
            line = raw_line[:-1] if terminated else raw_line
            if line.endswith(b"\r"):
                line = line[:-1]

            try:
                decoded = strict_json_loads(
                    line,
                    max_bytes=MAX_UART_CHUNK_INDEX_LINE_BYTES,
                )
            except (
                TypeError,
                ValueError,
                OverflowError,
                UnicodeError,
                RecursionError,
            ) as exc:
                errors.append(f"{location}: invalid JSON record: {exc}")
                continue
            if not isinstance(decoded, dict):
                errors.append(f"{location}: index record must be an object")
                continue
            record: Mapping[str, Any] = decoded

            version = record.get("index_version")
            if (
                not _is_integer(version)
                or version != UART_CHUNK_INDEX_VERSION
            ):
                errors.append(
                    f"{location}: index_version must be "
                    f"{UART_CHUNK_INDEX_VERSION}"
                )

            current_metadata = _read_metadata(record, errors, location)
            if current_metadata is not None:
                if metadata is None:
                    metadata = current_metadata
                elif current_metadata != metadata:
                    changed = [
                        key
                        for key in _STABLE_METADATA_FIELDS
                        if current_metadata[key] != metadata[key]
                    ]
                    errors.append(
                        f"{location}: capture metadata changed: "
                        + ", ".join(changed)
                    )

            record_type = record.get("record_type")
            if footer_count:
                errors.append(f"{location}: record appears after capture_end")

            if record_type == "uart_chunk":
                chunks += 1
                chunk_seq = _read_integer(
                    record,
                    "chunk_seq",
                    errors,
                    location,
                    minimum=1,
                )
                if chunk_seq is not None and chunk_seq != chunks:
                    errors.append(
                        f"{location}: expected chunk_seq {chunks}, "
                        f"got {chunk_seq}"
                    )

                byte_offset = _read_integer(
                    record,
                    "byte_offset",
                    errors,
                    location,
                )
                if (
                    byte_offset is not None
                    and byte_offset != indexed_raw_bytes
                ):
                    errors.append(
                        f"{location}: expected byte_offset "
                        f"{indexed_raw_bytes}, got {byte_offset}"
                    )
                byte_length = _read_integer(
                    record,
                    "byte_length",
                    errors,
                    location,
                    minimum=1,
                )
                if byte_length is not None:
                    indexed_raw_bytes += byte_length

                read_started_ns = _read_integer(
                    record,
                    "read_started_ns",
                    errors,
                    location,
                )
                read_finished_ns = _read_integer(
                    record,
                    "read_finished_ns",
                    errors,
                    location,
                )
                midpoint_ns = _read_integer(
                    record,
                    "observation_midpoint_ns",
                    errors,
                    location,
                )
                _read_integer(
                    record,
                    "timing_quality_metric_ns",
                    errors,
                    location,
                )
                if (
                    read_started_ns is not None
                    and read_finished_ns is not None
                ):
                    if read_started_ns > read_finished_ns:
                        errors.append(
                            f"{location}: read_started_ns exceeds "
                            "read_finished_ns"
                        )
                    if (
                        previous_read_finished_ns is not None
                        and read_started_ns < previous_read_finished_ns
                    ):
                        errors.append(
                            f"{location}: read time regressed or overlapped "
                            "the previous chunk"
                        )
                    expected_midpoint = (
                        read_started_ns + read_finished_ns
                    ) // 2
                    if (
                        midpoint_ns is not None
                        and midpoint_ns != expected_midpoint
                    ):
                        errors.append(
                            f"{location}: observation_midpoint_ns must be "
                            f"{expected_midpoint}"
                        )
                    previous_read_finished_ns = read_finished_ns
            elif record_type == "capture_end":
                footer_count += 1
                footer_chunks = _read_integer(
                    record,
                    "chunks",
                    errors,
                    location,
                )
                footer_raw_bytes = _read_integer(
                    record,
                    "raw_bytes",
                    errors,
                    location,
                )
                raw_sha256 = record.get("raw_sha256")
                if (
                    not isinstance(raw_sha256, str)
                    or len(raw_sha256) != 64
                    or any(
                        character not in "0123456789abcdef"
                        for character in raw_sha256
                    )
                ):
                    errors.append(
                        f"{location}: raw_sha256 must be a lowercase "
                        "SHA-256 hex digest"
                    )
                else:
                    footer_raw_sha256 = raw_sha256
                _read_integer(
                    record,
                    "frames_decoded",
                    errors,
                    location,
                )
                ended_monotonic_ns = _read_integer(
                    record,
                    "ended_monotonic_ns",
                    errors,
                    location,
                )
                if (
                    ended_monotonic_ns is not None
                    and previous_read_finished_ns is not None
                    and ended_monotonic_ns < previous_read_finished_ns
                ):
                    errors.append(
                        f"{location}: ended_monotonic_ns precedes the "
                        "last chunk"
                    )
                stop_reason = record.get("stop_reason")
                if not isinstance(stop_reason, str) or not stop_reason:
                    errors.append(
                        f"{location}: stop_reason must be a non-empty string"
                    )
                if footer_chunks is not None and footer_chunks != chunks:
                    errors.append(
                        f"{location}: footer chunks={footer_chunks}, "
                        f"indexed chunks={chunks}"
                    )
                if (
                    footer_raw_bytes is not None
                    and footer_raw_bytes != indexed_raw_bytes
                ):
                    errors.append(
                        f"{location}: footer raw_bytes={footer_raw_bytes}, "
                        f"indexed bytes={indexed_raw_bytes}"
                    )
            else:
                errors.append(
                    f"{location}: unsupported record_type {record_type!r}"
                )

    if footer_count == 0:
        errors.append(f"{index_path}: missing capture_end footer")
    elif footer_count != 1:
        errors.append(
            f"{index_path}: expected exactly one capture_end footer, "
            f"found {footer_count}"
        )

    if (
        actual_raw_bytes is not None
        and actual_raw_bytes != indexed_raw_bytes
    ):
        errors.append(
            f"{raw_path}: actual RAW size {actual_raw_bytes} does not match "
            f"indexed bytes {indexed_raw_bytes}"
        )
    if (
        actual_raw_bytes is not None
        and footer_raw_bytes is not None
        and actual_raw_bytes != footer_raw_bytes
    ):
        errors.append(
            f"{raw_path}: actual RAW size {actual_raw_bytes} does not match "
            f"footer raw_bytes {footer_raw_bytes}"
        )
    if (
        actual_raw_sha256 is not None
        and footer_raw_sha256 is not None
        and actual_raw_sha256 != footer_raw_sha256
    ):
        errors.append(
            f"{raw_path}: actual RAW SHA-256 does not match footer"
        )

    return _summary(
        raw_path=raw_path,
        index_path=index_path,
        errors=errors,
        records=records,
        chunks=chunks,
        indexed_raw_bytes=indexed_raw_bytes,
        actual_raw_bytes=actual_raw_bytes,
        footer_count=footer_count,
        footer_chunks=footer_chunks,
        footer_raw_bytes=footer_raw_bytes,
        footer_raw_sha256=footer_raw_sha256,
        actual_raw_sha256=actual_raw_sha256,
        metadata=metadata,
    )


def _summary(
    raw_path: Path,
    index_path: Path,
    errors: List[str],
    records: int,
    chunks: int,
    indexed_raw_bytes: int,
    actual_raw_bytes: Optional[int],
    footer_count: int,
    footer_chunks: Optional[int],
    footer_raw_bytes: Optional[int],
    footer_raw_sha256: Optional[str],
    actual_raw_sha256: Optional[str],
    metadata: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "healthy": not errors,
        "errors": list(errors),
        "raw_capture": str(raw_path),
        "raw_index": str(index_path),
        "index_version": UART_CHUNK_INDEX_VERSION,
        "records": records,
        "chunks": chunks,
        "indexed_raw_bytes": indexed_raw_bytes,
        "actual_raw_bytes": actual_raw_bytes,
        "footer_present": footer_count == 1,
        "footer_count": footer_count,
        "footer_chunks": footer_chunks,
        "footer_raw_bytes": footer_raw_bytes,
        "footer_raw_sha256": footer_raw_sha256,
        "actual_raw_sha256": actual_raw_sha256,
    }
    for key in _STABLE_METADATA_FIELDS:
        result[key] = None if metadata is None else metadata[key]
    return result
