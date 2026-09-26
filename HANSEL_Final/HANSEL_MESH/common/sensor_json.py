"""Strict canonical JSON codec for HANSEL sensor records."""

from __future__ import annotations

import json
from typing import Any, Dict, List, Tuple, Union

from common.sensor_contract import SensorRecord, record_from_dict, record_to_dict


MAX_SENSOR_JSON_BYTES = 4 * 1024 * 1024
MAX_JSON_INTEGER_DIGITS = 128


def _reject_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number is not allowed: {value}")


def _unique_object(pairs: List[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _bounded_int(value: str) -> int:
    digits = value.lstrip("-")
    if len(digits) > MAX_JSON_INTEGER_DIGITS:
        raise ValueError(
            f"JSON integer exceeds {MAX_JSON_INTEGER_DIGITS} digits"
        )
    return int(value)


def strict_json_loads(
    raw: Union[bytes, str],
    max_bytes: int = MAX_SENSOR_JSON_BYTES,
) -> Any:
    """Decode JSON while rejecting duplicate keys and NaN/Infinity."""

    if isinstance(raw, bytes):
        if len(raw) > max_bytes:
            raise ValueError(f"JSON exceeds {max_bytes} bytes")
        text = raw.decode("utf-8", errors="strict")
    elif isinstance(raw, str):
        if len(raw.encode("utf-8")) > max_bytes:
            raise ValueError(f"JSON exceeds {max_bytes} bytes")
        text = raw
    else:
        raise TypeError("raw JSON must be bytes or str")
    return json.loads(
        text,
        object_pairs_hook=_unique_object,
        parse_constant=_reject_constant,
        parse_int=_bounded_int,
    )


def canonical_json_bytes(value: object) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def encode_sensor_record(record: SensorRecord) -> bytes:
    data = canonical_json_bytes(record_to_dict(record))
    if len(data) > MAX_SENSOR_JSON_BYTES:
        raise ValueError(f"sensor record exceeds {MAX_SENSOR_JSON_BYTES} bytes")
    return data


def decode_sensor_record(raw: Union[bytes, str]) -> SensorRecord:
    return record_from_dict(strict_json_loads(raw))
