"""Unit tests for input validation and enums."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from rescue_network.schemas import InjuryStatus, RescueRequestCreate


def _valid_payload(**overrides):
    data = {
        "people_count": 2,
        "injury_status": "yes",
        "condition": "고립됨",
        "message": "도와주세요",
        "location_text": "OO아파트 근처",
    }
    data.update(overrides)
    return data


def test_valid_payload_parses():
    model = RescueRequestCreate(**_valid_payload())
    assert model.people_count == 2
    assert model.injury_status is InjuryStatus.YES
    assert model.location_text == "OO아파트 근처"


@pytest.mark.parametrize("count", [0, -1, 1001])
def test_people_count_out_of_range_rejected(count):
    with pytest.raises(ValidationError):
        RescueRequestCreate(**_valid_payload(people_count=count))


@pytest.mark.parametrize("count", [1, 1000])
def test_people_count_bounds_accepted(count):
    assert RescueRequestCreate(**_valid_payload(people_count=count)).people_count == count


def test_injury_status_enum_rejects_unknown_value():
    with pytest.raises(ValidationError):
        RescueRequestCreate(**_valid_payload(injury_status="maybe"))


@pytest.mark.parametrize("value", ["yes", "no", "unknown"])
def test_injury_status_enum_accepts_all_members(value):
    assert RescueRequestCreate(**_valid_payload(injury_status=value)).injury_status.value == value


def test_blank_required_fields_rejected():
    with pytest.raises(ValidationError):
        RescueRequestCreate(**_valid_payload(message=""))
    with pytest.raises(ValidationError):
        RescueRequestCreate(**_valid_payload(condition="   "))  # stripped to empty


def test_location_text_optional():
    model = RescueRequestCreate(**_valid_payload(location_text=None))
    assert model.location_text is None


def test_overlong_message_rejected():
    with pytest.raises(ValidationError):
        RescueRequestCreate(**_valid_payload(message="x" * 2001))
