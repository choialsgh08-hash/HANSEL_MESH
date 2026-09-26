from unittest.mock import patch

import _paths
from hansel_localization.bno085 import Bno085RvcSource, RvcFrameBuffer


def test_silence_messages_distinguish_transport_and_framing():
    source = Bno085RvcSource.__new__(Bno085RvcSource)
    source._max_age_s = .35
    source.last_byte_s = None
    assert "0 bytes" in source.silence_reason()
    source.last_byte_s = 10
    with patch("hansel_localization.bno085.time.monotonic", return_value=10.1):
        assert "checksum" in source.silence_reason()
    with patch("hansel_localization.bno085.time.monotonic", return_value=11):
        assert "stopped" in source.silence_reason()


def test_rvc_fragmentation_bad_frame_and_identical_new_heading():
    frames = RvcFrameBuffer(lambda data: (30,) if data == bytes(17) else None)
    packet = b"\xaa\xaa" + bytes(17)
    assert frames.feed(packet[:9]) is None
    assert frames.feed(packet[9:]) is not None
    assert frames.feed(b"\xaa\xaa" + b"x" * 17 + packet) is not None
    assert frames.valid_frames == 2
    assert frames.invalid_frames == 1
    assert frames.feed(b"") is None
