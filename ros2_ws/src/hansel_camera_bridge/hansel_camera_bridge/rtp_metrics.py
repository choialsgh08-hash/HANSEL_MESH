"""Minimal RTP parsing and receive-side quality accounting."""

from __future__ import annotations

from dataclasses import dataclass
import struct


@dataclass(frozen=True)
class RtpHeader:
    sequence: int
    timestamp: int
    marker: bool
    header_length: int


@dataclass(frozen=True)
class MetricsSnapshot:
    receiving: bool
    receive_fps: float
    loss_rate: float
    bitrate_bps: int
    total_packets: int
    lost_packets: int
    last_frame_monotonic: float | None


def parse_rtp_header(packet: bytes) -> RtpHeader:
    if len(packet) < 12:
        raise ValueError("RTP packet is shorter than 12 bytes")
    first, second, sequence, timestamp = struct.unpack("!BBHI", packet[:8])
    version = first >> 6
    if version != 2:
        raise ValueError(f"unsupported RTP version: {version}")
    csrc_count = first & 0x0F
    extension = bool(first & 0x10)
    header_length = 12 + csrc_count * 4
    if len(packet) < header_length:
        raise ValueError("truncated RTP CSRC list")
    if extension:
        if len(packet) < header_length + 4:
            raise ValueError("truncated RTP extension header")
        extension_words = struct.unpack(
            "!H", packet[header_length + 2 : header_length + 4]
        )[0]
        header_length += 4 + extension_words * 4
        if len(packet) < header_length:
            raise ValueError("truncated RTP extension payload")
    return RtpHeader(
        sequence=sequence,
        timestamp=timestamp,
        marker=bool(second & 0x80),
        header_length=header_length,
    )


class RtpMetricsTracker:
    def __init__(self, receive_timeout_s: float = 1.5) -> None:
        self.receive_timeout_s = float(receive_timeout_s)
        self.total_packets = 0
        self.total_lost = 0
        self._last_sequence: int | None = None
        self._last_timestamp: int | None = None
        self._current_timestamp_marked = False
        self.last_packet_monotonic: float | None = None
        self.last_frame_monotonic: float | None = None
        self._window_start: float | None = None
        self._window_packets = 0
        self._window_lost = 0
        self._window_bytes = 0
        self._window_frames = 0

    def observe(self, packet: bytes, now_monotonic: float) -> RtpHeader:
        header = parse_rtp_header(packet)
        if self._window_start is None:
            self._window_start = now_monotonic
        self.total_packets += 1
        self._window_packets += 1
        self._window_bytes += len(packet)
        self.last_packet_monotonic = now_monotonic

        if self._last_sequence is not None:
            delta = (header.sequence - self._last_sequence) & 0xFFFF
            if 1 < delta < 0x8000:
                lost = delta - 1
                self.total_lost += lost
                self._window_lost += lost
            if 0 < delta < 0x8000:
                self._last_sequence = header.sequence
        else:
            self._last_sequence = header.sequence

        if (
            self._last_timestamp is not None
            and header.timestamp != self._last_timestamp
            and not self._current_timestamp_marked
        ):
            self._window_frames += 1
            self.last_frame_monotonic = now_monotonic
        if header.timestamp != self._last_timestamp:
            self._last_timestamp = header.timestamp
            self._current_timestamp_marked = False
        if header.marker and not self._current_timestamp_marked:
            self._window_frames += 1
            self.last_frame_monotonic = now_monotonic
            self._current_timestamp_marked = True
        return header

    def snapshot(self, now_monotonic: float) -> MetricsSnapshot:
        if self._window_start is None:
            elapsed = 0.0
        else:
            elapsed = max(1e-9, now_monotonic - self._window_start)
        receiving = (
            self.last_packet_monotonic is not None
            and now_monotonic - self.last_packet_monotonic <= self.receive_timeout_s
        )
        total_expected = self._window_packets + self._window_lost
        snapshot = MetricsSnapshot(
            receiving=receiving,
            receive_fps=self._window_frames / elapsed if elapsed > 0.0 else 0.0,
            loss_rate=(
                self._window_lost / total_expected if total_expected > 0 else 0.0
            ),
            bitrate_bps=(
                round(self._window_bytes * 8.0 / elapsed) if elapsed > 0.0 else 0
            ),
            total_packets=self.total_packets,
            lost_packets=self.total_lost,
            last_frame_monotonic=self.last_frame_monotonic,
        )
        self._window_start = now_monotonic
        self._window_packets = 0
        self._window_lost = 0
        self._window_bytes = 0
        self._window_frames = 0
        return snapshot

