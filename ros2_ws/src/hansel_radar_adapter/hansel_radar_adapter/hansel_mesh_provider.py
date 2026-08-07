"""Tail HANSEL_MESH mission JSONL radar records and publish ROS messages."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct
import threading
import time
from typing import Any

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

from .provider import RadarProvider


class HanselMeshMissionLogProvider(RadarProvider):
    """Convert mission log `radar_frame` records to PointCloud2/OccupancyGrid.

    HANSEL_MESH radar native axes are lateral-right x and forward y. ROS output
    uses x forward, y left, z up, so (x_ros, y_ros, z_ros)=(y_raw,-x_raw,z_raw).
    """

    def __init__(self, adapter) -> None:
        super().__init__(adapter)
        p = adapter.get_parameter
        raw_path = str(p("mission_log_path").value).strip()
        if not raw_path:
            raise ValueError("mission_log_path must be configured")
        self.path = Path(raw_path).expanduser()
        self.start_at_end = bool(p("mission_log_start_at_end").value)
        self.poll_period_s = max(0.02, float(p("mission_log_poll_period_s").value))
        self.frame_id = str(p("frame_id").value)
        self.max_points = max(1, int(p("max_points_per_frame").value))
        self.publish_grid = bool(p("publish_occupancy_grid").value)
        self.grid_resolution = float(p("grid_resolution_m").value)
        self.grid_forward_range = float(p("grid_forward_range_m").value)
        self.grid_lateral_width = float(p("grid_lateral_width_m").value)
        if self.grid_resolution <= 0.0:
            raise ValueError("grid_resolution_m must be positive")
        if self.grid_forward_range <= 0.0 or self.grid_lateral_width <= 0.0:
            raise ValueError("grid extents must be positive")
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._records = 0
        self._invalid = 0
        self._incomplete = 0
        self._last_points = 0
        self._last_record_monotonic: float | None = None

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._tail_loop,
            name="hansel-radar-mission-log",
            daemon=True,
        )
        self._thread.start()
        self.adapter.get_logger().info(f"tailing radar mission log: {self.path}")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None

    def _tail_loop(self) -> None:
        stream = None
        current_identity = None
        first_open = True
        try:
            while not self._stop.is_set():
                try:
                    stat = self.path.stat()
                    identity = (stat.st_dev, stat.st_ino)
                    if stream is None or identity != current_identity:
                        if stream is not None:
                            stream.close()
                        stream = self.path.open("r", encoding="utf-8")
                        current_identity = identity
                        if first_open and self.start_at_end:
                            stream.seek(0, 2)
                        first_open = False
                    line_position = stream.tell()
                    line = stream.readline()
                    if not line:
                        time.sleep(self.poll_period_s)
                        continue
                    if not line.endswith("\n"):
                        # The writer may still be appending this JSONL record.
                        stream.seek(line_position)
                        time.sleep(self.poll_period_s)
                        continue
                    self._handle_line(line)
                except FileNotFoundError:
                    if stream is not None:
                        stream.close()
                        stream = None
                        current_identity = None
                    time.sleep(max(0.2, self.poll_period_s))
                except Exception as exc:
                    with self._lock:
                        self._invalid += 1
                    self.adapter.get_logger().warning(f"radar log read error: {exc}")
                    time.sleep(self.poll_period_s)
        finally:
            if stream is not None:
                stream.close()

    def _handle_line(self, line: str) -> None:
        outer = json.loads(line)
        if not isinstance(outer, dict):
            raise ValueError("mission log line must be an object")
        if outer.get("log_version") != 1:
            raise ValueError("unsupported or missing mission log version")
        log_seq = outer.get("log_seq")
        if isinstance(log_seq, bool) or not isinstance(log_seq, int) or log_seq < 1:
            raise ValueError("log_seq must be a positive integer")
        record = outer.get("record")
        if not isinstance(record, dict):
            raise ValueError("mission log record is missing")
        if record.get("schema_version") != 1:
            raise ValueError("unsupported or missing radar schema version")
        if record.get("record_type") != "radar_frame":
            return
        header = record.get("header")
        if not isinstance(header, dict):
            raise ValueError("radar header must be an object")
        payload = record.get("payload")
        if not isinstance(payload, dict):
            raise ValueError("radar payload must be an object")
        complete = payload.get("complete")
        if not isinstance(complete, bool):
            raise ValueError("payload.complete must be a boolean")
        if not complete:
            with self._lock:
                self._incomplete += 1
            return
        raw_points = payload.get("points")
        if not isinstance(raw_points, list):
            raise ValueError("radar points must be a list")
        if len(raw_points) > self.max_points:
            raise ValueError(f"radar point count exceeds {self.max_points}")
        if not all(isinstance(item, dict) for item in raw_points):
            raise ValueError("every radar point must be an object")
        points = [self._convert_point(item) for item in raw_points]
        stamp = self.adapter.get_clock().now().to_msg()
        # Use the configured ROS/URDF frame. The source frame ID is metadata from
        # another coordinate system and may not exist in the ROS TF tree.
        frame_id = self.frame_id
        self.adapter.publish_points(self._point_cloud(points, stamp, frame_id))
        if self.publish_grid:
            self.adapter.publish_map(self._occupancy_grid(points, stamp, frame_id))
        with self._lock:
            self._records += 1
            self._last_points = len(points)
            self._last_record_monotonic = time.monotonic()

    @staticmethod
    def _convert_point(item: dict[str, Any]) -> tuple[float, float, float, float, float, float]:
        raw_x = float(item["x_m"])
        raw_y = float(item["y_m"])
        raw_z = float(item["z_m"])
        velocity = float(item["radial_velocity_mps"])
        snr_raw = item.get("snr_db")
        noise_raw = item.get("noise_db")
        snr = 0.0 if snr_raw is None else float(snr_raw)
        noise = 0.0 if noise_raw is None else float(noise_raw)
        values = (raw_y, -raw_x, raw_z, velocity, snr, noise)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("radar point contains a non-finite value")
        return values

    def _point_cloud(self, points, stamp, frame_id: str) -> PointCloud2:
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        names = ("x", "y", "z", "radial_velocity", "snr", "noise")
        msg.fields = []
        for index, name in enumerate(names):
            field = PointField()
            field.name = name
            field.offset = index * 4
            field.datatype = PointField.FLOAT32
            field.count = 1
            msg.fields.append(field)
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<ffffff", *point) for point in points)
        return msg

    def _occupancy_grid(self, points, stamp, frame_id: str) -> OccupancyGrid:
        width = max(1, math.ceil(self.grid_forward_range / self.grid_resolution))
        height = max(1, math.ceil(self.grid_lateral_width / self.grid_resolution))
        # UNKNOWN elsewhere: absence of a radar return is not free space.
        data = [-1] * (width * height)
        lateral_origin = -self.grid_lateral_width / 2.0
        for x_forward, y_left, _z, _velocity, _snr, _noise in points:
            ix = int(x_forward / self.grid_resolution)
            iy = int((y_left - lateral_origin) / self.grid_resolution)
            if 0 <= ix < width and 0 <= iy < height:
                data[iy * width + ix] = 100
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.info.map_load_time = stamp
        msg.info.resolution = float(self.grid_resolution)
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = lateral_origin
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def diagnostic_items(self) -> dict[str, str]:
        with self._lock:
            age = (
                "never"
                if self._last_record_monotonic is None
                else f"{time.monotonic() - self._last_record_monotonic:.2f}"
            )
            return {
                "mode": "hansel_mesh_mission_log",
                "mission_log_path": str(self.path),
                "records_published": str(self._records),
                "invalid_records": str(self._invalid),
                "incomplete_frames_skipped": str(self._incomplete),
                "last_point_count": str(self._last_points),
                "max_points_per_frame": str(self.max_points),
                "last_record_age_s": age,
                "axis_mapping": "ros_x=raw_y,ros_y=-raw_x,ros_z=raw_z",
            }
