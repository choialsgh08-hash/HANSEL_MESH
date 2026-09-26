"""Fuse rear encoder speed with BNO085 yaw and freeze relay drop poses."""

from __future__ import annotations

import copy
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from hansel_interfaces.msg import ActiveChain, RelayPose, WheelState

from .bno085 import Bno085I2cSource, Bno085RvcSource
from .core import (
    PlanarOdometry,
    Pose2D,
    YawReference,
    quaternion_yaw,
    wrap_angle,
    yaw_quaternion,
)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _parse_offsets(entries: list[str]) -> dict[str, float]:
    offsets: dict[str, float] = {}
    for entry in entries:
        unit, separator, raw_value = str(entry).partition("=")
        if not separator or not unit.strip():
            raise ValueError(f"relay offset must be unit=meters: {entry!r}")
        value = float(raw_value)
        if value < 0.0:
            raise ValueError(f"relay offset must not be negative: {entry!r}")
        offsets[unit.strip()] = value
    return offsets


class HeadOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("head_odometry")
        self._declare_parameters()
        p = self.get_parameter

        self.odom_frame = str(p("odom_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.imu_frame = str(p("imu_frame").value)
        self.publish_tf = bool(p("publish_tf").value)
        self.provisional_calibration = bool(p("provisional_calibration").value)
        self.relay_offsets = _parse_offsets(list(p("relay_offsets").value))
        self.yaw_reference = YawReference(
            sample_count=int(p("imu_zero_samples").value),
            sign=float(p("imu_yaw_sign").value),
        )
        self.odometry = PlanarOdometry(
            wheel_diameter_m=float(p("wheel_diameter_m").value),
            encoder_cpr=float(p("encoder_cpr").value),
            left_encoder_sign=float(p("left_encoder_sign").value),
            right_encoder_sign=float(p("right_encoder_sign").value),
            left_distance_scale=float(p("left_distance_scale").value),
            right_distance_scale=float(p("right_distance_scale").value),
            max_dt_s=float(p("max_wheel_dt_s").value),
            history_min_step_m=float(p("history_min_step_m").value),
            max_history_points=int(p("max_history_points").value),
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        relay_event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.imu_publisher = self.create_publisher(Imu, "/hansel/head/imu/data", 20)
        self.odom_publisher = self.create_publisher(
            Odometry, "/hansel/head/odometry", 20
        )
        self.path_publisher = self.create_publisher(
            Path, "/hansel/head/path", latched_qos
        )
        self.relay_publisher = self.create_publisher(
            RelayPose, "/hansel/localization/relay_pose", relay_event_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/hansel/localization/relay_markers", latched_qos
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.create_subscription(
            WheelState,
            "/hansel/head/state/wheels",
            self._on_wheels,
            20,
        )
        self.create_subscription(
            ActiveChain,
            "/hansel/system/state/active_chain",
            self._on_active_chain,
            latched_qos,
        )
        self.create_service(
            Trigger, "/hansel/head/localization/zero", self._on_zero
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._bno = None
        self._next_bno_connect_s = 0.0
        self._last_bno_error = "not connected"
        self._bno_consecutive_errors = 0
        self._bno_total_errors = 0
        self._bno_reconnect_count = 0
        self._bno_connected_s = 0.0
        self._last_bno_sample_s: float | None = None
        self._bno_fresh_samples = 0
        self._last_bno_warning_s = -math.inf
        self._latest_yaw: float | None = None
        self._latest_gyro_z = 0.0
        self._last_imu_monotonic: float | None = None
        self._last_wheel_monotonic: float | None = None
        self._last_left_cps = 0.0
        self._last_right_cps = 0.0
        self._path = Path()
        self._path.header.frame_id = self.odom_frame
        self._last_path_distance_m: float | None = None
        self._last_path_yaw_rad: float | None = None
        self._relay_poses: dict[str, RelayPose] = {}
        self._seen_relays: set[str] = set()
        self._pending_relays: list[str] = []
        self._imu_ready_logged = False

        imu_rate_hz = float(p("imu_rate_hz").value)
        self.create_timer(1.0 / imu_rate_hz, self._read_imu)
        self.create_timer(1.0, self._publish_diagnostics)
        wheel_diameter = float(p("wheel_diameter_m").value)
        encoder_cpr = float(p("encoder_cpr").value)
        self.get_logger().warning(
            f"starting provisional odometry: wheel_diameter={wheel_diameter:.4f} m "
            f"CPR={encoder_cpr:.1f}; calibrate distance_scale before treating "
            "positions as measured"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "odom_frame": "odom",
            "base_frame": "base_link",
            "imu_frame": "base_link",
            "publish_tf": True,
            "imu_transport": "i2c",
            "imu_serial_port": "/dev/serial0",
            "imu_rate_hz": 20.0,
            "imu_zero_samples": 30,
            "imu_yaw_sign": 1.0,
            "imu_timeout_s": 0.35,
            "imu_max_yaw_rate_rad_s": 6.0,
            "bno_reconnect_s": 2.0,
            "bno_max_consecutive_errors": 10,
            "bno_error_log_period_s": 2.0,
            "wheel_diameter_m": 0.08,
            "encoder_cpr": 400.0,
            "left_encoder_sign": 1.0,
            "right_encoder_sign": -1.0,
            "left_distance_scale": 1.0,
            "right_distance_scale": 1.0,
            "max_wheel_dt_s": 0.25,
            "history_min_step_m": 0.005,
            "max_history_points": 3000,
            "max_path_poses": 1000,
            "relay_offsets": ["node1=0.20", "node2=0.40", "node3=0.60"],
            "provisional_calibration": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _connect_bno(self) -> bool:
        now = time.monotonic()
        if self._bno is not None:
            return True
        if now < self._next_bno_connect_s:
            return False
        try:
            transport = str(self.get_parameter("imu_transport").value)
            if transport == "i2c":
                self._bno = Bno085I2cSource(
                    frequency_hz=float(self.get_parameter("imu_rate_hz").value)
                )
            elif transport == "uart_rvc":
                self._bno = Bno085RvcSource(
                    port=str(self.get_parameter("imu_serial_port").value),
                    max_age_s=float(self.get_parameter("imu_timeout_s").value),
                )
            else:
                raise ValueError(f"unknown imu_transport: {transport}")
            self._bno_connected_s = time.monotonic()
            self._last_bno_sample_s = None
            self._last_bno_error = "none"
            self._bno_consecutive_errors = 0
            self.get_logger().info(
                f"BNO085 {transport} source opened; waiting for fresh heading reports"
            )
            return True
        except Exception as exc:
            self._last_bno_error = str(exc)
            self._next_bno_connect_s = time.monotonic() + float(
                self.get_parameter("bno_reconnect_s").value
            )
            self.get_logger().error(f"BNO085 connection failed: {exc}")
            return False

    def _handle_bno_read_error(self, exc: Exception) -> None:
        now = time.monotonic()
        self._last_bno_error = repr(exc)
        self._bno_consecutive_errors += 1
        self._bno_total_errors += 1

        log_period = float(self.get_parameter("bno_error_log_period_s").value)
        if now - self._last_bno_warning_s >= log_period:
            self.get_logger().warning(
                "BNO085 transient read error "
                f"({self._bno_consecutive_errors}/"
                f"{self.get_parameter('bno_max_consecutive_errors').value}): {exc}"
            )
            self._last_bno_warning_s = now

        limit = int(self.get_parameter("bno_max_consecutive_errors").value)
        if self._bno_consecutive_errors < max(1, limit):
            return

        self.get_logger().error(
            f"BNO085 failed {self._bno_consecutive_errors} consecutive reads; reconnecting"
        )
        try:
            self._bno.close()
        except Exception:
            pass
        self._bno = None
        self._bno_reconnect_count += 1
        self._bno_consecutive_errors = 0
        self._latest_yaw = None
        self._latest_gyro_z = 0.0
        self._last_imu_monotonic = None
        current_pose = self.odometry.current_pose
        self.yaw_reference.reset(0.0 if current_pose is None else current_pose.yaw_rad)
        self._imu_ready_logged = False
        self._next_bno_connect_s = now + float(
            self.get_parameter("bno_reconnect_s").value
        )

    def _read_imu(self) -> None:
        if not self._connect_bno():
            return
        try:
            sample = self._bno.read()
            if sample is None:
                last_sample = (
                    self._bno_connected_s
                    if self._last_bno_sample_s is None
                    else self._last_bno_sample_s
                )
                if time.monotonic() - last_sample > float(
                    self.get_parameter("imu_timeout_s").value
                ):
                    raise TimeoutError("no fresh BNO085 heading report")
                return
            sample_time = time.monotonic()
            raw_yaw = quaternion_yaw(*sample.quaternion_xyzw)
            yaw = self.yaw_reference.update(raw_yaw)
            if yaw is None:
                self._accept_bno_sample(sample_time)
                return

            yaw_rate = 0.0
            if self._latest_yaw is not None and self._last_imu_monotonic is not None:
                dt = sample_time - self._last_imu_monotonic
                if dt > 0.0:
                    yaw_rate = wrap_angle(yaw - self._latest_yaw) / dt
                    max_rate = float(
                        self.get_parameter("imu_max_yaw_rate_rad_s").value
                    )
                    if abs(yaw_rate) > max_rate:
                        raise ValueError(
                            f"implausible BNO085 yaw rate: {yaw_rate:.2f} rad/s"
                        )
            self._accept_bno_sample(sample_time)
            self._latest_yaw = yaw
            self._latest_gyro_z = yaw_rate
            self._last_imu_monotonic = sample_time
            if not self._imu_ready_logged:
                self._imu_ready_logged = True
                self.get_logger().info(
                    "BNO085 zero heading ready; ROS odom +X is robot forward"
                )
            self._publish_imu(yaw)
        except Exception as exc:
            self._handle_bno_read_error(exc)

    def _accept_bno_sample(self, sample_time: float) -> None:
        self._bno_fresh_samples += 1
        self._last_bno_sample_s = sample_time
        self._bno_consecutive_errors = 0
        self._last_bno_error = "none"

    def _publish_imu(self, yaw: float) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        qx, qy, qz, qw = yaw_quaternion(yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [999.0, 0.0, 0.0, 0.0, 999.0, 0.0, 0.0, 0.0, 0.0012]
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = self._latest_gyro_z
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0025]
        msg.linear_acceleration_covariance[0] = -1.0
        self.imu_publisher.publish(msg)

    def _on_wheels(self, msg: WheelState) -> None:
        self._last_wheel_monotonic = time.monotonic()
        self._last_left_cps = float(msg.actual_left_cps)
        self._last_right_cps = float(msg.actual_right_cps)
        imu_age = (
            math.inf
            if self._last_imu_monotonic is None
            else time.monotonic() - self._last_imu_monotonic
        )
        if (
            self._latest_yaw is None
            or imu_age > float(self.get_parameter("imu_timeout_s").value)
        ):
            return
        stamp_s = _stamp_seconds(msg.stamp)
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds / 1e9
        pose = self.odometry.update(
            stamp_s,
            self._last_left_cps,
            self._last_right_cps,
            self._latest_yaw,
        )
        if pose is None:
            return
        self._publish_pose(msg.stamp, pose)
        self._materialize_pending_relays()

    def _publish_pose(self, stamp, pose: Pose2D) -> None:
        pose_msg = self._pose_message(pose)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose_msg
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.0025
        left_mps = (
            self.odometry.left_sign
            * self._last_left_cps
            * self.odometry.left_m_per_count
        )
        right_mps = (
            self.odometry.right_sign
            * self._last_right_cps
            * self.odometry.right_m_per_count
        )
        odom.twist.twist.linear.x = 0.5 * (left_mps + right_mps)
        odom.twist.twist.angular.z = self._latest_gyro_z
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.005
        self.odom_publisher.publish(odom)

        stamped = PoseStamped()
        stamped.header = odom.header
        stamped.pose = pose_msg
        max_path = int(self.get_parameter("max_path_poses").value)
        path_distance_delta = (
            float("inf")
            if self._last_path_distance_m is None
            else pose.path_distance_m - self._last_path_distance_m
        )
        path_yaw_delta = (
            float("inf")
            if self._last_path_yaw_rad is None
            else abs(wrap_angle(pose.yaw_rad - self._last_path_yaw_rad))
        )
        if path_distance_delta >= 0.01 or path_yaw_delta >= math.radians(3.0):
            self._path.poses.append(stamped)
            self._last_path_distance_m = pose.path_distance_m
            self._last_path_yaw_rad = pose.yaw_rad
            if len(self._path.poses) > max_path:
                del self._path.poses[: len(self._path.poses) - max_path]
        self._path.header.stamp = stamp
        self.path_publisher.publish(self._path)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = pose.x_m
            transform.transform.translation.y = pose.y_m
            transform.transform.rotation = pose_msg.orientation
            self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _pose_message(pose: Pose2D) -> Pose:
        msg = Pose()
        msg.position.x = pose.x_m
        msg.position.y = pose.y_m
        qx, qy, qz, qw = yaw_quaternion(pose.yaw_rad)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        return msg

    def _on_active_chain(self, msg: ActiveChain) -> None:
        for unit in msg.relay_assumed_units:
            if unit not in self._seen_relays:
                self._seen_relays.add(unit)
                self._pending_relays.append(unit)
        self._materialize_pending_relays()

    def _materialize_pending_relays(self) -> None:
        remaining: list[str] = []
        for unit in self._pending_relays:
            offset = self.relay_offsets.get(unit)
            if offset is None:
                self.get_logger().warning(f"no trajectory offset configured for {unit}")
                continue
            sample = self.odometry.sample_behind(offset)
            if sample is None:
                remaining.append(unit)
                continue
            msg = RelayPose()
            msg.stamp = self.get_clock().now().to_msg()
            msg.unit_id = unit
            msg.pose = self._pose_message(sample.pose)
            msg.head_path_distance_m = float(sample.pose.path_distance_m)
            msg.offset_behind_head_m = float(offset)
            msg.provisional = bool(sample.provisional or self.provisional_calibration)
            self._relay_poses[unit] = msg
            self.relay_publisher.publish(msg)
            self.get_logger().warning(
                f"froze {unit} pose x={sample.pose.x_m:.3f} y={sample.pose.y_m:.3f} "
                f"offset={offset:.2f} m provisional={msg.provisional}"
            )
        self._pending_relays = remaining
        if self._relay_poses:
            self._publish_relay_markers()

    def _publish_relay_markers(self) -> None:
        markers = MarkerArray()
        for index, unit in enumerate(sorted(self._relay_poses)):
            relay = self._relay_poses[unit]
            body = Marker()
            body.header.frame_id = self.odom_frame
            body.header.stamp = self.get_clock().now().to_msg()
            body.ns = "hansel_relays"
            body.id = index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose = copy.deepcopy(relay.pose)
            body.scale.x = 0.12
            body.scale.y = 0.12
            body.scale.z = 0.06
            body.color.r = 0.10
            body.color.g = 0.75
            body.color.b = 0.30
            body.color.a = 0.95
            markers.markers.append(body)

            label = Marker()
            label.header = body.header
            label.ns = "hansel_relay_labels"
            label.id = 1000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = copy.deepcopy(relay.pose)
            label.pose.position.z += 0.16
            label.scale.z = 0.10
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = unit + (" (estimate)" if relay.provisional else "")
            markers.markers.append(label)
        self.marker_publisher.publish(markers)

    def _on_zero(self, request, response):
        del request
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[clear]))
        self.yaw_reference.reset()
        self.odometry.reset()
        self._path = Path()
        self._path.header.frame_id = self.odom_frame
        self._last_path_distance_m = None
        self._last_path_yaw_rad = None
        self._latest_yaw = None
        self._relay_poses.clear()
        self._seen_relays.clear()
        self._pending_relays.clear()
        self._imu_ready_logged = False
        response.success = True
        response.message = "pose and BNO085 startup heading reset; keep Head stationary"
        return response

    @staticmethod
    def _age(last_monotonic: float | None) -> str:
        if last_monotonic is None:
            return "never"
        return f"{time.monotonic() - last_monotonic:.2f}"

    def _publish_diagnostics(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "hansel/head/odometry"
        status.hardware_id = "head-bno085-rear-encoders"
        imu_age_s = (
            math.inf
            if self._last_bno_sample_s is None
            else time.monotonic() - self._last_bno_sample_s
        )
        if self._bno is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "BNO085 unavailable"
        elif imu_age_s > float(self.get_parameter("imu_timeout_s").value):
            status.level = DiagnosticStatus.ERROR
            status.message = "BNO085 data stale"
        elif not self.yaw_reference.ready:
            status.level = DiagnosticStatus.WARN
            status.message = "collecting stationary yaw zero"
        elif self._last_wheel_monotonic is None or (
            time.monotonic() - self._last_wheel_monotonic
            > float(self.get_parameter("max_wheel_dt_s").value)
        ):
            status.level = DiagnosticStatus.ERROR
            status.message = "wheel data missing or stale; position not updating"
        elif self.provisional_calibration:
            status.level = DiagnosticStatus.WARN
            status.message = "running with unmeasured CPR/distance scale"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "odometry running"
        pose = self.odometry.current_pose
        values = {
            "imu_transport": str(self.get_parameter("imu_transport").value),
            "imu_age_s": self._age(self._last_bno_sample_s),
            "bno_fresh_samples": str(self._bno_fresh_samples),
            "wheel_age_s": self._age(self._last_wheel_monotonic),
            "imu_zero_samples": (
                f"{self.yaw_reference.collected}/{self.yaw_reference.sample_count}"
            ),
            "last_bno_error": self._last_bno_error,
            "bno_consecutive_errors": str(self._bno_consecutive_errors),
            "bno_total_errors": str(self._bno_total_errors),
            "bno_reconnect_count": str(self._bno_reconnect_count),
            "left_cps_raw": f"{self._last_left_cps:.2f}",
            "right_cps_raw": f"{self._last_right_cps:.2f}",
            "encoder_cpr_assumed": f"{self.get_parameter('encoder_cpr').value}",
            "path_distance_m": "0.000" if pose is None else f"{pose.path_distance_m:.3f}",
            "relay_pose_count": str(len(self._relay_poses)),
            "provisional": str(self.provisional_calibration).lower(),
        }
        if isinstance(self._bno, Bno085RvcSource):
            values.update({
                "rvc_valid_frames": str(self._bno.frames.valid_frames),
                "rvc_invalid_frames": str(self._bno.frames.invalid_frames),
                "rvc_backlog_drops": str(self._bno.backlog_drops),
            })
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        msg.status = [status]
        self.diagnostics_publisher.publish(msg)

    def destroy_node(self):
        if self._bno is not None:
            try:
                self._bno.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HeadOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
