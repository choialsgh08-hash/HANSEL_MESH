"""RQT parameter editor and read-only status dashboard.

Live operation is keyboard-only.  The only editable values are the four values
explicitly requested by the operator: straight RPM, turn RPM, head up limit,
and head down limit.  PID, PWM, encoder, steering-ratio, servo-step, and pulse
calibration values remain fixed in YAML/code and are not exposed here.
"""

from __future__ import annotations

from functools import partial

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding.QtCore import QObject, Signal, Qt
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rqt_gui_py.plugin import Plugin

from sensor_msgs.msg import CompressedImage

from hansel_interfaces.msg import (
    ActiveChain, CameraReceiveStatus, HeadAngleState, NetworkStatus, UnitState, WheelState,
)


STATE_NAMES = {
    UnitState.INITIALIZING: "INITIALIZING",
    UnitState.STOPPED: "STOPPED",
    UnitState.ACTIVE: "ACTIVE",
    UnitState.DETACHING: "DETACHING",
    UnitState.RELAY_ASSUMED: "RELAY_ASSUMED",
    UnitState.ESTOP: "ESTOP",
    UnitState.FAULT: "FAULT",
}


class UiSignals(QObject):
    event = Signal(str)
    status = Signal(str)
    network = Signal(str)
    chain = Signal(str)
    camera_image = Signal(object)
    camera_status = Signal(str)


class HanselPanel(Plugin):
    def __init__(self, context) -> None:
        super().__init__(context)
        self.setObjectName("HanselPanel")
        self._node = context.node
        self._units = ["head", "node1", "node2", "node3"]
        self._unit_lines: dict[str, str] = {}
        self._wheel_lines: dict[str, str] = {}
        self._network_lines: dict[str, str] = {}
        self._subscriptions = []
        self._signals = UiSignals()
        self._parameter_clients = {
            unit: AsyncParameterClient(self._node, f"/hansel/{unit}/unit_controller")
            for unit in self._units
        }
        self._build_ui()
        self._wire_ros()
        context.add_widget(self._widget)

    @staticmethod
    def _spin(minimum, maximum, step, value, decimals=1, suffix=""):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _build_ui(self) -> None:
        self._widget = QWidget()
        root = QVBoxLayout(self._widget)

        notice = QLabel(
            "Operation is keyboard-only. RQT changes only Straight RPM, Turn RPM, "
            "Head up limit, and Head down limit."
        )
        notice.setWordWrap(True)
        notice.setStyleSheet("font-weight:bold;padding:6px;background:#e0f2fe;")
        root.addWidget(notice)

        drive_box = QGroupBox("Drive parameters")
        drive_form = QFormLayout(drive_box)
        self.straight_rpm = self._spin(1.0, 1000.0, 1.0, 120.0, 1, " RPM")
        self.turn_rpm = self._spin(1.0, 1000.0, 1.0, 102.0, 1, " RPM")
        drive_form.addRow("Straight RPM", self.straight_rpm)
        drive_form.addRow("Turn RPM", self.turn_rpm)
        apply_drive = QPushButton("Apply RPM")
        apply_drive.clicked.connect(self._apply_rpm)
        drive_form.addRow(apply_drive)
        root.addWidget(drive_box)

        head_box = QGroupBox("Head angle limits")
        head_form = QFormLayout(head_box)
        self.head_up_limit = self._spin(0.0, 180.0, 1.0, 180.0, 1, " deg")
        self.head_down_limit = self._spin(0.0, 180.0, 1.0, 180.0, 1, " deg")
        head_form.addRow("Up limit", self.head_up_limit)
        head_form.addRow("Down limit", self.head_down_limit)
        head_note = QLabel(
            "Down limit is entered as a positive magnitude. Example: Up 120 / Down 90 "
            "sets the logical range to -90 deg through +120 deg."
        )
        head_note.setWordWrap(True)
        head_form.addRow(head_note)
        apply_head = QPushButton("Apply head limits")
        apply_head.clicked.connect(self._apply_head_limits)
        head_form.addRow(apply_head)
        root.addWidget(head_box)

        camera_box = QGroupBox("Camera video")
        camera_layout = QVBoxLayout(camera_box)
        self._camera_view = QLabel("Waiting for /hansel/camera/image/compressed")
        self._camera_view.setAlignment(Qt.AlignCenter)
        self._camera_view.setMinimumSize(640, 360)
        self._camera_view.setStyleSheet(
            "background:#111827;color:#d1d5db;border:1px solid #374151;"
        )
        self._camera_status = QLabel("Camera status: waiting")
        camera_layout.addWidget(self._camera_view, 1)
        camera_layout.addWidget(self._camera_status)
        root.addWidget(camera_box, 2)

        self._chain = QLabel("Active chain: waiting")
        root.addWidget(self._chain)
        status_grid = QGridLayout()
        self._status = QPlainTextEdit(); self._status.setReadOnly(True)
        self._network = QPlainTextEdit(); self._network.setReadOnly(True)
        self._events = QPlainTextEdit(); self._events.setReadOnly(True)
        self._events.setMaximumBlockCount(300)
        status_grid.addWidget(QLabel("Unit / wheel state"), 0, 0)
        status_grid.addWidget(QLabel("Network"), 0, 1)
        status_grid.addWidget(QLabel("Events / diagnostics"), 0, 2)
        status_grid.addWidget(self._status, 1, 0)
        status_grid.addWidget(self._network, 1, 1)
        status_grid.addWidget(self._events, 1, 2)
        root.addLayout(status_grid, 1)

        self._signals.event.connect(self._events.appendPlainText)
        self._signals.status.connect(self._status.setPlainText)
        self._signals.network.connect(self._network.setPlainText)
        self._signals.chain.connect(self._chain.setText)
        self._signals.camera_image.connect(self._set_camera_image)
        self._signals.camera_status.connect(self._camera_status.setText)

    def _set_parameters(self, target: str, parameters: list[Parameter], label: str) -> None:
        client = self._parameter_clients[target]
        if not client.services_are_ready():
            self._signals.event.emit(f"{label}: parameter service unavailable for {target}")
            return
        future = client.set_parameters(parameters)
        future.add_done_callback(partial(self._parameter_result, label=label, target=target))

    def _parameter_result(self, future, *, label: str, target: str) -> None:
        try:
            response = future.result()
            failed = [result.reason or "rejected" for result in response.results if not result.successful]
            if failed:
                self._signals.event.emit(f"{label} rejected by {target}: {'; '.join(failed)}")
            else:
                self._signals.event.emit(f"{label} applied to {target}")
        except Exception as exc:
            self._signals.event.emit(f"{label} failed for {target}: {exc}")

    def _apply_rpm(self) -> None:
        straight = float(self.straight_rpm.value())
        turn = float(self.turn_rpm.value())
        parameters = [
            Parameter("straight_rpm", Parameter.Type.DOUBLE, straight),
            Parameter("turn_rpm", Parameter.Type.DOUBLE, turn),
        ]
        for unit in self._units:
            self._set_parameters(unit, parameters, "RPM parameters")

    def _apply_head_limits(self) -> None:
        up_limit = float(self.head_up_limit.value())
        down_limit = float(self.head_down_limit.value())
        if up_limit <= 0.0 or down_limit <= 0.0:
            self._signals.event.emit("Head up/down limits must both be greater than 0 deg")
            return
        parameters = [
            Parameter("head_min_angle_deg", Parameter.Type.DOUBLE, -down_limit),
            Parameter("head_center_angle_deg", Parameter.Type.DOUBLE, 0.0),
            Parameter("head_max_angle_deg", Parameter.Type.DOUBLE, up_limit),
        ]
        self._set_parameters("head", parameters, "head angle limits")

    def _wire_ros(self) -> None:
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions.append(self._node.create_subscription(
            ActiveChain, "/hansel/system/state/active_chain", self._on_chain, latched_qos))
        self._subscriptions.append(self._node.create_subscription(
            NetworkStatus, "/hansel/network/status", self._on_network, 10))
        self._subscriptions.append(self._node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10))
        self._subscriptions.append(self._node.create_subscription(
            HeadAngleState, "/hansel/head/state/front_angle", self._on_head, 10))
        self._subscriptions.append(self._node.create_subscription(
            CompressedImage, "/hansel/camera/image/compressed", self._on_camera_image, 2))
        self._subscriptions.append(self._node.create_subscription(
            CameraReceiveStatus, "/hansel/camera/receive_status", self._on_camera_status, 10))
        for unit in self._units:
            self._subscriptions.append(self._node.create_subscription(
                UnitState, f"/hansel/{unit}/state/unit", partial(self._on_unit, unit), 10))
            self._subscriptions.append(self._node.create_subscription(
                WheelState, f"/hansel/{unit}/state/wheels", partial(self._on_wheel, unit), 10))

    def _on_camera_image(self, msg: CompressedImage) -> None:
        self._signals.camera_image.emit(bytes(msg.data))

    def _on_camera_status(self, msg: CameraReceiveStatus) -> None:
        state = "receiving" if msg.receiving else "waiting"
        self._signals.camera_status.emit(
            f"Camera status: {state} | {msg.receive_fps:.1f} FPS | "
            f"loss {msg.loss_rate * 100.0:.2f}% | {msg.bitrate_bps} bps"
        )

    def _set_camera_image(self, payload: object) -> None:
        image = QImage.fromData(bytes(payload))
        if image.isNull():
            self._camera_view.setText("Invalid compressed camera frame")
            return
        pixmap = QPixmap.fromImage(image)
        self._camera_view.setPixmap(
            pixmap.scaled(
                self._camera_view.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def _render_status(self) -> None:
        lines = []
        for unit in self._units:
            lines.append(f"{unit}: {self._unit_lines.get(unit, 'waiting')}")
            lines.append(f"  {self._wheel_lines.get(unit, 'wheel state waiting')}")
        self._signals.status.emit("\n".join(lines))

    def _on_chain(self, msg: ActiveChain) -> None:
        self._signals.chain.emit("Active chain: " + " -> ".join(msg.active_drive_units))

    def _on_unit(self, unit: str, msg: UnitState) -> None:
        self._unit_lines[unit] = (
            f"{STATE_NAMES.get(msg.operation_state, msg.operation_state)} | {msg.status_message}"
        )
        self._render_status()

    def _on_wheel(self, unit: str, msg: WheelState) -> None:
        self._wheel_lines[unit] = (
            f"cmd={msg.command} target={msg.target_left_rpm:.1f}/{msg.target_right_rpm:.1f} RPM "
            f"actual={msg.actual_left_rpm:.1f}/{msg.actual_right_rpm:.1f} "
            f"PWM={msg.pwm_left:.1f}/{msg.pwm_right:.1f}"
        )
        self._render_status()

    def _on_head(self, msg: HeadAngleState) -> None:
        self._signals.event.emit(f"Head logical angle: {msg.commanded_angle_deg:.1f} deg")

    def _on_network(self, msg: NetworkStatus) -> None:
        metrics = ", ".join(f"{item.key}={item.value}" for item in msg.metrics)
        self._network_lines[msg.unit_id] = (
            f"{msg.unit_id}: {'available' if msg.data_available else 'unavailable'} "
            f"next={msg.next_hop or '-'} {metrics}"
        )
        self._signals.network.emit(
            "\n".join(self._network_lines[key] for key in sorted(self._network_lines))
        )

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.level >= DiagnosticStatus.WARN:
                self._signals.event.emit(f"{status.name}: {status.message}")

    def shutdown_plugin(self) -> None:
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)

    def save_settings(self, plugin_settings, instance_settings) -> None:
        del plugin_settings
        instance_settings.set_value("straight_rpm", self.straight_rpm.value())
        instance_settings.set_value("turn_rpm", self.turn_rpm.value())
        instance_settings.set_value("head_up_limit", self.head_up_limit.value())
        instance_settings.set_value("head_down_limit", self.head_down_limit.value())

    def restore_settings(self, plugin_settings, instance_settings) -> None:
        del plugin_settings
        for name, default in (
            ("straight_rpm", 120.0),
            ("turn_rpm", 102.0),
            ("head_up_limit", 180.0),
            ("head_down_limit", 180.0),
        ):
            widget = getattr(self, name)
            try:
                widget.setValue(float(instance_settings.value(name, default)))
            except (TypeError, ValueError):
                widget.setValue(default)
