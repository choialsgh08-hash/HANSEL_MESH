"""RQT runtime parameter editor; running controllers are the source of truth."""

from __future__ import annotations

from functools import partial
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding.QtCore import QObject, Signal, Qt, QTimer, QEvent
from python_qt_binding.QtGui import QCursor
from python_qt_binding.QtWidgets import (
    QApplication,
    QAbstractSpinBox,
    QDoubleSpinBox,
    QLineEdit,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rqt_gui_py.plugin import Plugin

from hansel_interfaces.msg import (
    ActiveChain, HeadAngleState, NetworkStatus, UnitState, WheelState,
)

from .runtime_configuration import Topology, RuntimeConfiguration, MOTOR_NAMES
from .parameter_endpoint import ParameterEndpoint
from .rqt_input import RqtInput


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
    chain_state = Signal(object)
    drive_state = Signal(object)


class HanselPanel(Plugin):
    def __init__(self, context) -> None:
        super().__init__(context)
        self.setObjectName("HanselPanel")
        self._node = context.node
        self._subscriptions = []
        self._parameter_clients = {}
        self._timer = None
        self._closing = False
        self._app = None
        self._input = None
        self._drive_states = {}
        try:
            path = Path(get_package_share_directory("hansel_bringup")) / "config/robot_chain.yaml"
            self._topology = Topology.load(path)
        except Exception as exc:
            self._widget = QWidget()
            layout = QVBoxLayout(self._widget)
            label = QLabel(f"Cannot load robot_chain.yaml: {exc}. Check the sourced hansel_bringup installation.")
            label.setWordWrap(True)
            layout.addWidget(label)
            context.add_widget(self._widget)
            return
        self._units = self._topology.units
        self._head_unit = self._topology.head
        self._unit_lines: dict[str, str] = {}
        self._wheel_lines: dict[str, str] = {}
        self._network_lines: dict[str, str] = {}
        self._subscriptions = []
        self._signals = UiSignals()
        self._parameter_clients = {
            unit: ParameterEndpoint(self._node, unit)
            for unit in self._units
        }
        self._configuration = RuntimeConfiguration(self._topology, self._parameter_clients,
                                                   write_guard=self._configuration_guard)
        self._input = RqtInput(self._node, self._configuration, self._signals.event.emit)
        self._dirty_fields = set()
        self._shown_revision = -1
        self._last_operation_result = ""
        self._updating_fields = False
        self._next_refresh = 0.0
        self._build_ui()
        self._signals.chain_state.connect(self._accept_chain)
        self._signals.drive_state.connect(self._accept_drive_state)
        self._wire_ros()
        context.add_widget(self._widget)
        self._app = QApplication.instance()
        self._app.installEventFilter(self)
        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._poll_configuration)
        self._timer.start(100)
        self._configuration.refresh()
        self._render_configuration()

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
        self._widget.setFocusPolicy(Qt.StrongFocus)
        self._widget.setMouseTracking(True)
        root = QVBoxLayout(self._widget)

        notice = QLabel(
            "Hover this panel in the active window to use keys. Hold W/S/A/D/Q/E/Z/C to drive; "
            "release to stop. U/J/K: head, 1..9: configured rear detach, !: E-stop, R: clear. "
            "G: Enable, B: Disable. Numeric editing does not drive. Settings require Drive Disabled."
        )
        notice.setWordWrap(True)
        notice.setStyleSheet("font-weight:bold;padding:6px;background:#e0f2fe;")
        root.addWidget(notice)
        self._enable_drive = QPushButton("Enable active drive (G)")
        self._enable_drive.clicked.connect(lambda: self._input.set_enabled(True))
        self._disable_drive = QPushButton("Disable active drive (B)")
        self._disable_drive.clicked.connect(lambda: self._input.set_enabled(False))
        root.addWidget(self._enable_drive)
        root.addWidget(self._disable_drive)


        drive_box = QGroupBox("Drive parameters")
        drive_form = QFormLayout(drive_box)
        self.straight_rpm = QLineEdit()
        self.turn_rpm = QLineEdit()
        self.maximum_pwm_percent = QLineEdit()
        for name in MOTOR_NAMES[:3]:
            field = getattr(self, name)
            field.setPlaceholderText("Waiting for controller values")
            field.textEdited.connect(partial(self._mark_dirty, name))
        drive_form.addRow("Straight RPM", self.straight_rpm)
        drive_form.addRow("Turn RPM", self.turn_rpm)
        drive_form.addRow("Max PWM Limit (%)", self.maximum_pwm_percent)
        self._apply_drive = QPushButton("Apply RPM to active units")
        self._apply_drive.clicked.connect(self._apply_rpm)
        drive_form.addRow(self._apply_drive)
        self._apply_pwm = QPushButton("Apply Max PWM Limit to active units")
        self._apply_pwm.clicked.connect(self._apply_pwm_limit)
        drive_form.addRow(self._apply_pwm)
        refresh = QPushButton("Read current values / discard edits")
        refresh.clicked.connect(self._refresh_parameters)
        drive_form.addRow(refresh)
        self._retry_rollback = QPushButton("Retry failed rollback")
        self._retry_rollback.clicked.connect(self._retry_parameters)
        drive_form.addRow(self._retry_rollback)
        note = QLabel("Max PWM Limit caps motor output; it does not scale RPM. "
                      "Mixed values are shown per unit below. Enter the requested common value explicitly.")
        note.setWordWrap(True)
        drive_form.addRow(note)
        self._parameter_values = QPlainTextEdit()
        self._parameter_values.setReadOnly(True)
        self._parameter_values.setMaximumHeight(150)
        drive_form.addRow("Current node values", self._parameter_values)
        self._configuration_status = QLabel("Waiting for ActiveChain and controller parameters")
        self._configuration_status.setWordWrap(True)
        drive_form.addRow(self._configuration_status)
        root.addWidget(drive_box)

        head_box = QGroupBox(f"Head angle limits ({self._head_unit})")
        head_form = QFormLayout(head_box)
        self.head_up_limit = self._spin(0.0, 180.0, 1.0, 0.0, 1, " deg")
        self.head_down_limit = self._spin(0.0, 180.0, 1.0, 0.0, 1, " deg")
        head_form.addRow("Up limit", self.head_up_limit)
        head_form.addRow("Down limit", self.head_down_limit)
        head_note = QLabel(
            "Down limit is entered as a positive magnitude. Example: Up 120 / Down 90 "
            "sets the logical range to -90 deg through +120 deg."
        )
        head_note.setWordWrap(True)
        head_form.addRow(head_note)
        self._apply_head = QPushButton("Apply head limits")
        self._apply_head.clicked.connect(self._apply_head_limits)
        head_form.addRow(self._apply_head)
        self.head_up_limit.valueChanged.connect(partial(self._mark_dirty, "head_limits"))
        self.head_down_limit.valueChanged.connect(partial(self._mark_dirty, "head_limits"))
        root.addWidget(head_box)

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

    def _accept_drive_state(self, state):
        if not self._closing:
            unit, enabled = state
            self._drive_states[unit] = (bool(enabled), time.monotonic())

    def _configuration_guard(self, targets):
        if self._input is not None and self._input.busy:
            return "Wait for the pending drive/detach request"
        # Settings are edited only while the whole current drive chain is disabled.
        targets = self._configuration.active or targets
        now = time.monotonic()
        unknown = [u for u in targets if u not in self._drive_states
                   or now - self._drive_states[u][1] > 1.0]
        if unknown:
            return "Fresh drive state required: " + ", ".join(unknown)
        enabled = [u for u in targets if self._drive_states[u][0]]
        return "Disable drive before editing: " + ", ".join(enabled) if enabled else ""

    @staticmethod
    def _is_editor(widget):
        # Spin boxes contain QLineEdit children; readonly status text is not an editor.
        return isinstance(widget, (QLineEdit, QAbstractSpinBox))

    def _keyboard_focus(self):
        if (self._closing or not self._configuration.active or not self._widget.isVisible()
                or not self._widget.window().isActiveWindow()):
            return False
        if not self._widget.rect().contains(self._widget.mapFromGlobal(QCursor.pos())):
            return False
        return not self._is_editor(QApplication.focusWidget())

    def eventFilter(self, watched, event):
        if self._input is None or self._closing:
            return False
        kind = event.type()
        if kind == QEvent.WindowDeactivate:
            self._input.session.focus(False)
            return False
        belongs = watched is self._widget or (isinstance(watched, QWidget) and self._widget.isAncestorOf(watched))
        if belongs and kind in (QEvent.Enter, QEvent.MouseButtonPress):
            if not self._is_editor(watched) and (kind == QEvent.MouseButtonPress
                                                or not self._is_editor(QApplication.focusWidget())):
                if self._widget.window().isActiveWindow():
                    self._widget.setFocus(Qt.MouseFocusReason)
        active = self._keyboard_focus()
        self._input.session.focus(active)
        if not active or kind not in (QEvent.KeyPress, QEvent.KeyRelease):
            return False
        key = event.text().lower()
        if not key and 32 <= event.key() <= 126:
            key = chr(event.key()).lower()
        if event.key() == Qt.Key_Escape:
            self._input.stop()
            return True
        recognized = bool(key) and key in "wsadqezcx ujkfv123456789!rgbp"
        if recognized and event.isAutoRepeat():
            return True
        if kind == QEvent.KeyRelease:
            self._input.session.release(key)
            return recognized
        return self._input.key(key)

    def _mark_dirty(self, name, *_):
        if not self._updating_fields:
            self._dirty_fields.add(name)

    def _apply_values(self, names):
        try:
            values = {name: float(getattr(self, name).text()) for name in names}
        except ValueError:
            self._configuration.last_result = "Enter numeric requested values; blank or mixed fields cannot be applied"
        else:
            self._configuration.apply(values)
        self._render_configuration()

    def _apply_rpm(self):
        self._apply_values(("straight_rpm", "turn_rpm"))

    def _apply_pwm_limit(self):
        self._apply_values(("maximum_pwm_percent",))

    def _apply_head_limits(self):
        self._configuration.apply({
            "head_min_angle_deg": -float(self.head_down_limit.value()),
            "head_center_angle_deg": 0.0,
            "head_max_angle_deg": float(self.head_up_limit.value()),
        }, head_only=True)
        self._render_configuration()

    def _accept_chain(self, state):
        if self._closing:
            return
        ordered, active, relay = state
        before = self._configuration.revision
        self._configuration.set_chain(ordered, active, relay)
        if self._configuration.revision != before:
            self._dirty_fields.clear()
            self._next_refresh = 0.0
        self._render_configuration()

    def _refresh_parameters(self):
        self._dirty_fields.clear()
        self._configuration.refresh()
        self._render_configuration()

    def _retry_parameters(self):
        self._configuration.retry_rollback()
        self._render_configuration()

    def _poll_configuration(self):
        self._input.poll()
        self._input.session.focus(self._keyboard_focus())
        self._input.session.tick()
        model = self._configuration
        was_busy = model.busy
        was_mutating = model.stage not in ("idle", "refresh")
        model.poll()
        if was_busy and not model.busy:
            # Leave an idle window even when an inactive unit's read timed out.
            self._next_refresh = time.monotonic() + 2.0
        if was_mutating and not model.busy:
            self._dirty_fields.clear()
        if not model.busy and not model.recovery_required and time.monotonic() >= self._next_refresh:
            model.refresh()
        self._render_configuration()

    def _render_configuration(self):
        model = self._configuration
        active = model.active or ()
        ready = bool(active) and all(u in model.current and u not in model.read_errors for u in active)
        guard = self._configuration_guard(active) if active else "Waiting for ActiveChain"
        editable = ready and not guard and not model.recovery_required and model.stage in ("idle", "refresh")
        self._updating_fields = True
        try:
            for name in MOTOR_NAMES[:3]:
                field = getattr(self, name)
                field.setEnabled(editable)
                if name not in self._dirty_fields:
                    value = model.common_value(name)
                    field.setText(repr(float(value)) if value is not None else "")
                    field.setPlaceholderText("Mixed values — enter requested value" if ready else "Waiting for controller values")
            head = model.current.get(self._head_unit, {})
            head_ready = (self._head_unit in active and bool(head) and self._head_unit not in model.read_errors
                          and not self._configuration_guard((self._head_unit,)))
            # Head settings come from the configured Head role, not a literal unit ID.
            for field in (self.head_up_limit, self.head_down_limit):
                field.setEnabled(head_ready and not model.busy and not model.recovery_required)
            if "head_limits" not in self._dirty_fields:
                self.head_up_limit.setValue(head.get("head_max_angle_deg", 0.0))
                self.head_down_limit.setValue(-head.get("head_min_angle_deg", 0.0))
            self._apply_drive.setEnabled(editable and not model.busy)
            self._apply_pwm.setEnabled(editable and not model.busy)
            self._apply_head.setEnabled(head_ready and not model.busy and not model.recovery_required)
            self._retry_rollback.setEnabled(model.recovery_required and not model.busy)
            self._enable_drive.setEnabled(bool(active) and not self._input.busy and not model.busy and not model.recovery_required)
            self._disable_drive.setEnabled(bool(active) and not self._input.busy)
        finally:
            self._updating_fields = False
        if self._shown_revision != model.revision:
            lines = []
            for unit in self._units:
                membership = "active" if unit in active else ("waiting for ActiveChain" if model.active is None else "inactive")
                values = model.current.get(unit)
                numbers = ", ".join(f"{name}={values[name]:g}" for name in MOTOR_NAMES) if values else model.read_errors.get(unit, "not read")
                lines.append(f"{unit} ({self._topology.roles[unit]}, {membership}): {numbers}")
            self._parameter_values.setPlainText("\n".join(lines))
            self._shown_revision = model.revision
        state = model.chain_error or ("Active targets: " + (", ".join(active) or "none"))
        if guard:
            state += " | " + guard
        if model.busy:
            state += " | " + model.stage
        self._configuration_status.setText(state + "\n" + model.last_result)
        if model.last_result and model.last_result != self._last_operation_result:
            self._signals.event.emit(model.last_result)
            self._last_operation_result = model.last_result

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
            HeadAngleState, f"/hansel/{self._head_unit}/state/front_angle", self._on_head, 10))
        for unit in self._units:
            self._subscriptions.append(self._node.create_subscription(
                UnitState, f"/hansel/{unit}/state/unit", partial(self._on_unit, unit), 10))
            self._subscriptions.append(self._node.create_subscription(
                WheelState, f"/hansel/{unit}/state/wheels", partial(self._on_wheel, unit), 10))

    def _render_status(self) -> None:
        lines = []
        for unit in self._units:
            lines.append(f"{unit}: {self._unit_lines.get(unit, 'waiting')}")
            lines.append(f"  {self._wheel_lines.get(unit, 'wheel state waiting')}")
        self._signals.status.emit("\n".join(lines))

    def _on_chain(self, msg: ActiveChain) -> None:
        self._signals.chain.emit("Active chain: " + " -> ".join(msg.active_drive_units))
        self._signals.chain_state.emit((tuple(msg.ordered_units), tuple(msg.active_drive_units), tuple(msg.relay_assumed_units)))

    def _on_unit(self, unit: str, msg: UnitState) -> None:
        self._signals.drive_state.emit((unit, msg.active_for_drive))
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
        self._closing = True
        if self._app is not None:
            self._app.removeEventFilter(self)
        if self._input is not None:
            self._input.close()
        if self._timer is not None:
            self._timer.stop()
        model = getattr(self, "_configuration", None)
        if model and (model.recovery_required or model.stage not in ("idle", "refresh")):
            self._node.get_logger().error(
                "RQT closed before configuration completion. Check actual node values; "
                f"stage={model.stage}, result={model.last_result}, snapshots={model._snapshots}"
            )
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        for endpoint in self._parameter_clients.values():
            endpoint.close()

    def save_settings(self, plugin_settings, instance_settings) -> None:
        # Runtime actuator values belong to the ROS nodes, never to GUI preferences.
        pass

    def restore_settings(self, plugin_settings, instance_settings) -> None:
        # Ignore old RPM/limit preferences. The startup query populates the controls.
        pass
