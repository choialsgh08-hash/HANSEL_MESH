"""Execute production callbacks with in-memory ROS messages/services, no hardware."""
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import sys
import threading
from concurrent.futures import Future

import pytest
import _paths  # noqa: F401
from hansel_unit_control.core import GitHubMotionMapper, HeadServoStepper, PidController, RampLimiter, SafetyController
from hansel_unit_control.hardware import DummyHardwareAdapter
from hansel_operator.chain_registry import ChainRegistry

ROOT = Path(__file__).resolve().parents[1] / "ros2_ws/src"


@pytest.fixture
def callbacks(monkeypatch):
    class Message(NS):
        MANUAL = 0
        AUTO = 1
        UP_STEP = 1
        DOWN_STEP = 2
        CENTER = 3
        MIN_LIMIT = 4
        MAX_LIMIT = 5

    Message.Request = Message.Response = Message
    modules = {
        "rclpy": [], "rclpy.node": ["Node"],
        "rclpy.callback_groups": ["ReentrantCallbackGroup"],
        "rclpy.executors": ["MultiThreadedExecutor"],
        "rclpy.qos": ["DurabilityPolicy", "HistoryPolicy", "QoSProfile", "ReliabilityPolicy"],
        "rcl_interfaces.msg": ["SetParametersResult"],
        "diagnostic_msgs.msg": ["DiagnosticArray", "DiagnosticStatus", "KeyValue"],
        "hansel_interfaces.msg": ["ActiveChain", "DetachEvent", "DetachMode", "DetachRecommendation",
                                  "EmergencyStop", "HeadServoCommand", "HeadAngleState", "MotionCommand", "UnitState", "WheelState", "RelayPose"],
        "hansel_interfaces.srv": ["PrepareDetach", "TriggerDetach", "SetDriveEnabled", "DetachUnit"],
        "geometry_msgs.msg": ["Pose", "PoseStamped", "TransformStamped"],
        "nav_msgs.msg": ["Odometry", "Path"], "sensor_msgs.msg": ["Imu"],
        "std_srvs.srv": ["Trigger"], "tf2_ros": ["TransformBroadcaster"],
        "visualization_msgs.msg": ["Marker", "MarkerArray"],
    }
    for name, members in modules.items():
        module = ModuleType(name)
        for member in members:
            setattr(module, member, Message)
        monkeypatch.setitem(sys.modules, name, module)
    loaded = []
    for package, filename in (("hansel_unit_control", "unit_controller"),
                              ("hansel_operator", "detach_coordinator"),
                              ("hansel_operator", "command_router"),
                              ("hansel_localization", "head_odometry_node")):
        spec = importlib.util.spec_from_file_location(f"{package}.{filename}", ROOT / package / package / f"{filename}.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        loaded.append(module)
    return loaded


def logger():
    return NS(warning=lambda *_: None, error=lambda *_: None)


def make_unit(module, unit_id):
    unit = module.UnitController.__new__(module.UnitController)
    unit.unit_id = unit_id
    unit.role = "head" if unit_id == "head" else "rear"
    unit.safety = SafetyController(watchdog_requires_enable=False)
    unit.mapper = GitHubMotionMapper(unit.role)
    unit.hardware = DummyHardwareAdapter(800, 800)
    unit.left_pid = PidController(.035, .015, 0, output_limit=75)
    unit.right_pid = PidController(.035, .015, 0, output_limit=75)
    unit.left_ramp, unit.right_ramp = RampLimiter(100), RampLimiter(100)
    unit._detach_inhibit = False
    unit._detach_hold_sequence = unit._released_hold_sequence = -1
    unit._io_error = ""
    unit._hardware_retry_at = 0.0
    unit.control_period_s = .05
    unit.actual_left_cps = unit.actual_right_cps = 0
    unit._last_control_time = 0
    unit._tick_count = 0
    unit.publish_divisor = 1000
    unit._last_head_sequence_by_source = {}
    unit.head_stepper = HeadServoStepper(step_angle_deg=5)
    unit._publish_head_state = lambda: None
    unit.get_parameter = lambda name: NS(value={"detach_duration_s": .35, "detach_press_pulse_us": 1515,
                                                "detach_rest_pulse_us": 800}[name])
    unit.get_logger = logger
    unit.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    unit._force_zero_outputs()
    return unit


def make_coordinator(modules, units):
    module = modules[1]
    obj = module.DetachCoordinator.__new__(module.DetachCoordinator)
    obj.registry = ChainRegistry(["head", "node1", "node2"],
                                 {"head": "head", "node1": "rear", "node2": "rear"},
                                 ["head", "node1", "node2"])
    obj._execution_lock = threading.Lock()
    obj._release_lock = threading.Lock()
    obj._pending_releases = {}
    obj._request_sequence = 0
    obj.get_parameter = lambda name: NS(value=.1)
    obj.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=10, nanosec=0)))
    obj.get_logger = logger
    events = []
    obj.event_publisher = NS(publish=events.append)

    def client(callback):
        def call(request):
            f = Future()
            try:
                f.set_result(callback(request, NS()))
            except Exception as exc:
                f.set_exception(exc)
            return f
        return NS(service_is_ready=lambda: True, call_async=call)

    obj.stop_clients = {name: client(unit._on_prepare_detach) for name, unit in units.items()}
    obj.detach_clients = {name: client(unit._on_trigger_detach) for name, unit in units.items()}
    def publish():
        for unit in units.values():
            unit._on_active_chain(NS(relay_assumed_units=obj.registry.relay_assumed_units))
    obj._publish_chain = publish
    return obj, events


@pytest.mark.parametrize("released,remaining,actuator", [
    ("node2", ["head", "node1"], "node1"), ("node1", ["head"], "head"),
])
def test_detach_actuator_remaining_drive_and_relay_latch(callbacks, released, remaining, actuator):
    units = {name: make_unit(callbacks[0], name) for name in ("head", "node1", "node2")}
    coordinator, events = make_coordinator(callbacks, units)
    result = coordinator._perform_detach(NS(released_unit_id=released, mode=0), NS())
    assert result.command_completed
    assert units[actuator].hardware.detach_count == 1
    assert coordinator.registry.active_drive_units == remaining
    assert {e.released_unit_id for e in events} == set(units) - set(remaining)
    for name, unit in units.items():
        if name in remaining:
            assert unit.safety.drive_enabled and not unit._detach_inhibit
            assert unit.current_command == "stop"
        else:
            assert not unit.safety.request_drive_enable()[0]
            assert not unit.safety.drive_allowed


def test_missing_completion_does_not_claim_or_mark_release(callbacks):
    units = {name: make_unit(callbacks[0], name) for name in ("head", "node1", "node2")}
    coordinator, events = make_coordinator(callbacks, units)
    def fail(**kwargs):
        raise TimeoutError("missing ACK")
    units["node1"].hardware.wait_detach = fail
    result = coordinator._perform_detach(NS(released_unit_id="node2", mode=0), NS())
    assert not result.command_completed and not events
    assert units["node2"]._detach_inhibit
    assert not coordinator.registry.relay_assumed_units


def test_hold_release_preserves_manual_disable_and_reports_failed_stop(callbacks):
    unit = make_unit(callbacks[0], "head")
    unit.safety.drive_enabled = False
    unit._on_prepare_detach(NS(hold=True, request_sequence=1), NS())
    unit._on_prepare_detach(NS(hold=False, request_sequence=1), NS())
    assert not unit.safety.drive_enabled
    def fail(*args):
        raise OSError("serial gone")
    unit.hardware.update = fail
    assert not unit._on_prepare_detach(NS(hold=True, request_sequence=2), NS()).stopped


@pytest.mark.parametrize("role", ["head", "node1", "node2"])
@pytest.mark.parametrize("command,expected", [("forward_left", (75, 0)), ("forward_right", (0, 75)),
                                             ("left", (-75, 75)), ("right", (75, -75))])
def test_router_to_controller_max_pwm_ignores_speed_scale(callbacks, monkeypatch, role, command, expected):
    unit = make_unit(callbacks[0], role)
    routed = callbacks[2].CommandRouter._normalize_rear(command) if role != "head" else command
    assert routed == command
    stamp = NS(sec=10, nanosec=0)
    now = [0.]
    monkeypatch.setattr(callbacks[0].time, "monotonic", lambda: now[0])
    for i in range(25):
        unit._on_motion(NS(stamp=stamp, sequence=i, command=routed, speed_scale=.2, source="test"))
        now[0] += .05
        unit._control_tick()
    assert (unit.left_pwm, unit.right_pwm) == expected
    unit._on_motion(NS(stamp=stamp, sequence=30, command="stop", speed_scale=.2, source="test"))
    for _ in range(20):
        now[0] += .05
        unit._control_tick()
    assert unit.left_pwm == unit.right_pwm == 0


def test_head_step_and_k_center(callbacks):
    unit = make_unit(callbacks[0], "head")
    for i in range(3):
        unit._on_head_servo_command(NS(source="test", sequence=i, command=1))
    assert unit.hardware.head_angle == 15
    unit._on_head_servo_command(NS(source="test", sequence=3, command=3))
    assert unit.hardware.head_angle == 0


def test_detach_marker_freezes_event_time_not_arrival_time(callbacks):
    import math
    from hansel_localization.core import PlanarOdometry
    module = callbacks[3]
    node = module.HeadOdometryNode.__new__(module.HeadOdometryNode)
    node.odometry = PlanarOdometry(1 / math.pi, 10, right_encoder_sign=1, max_dt_s=.3)
    for i in range(41):
        node.odometry.update(10 + i * .1, i, i, 0)
    node._seen_relays, node._pending_relays, node._relay_poses = set(), [], {}
    node._zero_stamp_s = 10
    node.relay_offsets = {"node2": .4}
    node.get_logger = logger
    node._pose_message = lambda pose: pose
    node._publish_relay_markers = lambda: None
    messages = []
    node.relay_publisher = NS(publish=messages.append)
    event = NS(released_unit_id="node2", stamp=NS(sec=12, nanosec=0))
    node._on_detach_event(event)
    assert len(messages) == 1
    assert messages[0].pose.x_m == pytest.approx(1.6)
    assert messages[0].stamp.sec == 12
    node._on_detach_event(event)
    assert len(messages) == 1
    node._seen_relays.clear()
    node._zero_stamp_s = 15
    node._on_detach_event(event)
    assert len(messages) == 1  # Retained event cannot create a new drop after zero.


def test_tail_first_then_head_cut_does_not_actuate_released_unit(callbacks):
    units = {name: make_unit(callbacks[0], name) for name in ("head", "node1", "node2")}
    coordinator, events = make_coordinator(callbacks, units)
    for name in ("node2", "node1"):
        assert coordinator._perform_detach(NS(released_unit_id=name, mode=0), NS()).command_completed
    assert [e.actuator_unit_id for e in events] == ["node1", "head"]
    assert coordinator.registry.active_drive_units == ["head"]
    assert units["head"].safety.drive_enabled
    assert not coordinator._perform_detach(NS(released_unit_id="node2", mode=0), NS()).accepted
