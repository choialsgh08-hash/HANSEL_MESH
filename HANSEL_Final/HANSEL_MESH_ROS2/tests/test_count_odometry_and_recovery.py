import math
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace as NS
from queue import Queue, Empty

import pytest
import _paths  # noqa: F401
from hansel_localization.core import PlanarOdometry
from hansel_unit_control.hardware import HardwareIOError, NanoSerialHardwareAdapter
from test_detach_drive_flow import callbacks, make_unit, make_coordinator  # noqa: F401


class Port:
    def __init__(self):
        self.sent = []
        self.is_open = True
        self.fail = False

    def write(self, data):
        if self.fail or not self.is_open:
            raise OSError("USB write failed")
        self.sent.append(data.decode().strip())

    def flush(self):
        pass

    def close(self):
        self.is_open = False


def nano():
    a = NanoSerialHardwareAdapter.__new__(NanoSerialHardwareAdapter)
    a.port_name = "/dev/serial/by-id/test-nano"
    a.baudrate = 115200
    a._serial = Port()
    a._write_lock = threading.Lock()
    a._state_lock = threading.Lock()
    a._left_cps = a._right_cps = a._head_angle = 0.
    a._last_state_monotonic = None
    a._read_error = a._last_error = ""
    a._invalid_lines = 0
    a._head_moving = False
    a._encoder_epoch = "first-boot"
    a._encoder_sequence = 0
    a._encoder_sample = a._sample_tick = None
    a._front_override = a._servo_config_line = None
    a._configuration_pending = False
    a._ready_after = 0.
    a._needs_reopen = False
    a._detach_token = 1
    a._detach_done = threading.Event()
    a.front_follow_drive = True
    a.motor_pwm_limit_percent = 75.
    return a


def motion(sequence=1, command="forward"):
    return NS(stamp=NS(sec=10, nanosec=0), sequence=sequence,
              command=command, speed_scale=.2, source="keyboard:test")


def odom():
    return PlanarOdometry(1 / math.pi, 100, right_encoder_sign=1)


@pytest.mark.parametrize("times", [(1., 1.1, 1.2), (1., 2., 20.)])
def test_distance_comes_only_from_counts_not_sampling_interval(times):
    o = odom()
    for stamp, count in zip(times, (1234, 1284, 1334)):
        o.update(stamp, count, count, 0)
    assert o.x_m == pytest.approx(1.)
    for stamp in (21., 22., 23.):
        o.update(stamp, 1334, 1334, 0)
    assert o.x_m == pytest.approx(1.)
    o.update(24, 1284, 1284, 0)
    assert o.x_m == pytest.approx(.5)


def test_counter_wrap_reboot_and_zero_do_not_move_the_origin():
    o = odom()
    o.update(1, 2**31 - 5, 2**31 - 5, 0, "boot1")
    o.update(2, -2**31 + 5, -2**31 + 5, 0, "boot1")
    assert o.x_m == pytest.approx(.1)
    o.update(3, 0, 0, 0, "boot2")
    assert o.x_m == pytest.approx(.1)
    o.update(4, 10, 10, 0, "boot2")
    assert o.x_m == pytest.approx(.2)
    o.reset()
    o.update(5, 10000, 10000, 0, "boot2")
    assert o.x_m == 0


def test_heading_gap_rebaselines_instead_of_inventing_unobserved_path():
    o = odom()
    o.update(1, 0, 0, 0)
    o.update(2, 100, 100, 0)
    o.pause_counts()
    o.update(3, 900, 900, math.pi / 2)
    o.update(4, 1000, 1000, math.pi / 2)
    assert (o.x_m, o.y_m) == pytest.approx((1., 1.))


def test_nano_real_totals_staleness_legacy_and_restart(monkeypatch):
    now = [1.]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    a = nano()
    a._consume_line("T,-200,300,5,1,-123,456,1000")
    sample = a.encoder_sample
    assert (sample.left, sample.right) == (-123, 456)
    assert a.update(0, 0, False, .05) == (-200, 300)
    a._consume_line("T,-200,300,5,1,-123,456,1000")
    assert a.encoder_sample.sequence == sample.sequence
    now[0] += .36
    assert a.encoder_sample is None
    with pytest.raises(HardwareIOError, match="stale"):
        a.update(75, 75, True, .05)
    assert a._serial.sent[-1] == "X"
    a._consume_line("B,boot")
    a._consume_line("T,0,0,0,0,0,0,50")
    assert a.encoder_sample.epoch != sample.epoch
    a._consume_line("T,99,99,0,0")
    assert a.encoder_sample is None
    assert a.diagnostic_items()["encoder_counts_valid"] == "false"


def test_tick_wrap_is_not_a_restart_but_backwards_uptime_is():
    a = nano()
    a._consume_line(f"T,0,0,0,0,10,10,{2**32-20}")
    epoch = a.encoder_sample.epoch
    a._consume_line("T,0,0,0,0,15,15,30")
    assert a.encoder_sample.epoch == epoch
    a._consume_line("T,0,0,0,0,16,16,1000")
    a._consume_line("T,0,0,0,0,0,0,50")
    assert a.encoder_sample.epoch != epoch


def test_odometry_callback_ignores_cps_and_duplicate_sample(callbacks):
    node = callbacks[3].HeadOdometryNode.__new__(callbacks[3].HeadOdometryNode)
    node.odometry = odom()
    node._latest_yaw = 0.
    node._last_imu_monotonic = time.monotonic()
    node.get_parameter = lambda _: NS(value=.35)
    published = []
    node._publish_pose = lambda stamp, pose: published.append(pose)
    node._materialize_pending_relays = lambda: None
    msg = NS(encoder_counts_valid=True, encoder_epoch="boot", encoder_sample_id=1,
             left_encoder_count=100, right_encoder_count=100,
             actual_left_cps=99999, actual_right_cps=99999, stamp=NS(sec=1, nanosec=0))
    node._on_wheels(msg)
    msg.stamp.sec = 2
    node._on_wheels(msg)
    assert len(published) == 1 and node.odometry.x_m == 0
    msg.encoder_sample_id = 2
    msg.left_encoder_count = msg.right_encoder_count = 200
    node._on_wheels(msg)
    assert node.odometry.x_m == pytest.approx(1.)
    msg.encoder_counts_valid = False
    msg.left_encoder_count = msg.right_encoder_count = 9999
    msg.stamp.sec = 3
    node._on_wheels(msg)
    assert node.odometry.x_m == pytest.approx(1.)


def test_reopen_uses_by_id_and_new_counts_epoch_without_replaying_actuators(monkeypatch):
    now = [1.]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    a = nano()
    a._consume_line("T,0,0,0,0,1000,1000,100")
    epoch = a.encoder_sample.epoch
    now[0] = 2.
    fresh = Port()
    opens = []
    def open_port(*args, **kwargs):
        opens.append((args, kwargs))
        return fresh
    monkeypatch.setattr("serial.Serial", open_port)
    with pytest.raises(HardwareIOError, match="reopened"):
        a.recover()
    assert opens[0][0][0] == a.port_name and opens[0][1]["exclusive"]
    with pytest.raises(HardwareIOError, match="startup"):
        a.recover()
    now[0] = 4.
    a._consume_line("T,0,0,0,0,0,0,100")
    a.recover()
    assert a.encoder_sample.epoch != epoch
    assert fresh.sent == ["X"]


def test_io_error_recovers_without_g_but_never_replays_old_motion(callbacks, monkeypatch):
    now = [0.]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    u = make_unit(callbacks[0], "head")
    u._on_motion(motion())
    update = u.hardware.update
    u.hardware.update = lambda *args: (_ for _ in ()).throw(OSError("write timeout"))
    u._control_tick()
    assert u._io_error and not u.safety.fault_latched
    u.hardware.update = update
    now[0] = 1.1
    u._control_tick()
    assert not u._io_error and u.current_command == "stop" and u.left_pwm == 0
    u._on_motion(motion(2))
    now[0] += .05
    u._control_tick()
    assert u.left_pwm > 0 and u.safety.drive_allowed


@pytest.mark.parametrize("restriction", ["manual", "relay", "estop"])
def test_recovery_does_not_clear_other_drive_states(callbacks, monkeypatch, restriction):
    now = [0.]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    u = make_unit(callbacks[0], "head")
    if restriction == "manual":
        u.safety.drive_enabled = False
    elif restriction == "relay":
        u.safety.set_relay_assumed()
    else:
        u.safety.set_estop(True)
    u._record_io_error(OSError("lost"))
    now[0] = 2.
    u._control_tick()
    assert not u._io_error and not u.safety.drive_allowed


def test_front_mode_survives_ticks_v_stops_and_w_restores_rears(callbacks, monkeypatch):
    monkeypatch.setattr(time, "monotonic", lambda: 0.)
    u = make_unit(callbacks[0], "head")
    a = nano()
    a._consume_line("T,0,0,0,0,0,0,10")
    u.hardware = a
    u._on_motion(motion(command="front_motor_forward"))
    for _ in range(20):
        u._control_tick()
    assert a._serial.sent and all(line == "F,1,20.000" for line in a._serial.sent)
    u._on_motion(motion(2, "front_motor_stop"))
    u._control_tick()
    assert a._serial.sent[-1] == "F,0,0.000"
    u._on_motion(motion(3, "forward"))
    u._control_tick()
    assert a._serial.sent[-1].startswith("M,")
    u._on_motion(motion(4, "stop"))
    for _ in range(100):
        u._control_tick()
    assert a._front_override is None


def test_unsent_detach_releases_all_units_and_does_not_actuate(callbacks):
    units = {name: make_unit(callbacks[0], name) for name in ("head", "node1", "node2")}
    co, events = make_coordinator(callbacks, units)
    co.detach_clients["node1"].service_is_ready = lambda: False
    result = co._perform_detach(NS(released_unit_id="node2", mode=0), NS())
    assert not result.command_completed and not events
    assert units["node1"].hardware.detach_count == 0
    assert all(not u._detach_inhibit and u.safety.drive_allowed for u in units.values())


def test_failed_release_response_is_retried_and_stale_hold_cannot_return(callbacks):
    units = {name: make_unit(callbacks[0], name) for name in ("head", "node1", "node2")}
    co, _ = make_coordinator(callbacks, units)
    client = co.stop_clients["node2"]
    call = client.call_async
    def failing_release(request):
        if request.hold:
            return call(request)
        f = Future()
        f.set_result(NS(stopped=False))
        return f
    client.call_async = failing_release
    co.detach_clients["node1"].service_is_ready = lambda: False
    co._perform_detach(NS(released_unit_id="node2", mode=0), NS())
    assert units["node2"]._detach_inhibit and "node2" in co._pending_releases
    client.call_async = call
    co._retry_stop_releases()
    co._retry_stop_releases()
    assert not units["node2"]._detach_inhibit and not co._pending_releases
    units["node2"]._on_prepare_detach(NS(hold=True, request_sequence=1), NS())
    assert not units["node2"]._detach_inhibit


def test_old_release_cannot_remove_a_new_hold(callbacks):
    u = make_unit(callbacks[0], "node2")
    u._on_prepare_detach(NS(hold=True, request_sequence=2), NS())
    u._on_prepare_detach(NS(hold=False, request_sequence=1), NS())
    assert u._detach_inhibit
    u._on_prepare_detach(NS(hold=False, request_sequence=2), NS())
    assert not u._detach_inhibit


def test_missing_detach_ack_does_not_fault_the_remaining_actuator(callbacks):
    units = {name: make_unit(callbacks[0], name) for name in ("head", "node1", "node2")}
    co, events = make_coordinator(callbacks, units)
    units["node1"].hardware.wait_detach = lambda **kw: (_ for _ in ()).throw(TimeoutError("ACK lost"))
    result = co._perform_detach(NS(released_unit_id="node2", mode=0), NS())
    assert not result.command_completed and not events
    assert units["node2"]._detach_inhibit
    assert not units["node1"]._detach_inhibit and not units["node1"].safety.fault_latched
    assert units["node1"].safety.drive_allowed


def test_out_of_order_counts_do_not_change_baseline():
    o = odom()
    o.update(1, 0, 0, 0)
    o.update(3, 100, 100, 0)
    assert o.update(2, 50, 50, 0) is None
    o.update(4, 200, 200, 0)
    assert o.x_m == pytest.approx(2.)


def test_uart_timeout_fragments_are_reassembled_before_parsing():
    a = nano()
    incoming = Queue()
    a._stop = threading.Event()
    def read():
        try:
            return incoming.get(timeout=.01)
        except Empty:
            return b""
    a._serial = NS(readline=read)
    worker = threading.Thread(target=a._reader_loop)
    worker.start()
    try:
        incoming.put(b"T,0,0,0,0,12,")
        incoming.put(b"34,100\n")
        for _ in range(50):
            if a.encoder_sample is not None:
                break
            a._stop.wait(.01)
        assert (a.encoder_sample.left, a.encoder_sample.right) == (12, 34)
    finally:
        a._stop.set()
        worker.join(timeout=1)
        assert not worker.is_alive()


def test_never_completed_hold_release_future_gets_retried(callbacks, monkeypatch):
    units = {name: make_unit(callbacks[0], name) for name in ("head", "node1", "node2")}
    co, _ = make_coordinator(callbacks, units)
    u = units["node2"]
    u._on_prepare_detach(NS(hold=True, request_sequence=1), NS())
    lost = Future()
    request = NS(hold=False, request_sequence=1)
    co._pending_releases["node2"] = (request, lost, .5)
    monkeypatch.setattr(time, "monotonic", lambda: 1.)
    co._retry_stop_releases()
    co._retry_stop_releases()
    assert lost.cancelled() and not u._detach_inhibit and not co._pending_releases


def test_gpio_retry_replaces_a_broken_connection_without_corrupting_old_state(monkeypatch):
    import hansel_unit_control.rpi_gpio as gpio
    constants = NS(INPUT=0, OUTPUT=1, PUD_UP=2, PUD_DOWN=1, PUD_OFF=0, EITHER_EDGE=2)
    class Pi:
        connected = True
        broken = False
        def get_mode(self, pin): return 0
        def read_bank_1(self): return 0
        def callback(self, *args): return NS(cancel=lambda: None)
        def stop(self): self.connected = False
        def set_PWM_dutycycle(self, *args):
            if self.broken:
                raise OSError("pigpio socket lost")
            return 0
        def __getattr__(self, name): return lambda *args: 0
    first = Pi()
    monkeypatch.setattr(gpio, "connect_local", lambda _: (constants, first))
    a = gpio.RaspberryPiHardwareAdapter()
    first.broken = True
    with pytest.raises(HardwareIOError):
        a.update(10, 10, True, .05)
    def unavailable(port): raise RuntimeError("pigpiod not ready")
    monkeypatch.setattr(gpio, "connect_local", unavailable)
    with pytest.raises(HardwareIOError):
        a.recover()
    assert a.diagnostic_items()["backend"] == "rpi_gpio"
    second = Pi()
    monkeypatch.setattr(gpio, "connect_local", lambda _: (constants, second))
    replacement = a.recover()
    try:
        assert replacement is not a and replacement.motor_pwm_limit_percent == 75
        replacement.update(0, 0, False, .05)
    finally:
        replacement.close()


def test_keyboard_f_v_are_persistent_commands(callbacks):
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "ros2_ws/src/hansel_operator/hansel_operator/operator_input.py"
    spec = importlib.util.spec_from_file_location("test_operator_input", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    operator = module.OperatorInput.__new__(module.OperatorInput)
    operator._gui_until = 0
    operator.speed_scale = .2
    operator._publish_active = lambda: None
    operator.get_logger = lambda: NS(info=lambda _: None)
    assert operator._handle_key_locked("f") and operator.active_command == "front_motor_forward"
    assert operator._handle_key_locked("v") and operator.active_command == "front_motor_stop"
    assert operator._handle_key_locked("w") and operator.active_command == "forward"
