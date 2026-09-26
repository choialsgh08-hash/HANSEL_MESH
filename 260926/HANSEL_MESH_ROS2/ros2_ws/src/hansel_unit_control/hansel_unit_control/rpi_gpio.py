"""Rear-unit GPIO I/O via local pigpiod; all pin numbers are BCM.

No ROS nodes, command routing, drive-enable latch or detach ACK policy live here.
The controller's update(..., drive_enabled, dt) contract is unchanged.
"""
from __future__ import annotations

import math
import threading
import time

from .hardware import HardwareAdapter


GPIO_DEFAULTS = {
    'gpio_pigpio_port': 8888,
    'gpio_left_pwm': 18, 'gpio_left_in1': 23, 'gpio_left_in2': 24,
    'gpio_right_pwm': 13, 'gpio_right_in1': 27, 'gpio_right_in2': 22,
    'gpio_left_encoder_a': 20, 'gpio_left_encoder_b': 21,
    'gpio_right_encoder_a': 26, 'gpio_right_encoder_b': 16,
    'gpio_detach_servo': 6,
    'gpio_pwm_frequency': 1000,
    'gpio_left_motor_sign': 1, 'gpio_right_motor_sign': 1,
    'gpio_left_encoder_sign': 1, 'gpio_right_encoder_sign': 1,
    'gpio_encoder_edges_per_count': 4,
    'gpio_encoder_pull': 'up',
    'gpio_detach_rest_s': 0.12,
}
PIN_KEYS = ('left_pwm', 'left_in1', 'left_in2', 'right_pwm', 'right_in1',
            'right_in2', 'left_encoder_a', 'left_encoder_b',
            'right_encoder_a', 'right_encoder_b', 'detach_servo')


def checked(result):
    """Also handle pigpio installations with global exceptions disabled."""
    if isinstance(result, int) and result < 0:
        raise RuntimeError(f'pigpio returned error {result}')
    return result


def validate_options(options):
    defaults = {key.removeprefix('gpio_'): value for key, value in GPIO_DEFAULTS.items()}
    unknown = set(options) - set(defaults)
    if unknown:
        raise ValueError('Unknown GPIO options: ' + ', '.join(sorted(unknown)))
    p = {**defaults, **options}
    pins = [p[key] for key in PIN_KEYS]
    if any(type(pin) is not int or not 2 <= pin <= 27 for pin in pins):
        raise ValueError('Pins must be BCM integers 2..27, not physical header numbers')
    if len(set(pins)) != len(pins):
        raise ValueError('Motor, encoder and servo pins must be unique')
    if type(p['pigpio_port']) is not int or not 1024 <= p['pigpio_port'] <= 32000:
        raise ValueError('pigpio_port must be an integer in 1024..32000')
    if type(p['pwm_frequency']) is not int or not 10 <= p['pwm_frequency'] <= 8000:
        raise ValueError('PWM frequency must be an integer in 10..8000 Hz')
    for key in ('left_motor_sign', 'right_motor_sign', 'left_encoder_sign', 'right_encoder_sign'):
        if type(p[key]) is not int or p[key] not in (-1, 1):
            raise ValueError(f'{key} must be +1 or -1')
    if type(p['encoder_edges_per_count']) is not int or p['encoder_edges_per_count'] not in (1, 2, 4):
        raise ValueError('encoder_edges_per_count must be 1, 2 or 4')
    if p['encoder_pull'] not in ('up', 'down', 'none'):
        raise ValueError('encoder_pull must be up, down or none')
    if not math.isfinite(p['detach_rest_s']) or not .02 <= p['detach_rest_s'] <= 1:
        raise ValueError('detach_rest_s must be 0.02..1 seconds')
    return p


def connect_local(port):
    try:
        import pigpio
    except ImportError as exc:
        raise RuntimeError('Install python3-pigpio in the same Python environment as ROS') from exc
    pi = pigpio.pi('127.0.0.1', port)
    if not pi.connected:
        pi.stop()
        raise RuntimeError(f'Cannot connect to local pigpiod on port {port}; '
                           'start it with sudo pigpiod -l (or the pigpiod service)')
    return pigpio, pi


class _Quadrature:
    # Positive sequence AB: 00 -> 01 -> 11 -> 10 -> 00.
    _DELTA = (0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0)

    def __init__(self, a, b):
        self.state = (a << 1) | b
        self.count = self.invalid = 0
        self._tick = None
        self._base = self.state
        self._delta = self._invalid = 0

    def edge(self, channel, level, tick):
        if level not in (0, 1):  # pigpio TIMEOUT is not an edge.
            return
        # Notifications arrive in order. Equality groups a single sampled bank;
        # no increasing-tick assumption, so the 32-bit microsecond wrap is valid.
        if tick != self._tick:
            self._tick, self._base = tick, self.state
            self._delta = self._invalid = 0
        mask = 2 if channel == 0 else 1
        self.state = (self.state & ~mask) | (level * mask)
        delta = self._DELTA[(self._base << 2) | self.state]
        invalid = int((self._base ^ self.state) == 3)
        # Both channels changing in the same sample have unknown direction.
        # Undo the partial edge rather than inventing two counts from callback order.
        self.count += delta - self._delta
        self.invalid += invalid - self._invalid
        self._delta, self._invalid = delta, invalid


class RaspberryPiHardwareAdapter(HardwareAdapter):
    def __init__(self, *, motor_pwm_limit_percent=40.0, **options):
        p = validate_options(options)
        self._lock = threading.RLock()
        self._encoder_lock = threading.Lock()
        self._stop = threading.Event()
        self._closed = False
        self._pi = None
        self._outputs = []
        self._callbacks = []
        self._thread = None
        self._detach_busy = False
        self._servo_error = ''
        self._last_error = ''
        self._motors = ((p['left_pwm'], p['left_in1'], p['left_in2']),
                        (p['right_pwm'], p['right_in1'], p['right_in2']))
        self._encoders = ((p['left_encoder_a'], p['left_encoder_b']),
                          (p['right_encoder_a'], p['right_encoder_b']))
        self._servo = p['detach_servo']
        self._motor_signs = (p['left_motor_sign'], p['right_motor_sign'])
        self._encoder_signs = (p['left_encoder_sign'], p['right_encoder_sign'])
        self._divider = p['encoder_edges_per_count']
        self._rest_s = float(p['detach_rest_s'])
        self._port = p['pigpio_port']
        self._last_pwm = [0., 0.]
        self._last_counts = (0, 0)
        self._actual_frequencies = []
        self.set_motor_pwm_limit(motor_pwm_limit_percent)
        try:
            self._pigpio, self._pi = connect_local(self._port)
            for key in PIN_KEYS:
                mode = checked(self._pi.get_mode(p[key]))
                if mode not in (self._pigpio.INPUT, self._pigpio.OUTPUT):
                    raise RuntimeError(f'BCM{p[key]} is assigned to an alternate function; check overlays')
            # Stop PWM before touching H-bridge direction pins.
            output_pins = [p['left_pwm'], p['right_pwm'], p['left_in1'], p['left_in2'],
                           p['right_in1'], p['right_in2'], self._servo]
            for pin in output_pins:
                self._outputs.append(pin)
                if pin in (p['left_pwm'], p['right_pwm']):
                    checked(self._pi.set_PWM_dutycycle(pin, 0))
                if pin == self._servo:
                    checked(self._pi.set_servo_pulsewidth(pin, 0))
                checked(self._pi.write(pin, 0))
                checked(self._pi.set_mode(pin, self._pigpio.OUTPUT))
            for pwm, _, _ in self._motors:
                checked(self._pi.set_PWM_range(pwm, 10000))
                self._actual_frequencies.append(checked(self._pi.set_PWM_frequency(pwm, p['pwm_frequency'])))
            pull = {'up': self._pigpio.PUD_UP, 'down': self._pigpio.PUD_DOWN,
                    'none': self._pigpio.PUD_OFF}[p['encoder_pull']]
            for pair in self._encoders:
                for pin in pair:
                    checked(self._pi.set_mode(pin, self._pigpio.INPUT))
                    checked(self._pi.set_pull_up_down(pin, pull))
            bank = checked(self._pi.read_bank_1())
            self._decoders = [_Quadrature((bank >> a) & 1, (bank >> b) & 1) for a, b in self._encoders]
            for side, pair in enumerate(self._encoders):
                for channel, pin in enumerate(pair):
                    self._callbacks.append(self._pi.callback(pin, self._pigpio.EITHER_EDGE,
                                                             self._callback(side, channel)))
            self._last_sample = time.monotonic()
        except Exception:
            self.close()
            raise

    def _callback(self, side, channel):
        def on_edge(gpio, level, tick):
            with self._encoder_lock:
                if not self._stop.is_set():
                    self._decoders[side].edge(channel, level, tick)
        return on_edge

    def set_motor_pwm_limit(self, limit_percent):
        limit = float(limit_percent)
        if not math.isfinite(limit) or not 0 < limit <= 100:
            raise ValueError('Motor PWM limit must be >0 and <=100 percent')
        with self._lock:
            self.motor_pwm_limit_percent = limit

    def _zero(self):
        errors = []
        for pwm, in1, in2 in self._motors:
            for method, pin, value in ((self._pi.set_PWM_dutycycle, pwm, 0),
                                       (self._pi.write, pwm, 0),
                                       (self._pi.write, in1, 0), (self._pi.write, in2, 0)):
                if pin in self._outputs:
                    try:
                        checked(method(pin, value))
                    except Exception as exc:
                        errors.append(str(exc))
        self._last_pwm = [0., 0.]
        if errors:
            raise RuntimeError('; '.join(errors))

    def _apply_motor(self, side, logical_pwm):
        pwm, in1, in2 = self._motors[side]
        value = logical_pwm * self._motor_signs[side]
        if not value or self._last_pwm[side] * value <= 0:
            checked(self._pi.set_PWM_dutycycle(pwm, 0))
            checked(self._pi.write(in1, 0))
            checked(self._pi.write(in2, 0))
        if value:
            checked(self._pi.write(in1, int(value > 0)))
            checked(self._pi.write(in2, int(value < 0)))
            checked(self._pi.set_PWM_dutycycle(pwm, round(abs(value) * 100)))
        self._last_pwm[side] = value

    def update(self, left_pwm, right_pwm, drive_enabled, dt):
        del dt  # Use time since the actual sample, including service-triggered updates.
        with self._lock:
            if self._closed:
                raise RuntimeError('GPIO adapter is closed')
            try:
                if self._servo_error:
                    raise RuntimeError(self._servo_error)
                values = (float(left_pwm), float(right_pwm))
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('Motor PWM must be finite')
                limit = self.motor_pwm_limit_percent
                for side, value in enumerate(values):
                    self._apply_motor(side, max(-limit, min(limit, value)) if drive_enabled else 0)
                with self._encoder_lock:
                    now = time.monotonic()
                    elapsed = now - self._last_sample
                    counts = tuple(d.count for d in self._decoders)
                    cps = tuple((new - old) * sign / self._divider / elapsed
                                for new, old, sign in zip(counts, self._last_counts, self._encoder_signs)
                                ) if elapsed > 0 else (0., 0.)
                    self._last_counts, self._last_sample = counts, now
                return cps
            except Exception as exc:
                self._last_error = str(exc)
                try:
                    self._zero()  # Do not leave a half-applied pair of motor outputs.
                except Exception:
                    pass
                raise  # The original controller applies its existing error policy.

    def configure_servo(self, **kwargs):
        pass  # Shared head calibration has no actuator on a rear unit.

    def set_head_angle(self, angle_deg):
        raise ValueError('Rear GPIO unit has no head servo')

    def set_front_motor(self, command, speed_scale):
        raise ValueError('Rear GPIO unit has no front motor')

    @property
    def detach_busy(self):
        with self._lock:
            return self._detach_busy

    def trigger_detach(self, duration_s, press_pulse_us, rest_pulse_us):
        duration = float(duration_s)
        if not math.isfinite(duration) or not .05 <= duration <= 2:
            raise ValueError('Detach duration must be 0.05..2 seconds')
        if any(type(p) is not int or not 500 <= p <= 2500 for p in (press_pulse_us, rest_pulse_us)):
            raise ValueError('Servo pulse widths must be integers in 500..2500 us')
        with self._lock:
            if self._closed or self._detach_busy:
                raise RuntimeError('GPIO closed or servo pulse sequence already active')
            self._servo_error = ''
            checked(self._pi.set_servo_pulsewidth(self._servo, press_pulse_us))
            self._detach_busy = True
            self._thread = threading.Thread(target=self._detach_worker,
                args=(duration, rest_pulse_us), daemon=True, name='hansel-detach-servo')
            try:
                self._thread.start()
            except Exception:
                self._detach_busy = False
                checked(self._pi.set_servo_pulsewidth(self._servo, 0))
                raise

    def _detach_worker(self, duration, rest):
        try:
            if self._stop.wait(duration):
                return
            with self._lock:
                if self._closed:
                    return
                checked(self._pi.set_servo_pulsewidth(self._servo, rest))
            self._stop.wait(self._rest_s)
        except Exception as exc:
            with self._lock:
                self._servo_error = str(exc)
        finally:
            with self._lock:
                if not self._closed:
                    try:
                        checked(self._pi.set_servo_pulsewidth(self._servo, 0))
                        checked(self._pi.write(self._servo, 0))
                    except Exception as exc:
                        self._servo_error = str(exc)
                self._detach_busy = False

    def diagnostic_items(self):
        with self._lock, self._encoder_lock:
            return {'backend': 'rpi_gpio', 'library': 'pigpio', 'numbering': 'BCM',
                    'pigpiod': f'127.0.0.1:{self._port}',
                    'pwm_frequency_hz': str(self._actual_frequencies),
                    'encoder_edges_per_count': str(self._divider),
                    'left_encoder_edges': str(self._decoders[0].count),
                    'right_encoder_edges': str(self._decoders[1].count),
                    'encoder_invalid_events': str(sum(d.invalid for d in self._decoders)),
                    'detach_busy': str(self._detach_busy).lower(),
                    'gpio_error': self._servo_error or self._last_error or 'none'}

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._stop.set()
            if self._pi is not None:
                try:
                    self._zero()
                except Exception:
                    pass
                if self._servo in self._outputs:
                    for method in (self._pi.set_servo_pulsewidth, self._pi.write):
                        try:
                            checked(method(self._servo, 0))
                        except Exception:
                            pass
                for callback in self._callbacks:
                    try:
                        callback.cancel()
                    except Exception:
                        pass
                self._callbacks.clear()
                self._pi.stop()
            self._detach_busy = False
        if self._thread is not None and self._thread.ident is not None:
            self._thread.join(timeout=1)
