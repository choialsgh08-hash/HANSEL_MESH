from queue import Queue, Empty
import threading
from types import SimpleNamespace as NS
import time

import pytest
import _paths  # noqa: F401
from hansel_unit_control.hardware import NanoSerialHardwareAdapter
from hansel_unit_control.rpi_gpio import RaspberryPiHardwareAdapter


def test_nano_waits_for_matching_completion_and_reads_moving_telemetry():
    adapter = NanoSerialHardwareAdapter.__new__(NanoSerialHardwareAdapter)
    adapter._detach_done = threading.Event()
    adapter._detach_token = 1
    adapter._state_lock = threading.Lock()
    adapter._stop = threading.Event()
    adapter._invalid_lines = 0
    adapter._head_moving = False
    incoming = Queue()
    def read():
        try:
            return incoming.get(timeout=.01)
        except Empty:
            return b""
    adapter._serial = NS(readline=read)
    sent = []
    adapter._send = sent.append
    worker = threading.Thread(target=adapter._reader_loop)
    worker.start()
    try:
        adapter.trigger_detach(.35, 1515, 800)
        assert sent == ["D,350,1515,800,2"]
        incoming.put(b"D,1,done\n")
        with pytest.raises(TimeoutError):
            adapter.wait_detach(.02)
        incoming.put(b"T,0,0,25,1\n")
        incoming.put(b"D,2,done\n")
        adapter.wait_detach(.5)
        assert adapter.head_moving
        adapter.trigger_detach(.35, 1515, 800)
        incoming.put(b"D,3,busy\n")
        with pytest.raises(RuntimeError, match="busy"):
            adapter.wait_detach(.5)
    finally:
        adapter._stop.set()
        worker.join(timeout=1)


def gpio_adapter(*, fail_press=False):
    adapter = RaspberryPiHardwareAdapter.__new__(RaspberryPiHardwareAdapter)
    adapter._lock = threading.RLock()
    adapter._stop = threading.Event()
    adapter._closed = adapter._detach_busy = False
    adapter._servo = 6
    adapter._outputs = [6]
    adapter._callbacks = []
    adapter._zero = lambda: None
    pulses = []
    def pulse(pin, value):
        assert pin == 6
        pulses.append(value)
        if fail_press and value == 562:
            raise OSError("servo output failed")
        return 0
    adapter._pi = NS(set_servo_pulsewidth=pulse, write=lambda *args: 0, stop=lambda: None)
    return adapter, pulses


def test_gpio_completes_without_returning_or_disabling_press():
    adapter, pulses = gpio_adapter()
    adapter.trigger_detach(.05, 562, 1038)
    assert adapter.detach_busy and pulses == [562]
    adapter.wait_detach(.5)
    assert not adapter.detach_busy
    assert pulses == [562]
    adapter.trigger_detach(.05, 562, 1038)
    adapter.wait_detach(.5)
    assert pulses == [562, 562]
    adapter.close()
    assert pulses == [562, 562, 0]


def test_gpio_press_failure_does_not_report_completion():
    adapter, pulses = gpio_adapter(fail_press=True)
    with pytest.raises(OSError, match="output failed"):
        adapter.trigger_detach(.05, 562, 1038)
    assert not adapter.detach_busy
    assert pulses == [562]


def test_gpio_close_interrupts_press_without_returning():
    adapter, pulses = gpio_adapter()
    adapter.trigger_detach(.2, 562, 1038)
    adapter.close()
    with pytest.raises(RuntimeError, match="interrupted"):
        adapter.wait_detach(.5)
    assert not adapter.detach_busy
    assert pulses == [562, 0]
