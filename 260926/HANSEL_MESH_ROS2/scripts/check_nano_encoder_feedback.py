#!/usr/bin/env python3
"""Opt-in, wheels-raised Nano test. Never run beside a motor controller.

The existing Nano bridge protocol is required. Both rear motors receive 20%
PWM for about one second; front follow is disabled. No PID or servo commands
are used. Physical power isolation must remain accessible: software cannot
guarantee a stop after cable loss, SIGKILL, or a firmware/hardware failure.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid


PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
PWM = 20.0
PULSE_SECONDS = 1.0
STALE_SECONDS = 0.2


def parse_telemetry(line):
    fields = next(csv.reader([line]))
    if not fields or fields[0] != "T":
        return None
    if len(fields) != 4:
        raise ValueError("unexpected Nano telemetry layout: " + repr(line))
    values = tuple(float(value) for value in fields[1:])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite Nano telemetry")
    return values


def check_port_free(port):
    # Root is required by main so fuser can see other users' open descriptors.
    result = subprocess.run(
        ["fuser", "-v", str(port)], capture_output=True, text=True, timeout=3,
    )
    if result.returncode != 1 or result.stdout.strip() or result.stderr.strip():
        raise RuntimeError(
            "Nano port is busy or ownership could not be checked. Stop the "
            "unit controller first.\n" + result.stdout + result.stderr
        )


def send(ser, command):
    payload = (command + "\n").encode("ascii")
    if ser.write(payload) != len(payload):
        raise RuntimeError("short serial write")


def stop_motors(ser):
    sent = False
    for _ in range(3):
        try:
            send(ser, "X")
            sent = True
        except Exception:
            pass
    return sent


def run_probe(ser, emit, clock=time.monotonic, sleep=time.sleep):
    start = clock()
    pending = bytearray()
    rows = []
    last_valid = None

    def observe(seconds, phase, command=None):
        nonlocal last_valid
        deadline = clock() + seconds
        next_send = clock()
        while clock() < deadline:
            now = clock()
            if command is not None:
                if last_valid is None or now - last_valid > STALE_SECONDS:
                    raise RuntimeError("Nano telemetry stale; pulse aborted")
                if now >= next_send:
                    send(ser, command)
                    next_send = now + 0.1
            chunk = ser.readline()
            if not chunk:
                continue
            pending.extend(chunk)
            if len(pending) > 1024:
                raise RuntimeError("oversized/unterminated Nano serial line")
            if not pending.endswith(b"\n"):
                continue
            raw = bytes(pending).decode("ascii").strip()
            pending.clear()
            row = {"phase": phase, "elapsed_s": clock() - start, "raw": raw}
            # Save the original wire message even when decoding its fields fails.
            emit(row)
            values = parse_telemetry(raw)
            if values is not None:
                last_valid = clock()
                row.update(left_cps=values[0], right_cps=values[1])
                rows.append(row)
            elif raw and (phase == "pulse" or raw != "E,ready"):
                raise RuntimeError("unexpected Nano response: " + repr(raw))

    try:
        send(ser, "X")
        sleep(2.2)  # Opening the CH340 port can reset the Nano.
        send(ser, "X")
        ser.reset_input_buffer()
        send(ser, "Q")
        observe(0.8, "baseline")
        if len(rows) < 3 or last_valid is None or clock() - last_valid > STALE_SECONDS:
            raise RuntimeError("No fresh Nano T,left,right,angle stream; no pulse sent")
        if any(abs(row[side]) > 1.0 for row in rows[-3:]
               for side in ("left_cps", "right_cps")):
            raise RuntimeError("Encoder motion at baseline; no pulse sent")
        emit({"event": "pulse_start", "pwm_percent": PWM,
              "duration_s": PULSE_SECONDS, "front_follow": False})
        observe(PULSE_SECONDS, "pulse", "M,20.000,20.000,0")
    finally:
        if not stop_motors(ser):
            raise RuntimeError("STOP write failed. ISOLATE MOTOR POWER NOW.")

    # Motor output is already stopped before this observation or any summary I/O.
    observe(0.4, "stopped")
    moving = [row for row in rows if row["phase"] == "pulse"]
    result = {"event": "summary", "pulse_samples": len(moving)}
    for side in ("left", "right"):
        counts = [abs(row[side + "_cps"]) for row in moving]
        result[side + "_nonzero_samples"] = sum(value > 0 for value in counts)
        result[side + "_max_abs_cps"] = max(counts, default=0.0)
    emit(result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-pulse", action="store_true",
                        help="request an interactive 20%%, one-second rear motor test")
    parser.add_argument("--log", default="/tmp/nano_encoder_check.jsonl")
    args = parser.parse_args(argv)
    if not args.run_pulse:
        print("No port opened and no motion sent. Use --run-pulse on Head to opt in.")
        return 0
    ser = None
    try:
        if socket.gethostname().split(".")[0] != "head":
            raise RuntimeError("Run on Head, not on the laptop or Base")
        if os.geteuid() != 0:
            raise RuntimeError("Run with sudo so all serial port owners can be checked")
        if not sys.stdin.isatty():
            raise RuntimeError("An interactive Head terminal is required")
        import fcntl
        import termios
        import serial
        from serial.tools.list_ports import comports

        port = Path(PORT).resolve(strict=True)
        if not any(Path(p.device).resolve() == port and p.vid == 0x1A86 and p.pid == 0x7523
                   for p in comports()):
            raise RuntimeError("Expected CH340 Nano not found at the stable port")
        check_port_free(port)
        print("Stop keyboard control AND unit_controller; do not restart them during this test.")
        print("Secure robot with ALL wheels clear. Keep hands away and power cutoff accessible.")
        print("This tests REAR wheels only: equal 20% PWM for about 1 second, then STOP.")
        print("No automatic retry or PWM increase. Nano bridge M/T/X firmware is required.")
        if input("Type PULSE to confirm the bench is ready: ").strip() != "PULSE":
            print("Cancelled. No port opened.")
            return 0
        check_port_free(port)
        run_id = uuid.uuid4().hex
        with Path(args.log).open("a", encoding="utf-8") as log:
            def emit(row):
                entry = dict(row, run_id=run_id)
                log.write(json.dumps(entry) + "\n")
                log.flush()
                if "event" in row:
                    print(json.dumps(row), flush=True)

            emit({"event": "start", "port": str(port), "wall_time": time.time()})
            ser = serial.Serial(str(port), 115200, timeout=0.02,
                                write_timeout=0.1, exclusive=True)
            # Block subsequent non-root opens even from clients not using flock.
            fcntl.ioctl(ser.fileno(), termios.TIOCEXCL)
            try:
                run_probe(ser, emit)
            except BaseException as exc:
                stop_motors(ser)
                emit({"event": "aborted", "error": str(exc) or type(exc).__name__})
                raise
        print("Log: " + args.log)
        print("Zero counts are inconclusive if the corresponding rear wheel did not rotate.")
        print("Do not resume normal PID driving until the missing feedback is resolved.")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print("[ERROR] " + (str(exc) or type(exc).__name__), file=sys.stderr)
        return 1
    finally:
        if ser is not None:
            try:
                if not stop_motors(ser):
                    print("STOP NOT CONFIRMED: ISOLATE MOTOR POWER.", file=sys.stderr)
            finally:
                ser.close()


def interrupted(signum, frame):
    raise KeyboardInterrupt("signal " + str(signum))


if __name__ == "__main__":
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    raise SystemExit(main())
