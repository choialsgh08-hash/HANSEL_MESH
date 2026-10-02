#!/usr/bin/env python3
"""Own one process group for the lifetime of an SSH input stream."""
import argparse
import fcntl
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time


def stop_group(child, grace_s=2):
    # Descendants may outlive their immediate shell (notably camera pipelines).
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
        try:
            child.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.1)
    child.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", choices=["unit", "operator", "router", "inputs", "odometry", "map", "radar", "camera"])
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    lock = open(f"/tmp/hansel-{os.getuid()}-{args.name}.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.exit(2, f"Already managed: {args.name}\n")
    names = {"operator": "command_router", "router": "command_router", "odometry": "head_odometry", "map": "run_head_map.py",
             "radar": "run_radar_stack.py", "camera": "rpicam-vid", "unit": "unit_controller"}
    for process in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            argv = process.read_bytes().split(b"\0")
        except OSError:
            continue
        if any(b" " not in a and os.path.basename(os.fsdecode(a)) == names.get(args.name)
               for a in argv):
            parser.exit(2, f"Existing {args.name} process {process.parent.name}; leave it or stop it explicitly first.\n")
    ended = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: ended.set())
    child = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL)
    print(f"[managed] {args.name} pid={child.pid}", flush=True)
    try:
        # A buffered stdin reader thread can abort Python at child exit.
        # Poll the SSH pipe here so child failure never waits for stdin to close.
        while child.poll() is None and not ended.is_set():
            readable, _, _ = select.select([sys.stdin.fileno()], [], [], 0.2)
            if readable and not os.read(sys.stdin.fileno(), 4096):
                ended.set()
    finally:
        # Radar closes its receiver before sending sensorStop on the same UART.
        # Leave the other processes' shutdown timing unchanged.
        stop_group(child, grace_s=8 if args.name == "radar" else 2)
        lock.close()
    return child.returncode if child.returncode and not ended.is_set() else 0


if __name__ == "__main__":
    sys.exit(main())
