#!/usr/bin/env python3
"""Operator UDP client that sends directly to mesh node IPs."""

import argparse
import json
import select
import socket
import sys
import termios
import time
import tty
from typing import Optional


COMMANDS = {
    "w": "forward",
    "s": "backward",
    "a": "left",
    "d": "right",
    "x": "stop",
    " ": "stop",
    "fl": "forward_left",
    "fr": "forward_right",
    "bl": "backward_left",
    "br": "backward_right",
    "mfl": "mild_forward_left",
    "mfr": "mild_forward_right",
    "mbl": "mild_backward_left",
    "mbr": "mild_backward_right",
    "sf": "slow_forward",
    "sb": "slow_backward",
    "hu": "head_servo_up",
    "hd": "head_servo_down",
    "hc": "head_servo_center",
    "hmin": "head_servo_min",
    "hmax": "head_servo_max",
    "front": "front_motor_forward",
    "front_back": "front_motor_backward",
    "front_stop": "front_motor_stop",
    "detach": "detach_press",
    "detach_rest": "detach_rest",
}

TARGETS = {
    "head": "192.168.50.10",
    "node1": "192.168.50.11",
    "node2": "192.168.50.12",
}


LIVE_KEYS = {
    "w": "forward",
    "s": "backward",
    "a": "left",
    "d": "right",
    "x": "stop",
    " ": "stop",
    "e": "forward_right",
    "q": "forward_left",
    "c": "backward_right",
    "z": "backward_left",
}


ONE_SHOT_LIVE_KEYS = {
    "u": "head_servo_up",
    "j": "head_servo_down",
    "k": "head_servo_center",
    "f": "front_motor_forward",
    "v": "front_motor_stop",
    "1": "detach_press",
    "2": "detach_rest",
}

HEAD_ONLY_COMMANDS = {
    "head_servo_up",
    "head_servo_down",
    "head_servo_center",
    "head_servo_min",
    "head_servo_max",
    "servo_up",
    "servo_down",
    "servo_center",
    "servo_min",
    "servo_max",
    "front_motor_forward",
    "front_motor_backward",
    "front_motor_stop",
    "front_forward",
    "front_backward",
    "front_stop",
}

TARGET_SPECIFIC_COMMANDS = {
    "detach_press",
    "detach_rest",
}

FORWARD_STEERING_COMMANDS = {
    "forward_left",
    "forward_right",
    "mild_forward_left",
    "mild_forward_right",
}

BACKWARD_STEERING_COMMANDS = {
    "backward_left",
    "backward_right",
    "mild_backward_left",
    "mild_backward_right",
}

HEAD_SPIN_COMMANDS = {
    "left",
    "right",
}

STRAIGHT_ALL_COMMANDS = {
    "forward",
    "backward",
    "stop",
    "slow_forward",
    "slow_backward",
}


def target_items(target: str, command: str):
    if target == "all" and command in TARGET_SPECIFIC_COMMANDS:
        print(f"[SAFE SKIP] command={command} requires target head/node1/node2, not all")
        return []
    if target == "all" and command in HEAD_ONLY_COMMANDS:
        return [("head", TARGETS["head"], command)]
    if target == "all" and command in FORWARD_STEERING_COMMANDS:
        return [
            ("head", TARGETS["head"], command),
            ("node1", TARGETS["node1"], "slow_forward"),
            ("node2", TARGETS["node2"], "slow_forward"),
        ]
    if target == "all" and command in BACKWARD_STEERING_COMMANDS:
        return [
            ("head", TARGETS["head"], command),
            ("node1", TARGETS["node1"], "slow_backward"),
            ("node2", TARGETS["node2"], "slow_backward"),
        ]
    if target == "all" and command in HEAD_SPIN_COMMANDS:
        return [
            ("head", TARGETS["head"], command),
            ("node1", TARGETS["node1"], "stop"),
            ("node2", TARGETS["node2"], "stop"),
        ]
    if target == "all":
        if command not in STRAIGHT_ALL_COMMANDS:
            print(f"[SAFE SKIP] command={command} is not allowed for all")
            return []
        return [(name, ip, command) for name, ip in TARGETS.items()]
    return [(target, TARGETS[target], command)]


def send_one(sock: socket.socket, ip: str, port: int, seq: int, target: str, command: str, speed: Optional[float]) -> None:
    message = {
        "seq": seq,
        "target": target,
        "command": command,
        "source": "operator",
        "time": time.time(),
    }
    if speed is not None:
        message["speed"] = speed
    sock.sendto(json.dumps(message).encode("utf-8"), (ip, port))
    print(f"sent seq={seq} target={target} ip={ip} command={command}")


def send(sock: socket.socket, port: int, seq: int, target: str, command: str, speed: Optional[float]) -> None:
    for name, ip, routed_command in target_items(target, command):
        send_one(sock, ip, port, seq, name, routed_command, speed)


def run_line_mode(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = args.target
    seq = 1

    print("End-to-end mesh control client")
    print("commands: w=forward s=backward a=left d=right x=stop quit=quit")
    print("extra: fl/fr/bl/br, hu/hd/hc/hmin/hmax, front/front_stop, detach/detach_rest")
    print("change target: t head | t node1 | t node2 | t all")

    while True:
        raw = input(f"[{target}]> ").strip()
        if not raw:
            continue
        if raw in {"q", "quit"}:
            break
        if raw.startswith("t "):
            next_target = raw.split(maxsplit=1)[1]
            if next_target != "all" and next_target not in TARGETS:
                print(f"unknown target: {next_target}")
                continue
            target = next_target
            continue

        command = COMMANDS.get(raw, raw)
        for _ in range(args.repeat):
            send(sock, args.port, seq, target, command, args.speed)
            time.sleep(args.repeat_delay)
        seq += 1

    return 0


def run_live_mode(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = args.target
    seq = 1
    active_command = "stop"
    last_send = 0.0
    old_settings = termios.tcgetattr(sys.stdin)

    print("Live mesh control")
    print("drive: w/s, steer head only: a/d/q/e/z/c, stop: x or space, quit: Ctrl-C")
    print("one-shot: u=head up, j=head down, k=head center, f=front motor, v=front stop, 1=detach, 2=detach rest")
    print(f"target={target} speed={args.speed if args.speed is not None else 'role default'}")

    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], args.send_interval)
            if ready:
                key = sys.stdin.read(1)
                if key == "\x03":
                    raise KeyboardInterrupt
                if key in LIVE_KEYS:
                    active_command = LIVE_KEYS[key]
                    send(sock, args.port, seq, target, active_command, args.speed)
                    seq += 1
                    last_send = time.monotonic()
                elif key in ONE_SHOT_LIVE_KEYS:
                    send(sock, args.port, seq, target, ONE_SHOT_LIVE_KEYS[key], args.speed)
                    seq += 1

            now = time.monotonic()
            if active_command != "stop" and now - last_send >= args.send_interval:
                send(sock, args.port, seq, target, active_command, args.speed)
                seq += 1
                last_send = now

    except KeyboardInterrupt:
        print()
        send(sock, args.port, seq, target, "stop", args.speed)
        return 0
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def run(args: argparse.Namespace) -> int:
    if args.live:
        return run_live_mode(args)
    return run_line_mode(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send UDP commands directly to mesh node IPs.")
    parser.add_argument("--target", default="head", choices=sorted(TARGETS) + ["all"], help="initial target")
    parser.add_argument("--port", type=int, default=7000, help="node UDP control port")
    parser.add_argument("--speed", type=float, default=None, help="0.0-1.0 speed scale sent with drive commands")
    parser.add_argument("--repeat", type=int, default=2, help="line mode UDP repeat count")
    parser.add_argument("--repeat-delay", type=float, default=0.01, help="seconds between line mode repeats")
    parser.add_argument("--live", action="store_true", help="send current key command repeatedly for driving")
    parser.add_argument("--send-interval", type=float, default=0.1, help="live mode send interval")
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
