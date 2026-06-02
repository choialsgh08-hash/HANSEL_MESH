#!/usr/bin/env python3
"""End-to-end UDP control server for a mesh node."""

import argparse
import json
import socket
import time

try:
    from robot.motor_driver import build_robot_controller
except ImportError:
    from motor_driver import build_robot_controller


def apply_command(controller, role: str, command: str, message: dict) -> None:
    print(
        f"[{role}] apply command={command} seq={message.get('seq')} "
        f"source={message.get('source', 'unknown')}"
    )
    controller.handle_command(command, message)


def run(args: argparse.Namespace) -> int:
    controller = build_robot_controller(args.role, dry_run=args.dry_run)
    try:
        controller.start()
    except Exception as exc:
        print(f"[{args.role}] failed to start motor controller: {exc}")
        print(f"[{args.role}] check: sudo, RPi.GPIO, encoder wiring, and duplicate GPIO pins")
        return 2

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.1)

    last_seen = 0.0
    stopped = True

    print(f"[{args.role}] listening on {args.host}:{args.port}")
    print(f"[{args.role}] timeout: {args.timeout}s")
    print(f"[{args.role}] dry_run: {args.dry_run}")

    try:
        while True:
            now = time.monotonic()
            if last_seen and not stopped and now - last_seen > args.timeout:
                apply_command(controller, args.role, "stop", {"seq": "timeout", "source": "watchdog"})
                stopped = True

            try:
                data, peer = sock.recvfrom(4096)
            except socket.timeout:
                continue

            try:
                message = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                message = {
                    "command": data.decode(errors="ignore").strip(),
                    "source": str(peer),
                }

            command = str(message.get("command", "stop"))
            last_seen = time.monotonic()
            stopped = command == "stop"
            print(f"[{args.role}] packet from={peer} raw={message}")
            apply_command(controller, args.role, command, message)

    except KeyboardInterrupt:
        print()
        print(f"[{args.role}] KeyboardInterrupt detected.")

    finally:
        controller.stop()
        sock.close()

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive end-to-end UDP control over BATMAN mesh.")
    parser.add_argument("--role", required=True, help="head, node1, or node2")
    parser.add_argument("--host", default="0.0.0.0", help="bind address")
    parser.add_argument("--port", type=int, default=7000, help="UDP control port")
    parser.add_argument("--timeout", type=float, default=0.5, help="seconds before automatic stop")
    parser.add_argument("--dry-run", action="store_true", help="log commands without touching GPIO")
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
