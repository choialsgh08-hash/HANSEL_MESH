#!/usr/bin/env python3
"""Operator UDP client that sends directly to mesh node IPs."""

import argparse
import json
import select
import socket
import subprocess
import sys
import termios
import time
import tty
from typing import List, Optional

try:
    from controller.quality_supervisor import QualityConfig, QualitySupervisor
except ImportError:
    from quality_supervisor import QualityConfig, QualitySupervisor


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
    "1": "detach_node1",
    "2": "detach_node2",
    "3": "detach_node3",
    "detach_node1": "detach_node1",
    "detach_node2": "detach_node2",
    "detach_node3": "detach_node3",
}

TARGETS = {
    "head": "192.168.50.10",
    "node1": "192.168.50.11",
    "node2": "192.168.50.12",
    "node3": "192.168.50.13",
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
}

DETACH_RELEASE_ACTUATORS = {
    "node1": "head",
    "node2": "node1",
    "node3": "node2",
}

MANUAL_DETACH_KEYS = {
    "1": "node1",
    "2": "node2",
    "3": "node3",
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


def target_items(target: str, command: str, active_targets: Optional[List[str]] = None):
    active_names = active_targets if active_targets is not None else list(TARGETS.keys())
    active_names = [name for name in active_names if name in TARGETS]

    if target == "all" and command in TARGET_SPECIFIC_COMMANDS:
        print(f"[SAFE SKIP] command={command} requires a single target, not all")
        return []
    if target == "all" and command in HEAD_ONLY_COMMANDS:
        return [("head", TARGETS["head"], command)] if "head" in active_names else []
    if target == "all" and command in FORWARD_STEERING_COMMANDS:
        items = []
        if "head" in active_names:
            items.append(("head", TARGETS["head"], command))
        items.extend((name, TARGETS[name], "slow_forward") for name in active_names if name != "head")
        return items
    if target == "all" and command in BACKWARD_STEERING_COMMANDS:
        items = []
        if "head" in active_names:
            items.append(("head", TARGETS["head"], command))
        items.extend((name, TARGETS[name], "slow_backward") for name in active_names if name != "head")
        return items
    if target == "all" and command in HEAD_SPIN_COMMANDS:
        items = []
        if "head" in active_names:
            items.append(("head", TARGETS["head"], command))
        items.extend((name, TARGETS[name], "stop") for name in active_names if name != "head")
        return items
    if target == "all":
        if command not in STRAIGHT_ALL_COMMANDS:
            print(f"[SAFE SKIP] command={command} is not allowed for all")
            return []
        return [(name, TARGETS[name], command) for name in active_names]
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


def send(sock: socket.socket, port: int, seq: int, target: str, command: str,
         speed: Optional[float], active_targets: Optional[List[str]] = None) -> None:
    for name, ip, routed_command in target_items(target, command, active_targets):
        send_one(sock, ip, port, seq, name, routed_command, speed)


def send_direct(sock: socket.socket, port: int, seq: int, target: str,
                command: str, speed: Optional[float]) -> None:
    send_one(sock, TARGETS[target], port, seq, target, command, speed)


def send_detach_release(sock: socket.socket, port: int, seq: int, released_node: str) -> bool:
    actuator = DETACH_RELEASE_ACTUATORS.get(released_node)
    if actuator is None:
        print(f"[SAFE SKIP] no detach actuator is mapped for released node: {released_node}")
        return False
    if actuator not in TARGETS:
        print(f"[SAFE SKIP] detach actuator target is unknown: {actuator}")
        return False
    if released_node in TARGETS:
        print(f"[detach] putting released node {released_node} into relay hold before detach")
        send_direct(sock, port, seq, released_node, "relay_hold", None)
        time.sleep(0.05)
    print(f"[detach] release {released_node}: moving {actuator} GPIO6 servo")
    send_direct(sock, port, seq, actuator, "detach_press", None)
    return True


def send_camera_profile(sock: socket.socket, port: int, seq: int, profile: int,
                        dest_ip: str, dest_port: int, transport: str) -> None:
    message = {
        "seq": seq,
        "target": "head",
        "command": "camera_profile",
        "profile": profile,
        "dest_ip": dest_ip,
        "dest_port": dest_port,
        "transport": transport,
        "source": "quality-supervisor",
        "time": time.time(),
    }
    sock.sendto(json.dumps(message).encode("utf-8"), (TARGETS["head"], port))
    print(f"[quality] sent camera profile={profile} to head")


def run_line_mode(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = args.target
    seq = 1

    print("End-to-end mesh control client")
    print("commands: w=forward s=backward a=left d=right x=stop quit=quit")
    print("extra: fl/fr/bl/br, hu/hd/hc/hmin/hmax, front/front_stop, detach/detach_rest")
    print("manual detach: 1=head servo releases node1, 2=node1 releases node2, 3=node2 releases node3")
    print("change target: t head | t node1 | t node2 | t node3 | t all")

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
        if command.startswith("detach_node"):
            released_node = command.replace("detach_", "", 1)
            if send_detach_release(sock, args.port, seq, released_node):
                seq += 1
            continue

        for _ in range(args.repeat):
            send(sock, args.port, seq, target, command, args.speed)
            time.sleep(args.repeat_delay)
        seq += 1

    return 0


def build_quality_supervisor(args: argparse.Namespace) -> Optional[QualitySupervisor]:
    if not args.quality_log:
        return None
    config = QualityConfig(
        video_log=args.quality_log,
        target_fps=args.quality_target_fps,
        interval=args.quality_interval,
        ping_ip=args.quality_ping_ip,
        base_ssh=args.quality_base_ssh,
        warn_speed_cap=args.quality_warn_speed,
        danger_speed_cap=0.0,
    )
    return QualitySupervisor(config)


def parse_detach_order(raw: str) -> List[str]:
    order = []
    for item in raw.split(","):
        name = item.strip()
        if name and name in TARGETS and name != "head":
            order.append(name)
    return order


def effective_speed(requested: Optional[float], speed_cap: Optional[float]) -> Optional[float]:
    if speed_cap is None:
        return requested
    base = 1.0 if requested is None else requested
    return min(base, speed_cap)


def run_camera_profile_command(command_template: Optional[str], profile: int) -> None:
    if not command_template:
        return
    command = command_template.format(profile=profile)
    print(f"[quality] camera profile command: {command}")
    subprocess.Popen(command, shell=True)


def run_live_mode(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = args.target
    seq = 1
    active_command = "stop"
    last_send = 0.0
    active_targets = list(TARGETS.keys())
    detach_order = parse_detach_order(args.detach_order)
    detached = set()
    last_detach_at = 0.0
    last_quality_status = None
    last_camera_profile = 0
    supervisor = build_quality_supervisor(args)
    old_settings = termios.tcgetattr(sys.stdin)

    print("Live mesh control")
    print("drive: w/s, steer head only: a/d/q/e/z/c, stop: x or space, quit: Ctrl-C")
    print("one-shot: u=head up, j=head down, k=head center, f=front motor, v=front stop")
    print("manual detach: 1=head releases node1, 2=node1 releases node2, 3=node2 releases node3")
    print(f"target={target} speed={args.speed if args.speed is not None else 'role default'}")
    if supervisor:
        print(
            f"quality supervision: log={args.quality_log} "
            f"warn_speed={args.quality_warn_speed} auto_detach={args.auto_detach}"
        )

    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            decision = supervisor.update() if supervisor else None
            send_speed = effective_speed(args.speed, decision.speed_cap if decision else None)

            if decision and decision.status != last_quality_status:
                print()
                print(
                    f"[quality] status={decision.status} raw={decision.raw_status} "
                    f"speed_cap={decision.speed_cap} profile={decision.camera_profile} "
                    f"reasons={'; '.join(decision.reasons)}"
                )
                last_quality_status = decision.status

            if decision and decision.camera_profile != last_camera_profile:
                if args.auto_camera_profile:
                    send_camera_profile(
                        sock,
                        args.port,
                        seq,
                        decision.camera_profile,
                        args.camera_dest_ip,
                        args.camera_dest_port,
                        args.camera_transport,
                    )
                    seq += 1
                run_camera_profile_command(args.camera_profile_cmd, decision.camera_profile)
                last_camera_profile = decision.camera_profile

            if decision and decision.status == "DANGER":
                if active_command != "stop":
                    send(sock, args.port, seq, target, "stop", send_speed, active_targets)
                    seq += 1
                    active_command = "stop"
                if args.auto_detach and time.monotonic() - last_detach_at >= args.detach_cooldown:
                    next_detach = next((name for name in detach_order if name in active_targets and name not in detached), None)
                    if next_detach:
                        print(f"[quality] DANGER persists. Detaching {next_detach}.")
                        if send_detach_release(sock, args.port, seq, next_detach):
                            seq += 1
                            detached.add(next_detach)
                            active_targets = [name for name in active_targets if name != next_detach]
                            last_detach_at = time.monotonic()
                            print(f"[quality] active moving targets now: {active_targets}")

            ready, _, _ = select.select([sys.stdin], [], [], args.send_interval)
            if ready:
                key = sys.stdin.read(1)
                if key == "\x03":
                    raise KeyboardInterrupt
                if key in LIVE_KEYS:
                    active_command = LIVE_KEYS[key]
                    if decision and decision.status == "DANGER" and active_command != "stop":
                        print("[quality] DANGER: drive command suppressed until quality recovers or detach completes")
                        active_command = "stop"
                    send(sock, args.port, seq, target, active_command, send_speed, active_targets)
                    seq += 1
                    last_send = time.monotonic()
                elif key in ONE_SHOT_LIVE_KEYS:
                    send(sock, args.port, seq, target, ONE_SHOT_LIVE_KEYS[key], send_speed, active_targets)
                    seq += 1
                elif key in MANUAL_DETACH_KEYS:
                    released_node = MANUAL_DETACH_KEYS[key]
                    if send_detach_release(sock, args.port, seq, released_node):
                        seq += 1
                        detached.add(released_node)
                        active_targets = [name for name in active_targets if name != released_node]
                        print(f"[detach] active moving targets now: {active_targets}")

            now = time.monotonic()
            if active_command != "stop" and now - last_send >= args.send_interval:
                send(sock, args.port, seq, target, active_command, send_speed, active_targets)
                seq += 1
                last_send = now

    except KeyboardInterrupt:
        print()
        send(sock, args.port, seq, target, "stop", args.speed, active_targets)
        return 0
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def run(args: argparse.Namespace) -> int:
    if args.live:
        return run_live_mode(args)
    return run_line_mode(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send UDP commands directly to mesh node IPs.")
    parser.add_argument("--target", default="all", choices=sorted(TARGETS) + ["all"], help="initial target")
    parser.add_argument("--port", type=int, default=7000, help="node UDP control port")
    parser.add_argument("--speed", type=float, default=None, help="0.0-1.0 speed scale sent with drive commands")
    parser.add_argument("--repeat", type=int, default=2, help="line mode UDP repeat count")
    parser.add_argument("--repeat-delay", type=float, default=0.01, help="seconds between line mode repeats")
    parser.add_argument("--live", action="store_true", help="send current key command repeatedly for driving")
    parser.add_argument("--send-interval", type=float, default=0.1, help="live mode send interval")
    parser.add_argument("--quality-log", default=None, help="enable video-first supervision using this video_probe JSONL log")
    parser.add_argument("--quality-target-fps", type=float, default=15.0, help="expected camera fps for quality supervision")
    parser.add_argument("--quality-interval", type=float, default=0.5, help="quality evaluation interval")
    parser.add_argument("--quality-ping-ip", default="192.168.50.10", help="IP to ping for RTT/loss while supervising")
    parser.add_argument("--quality-base-ssh", default=None, help="optional base SSH target for BATMAN TQ, e.g. hansel@192.168.60.1")
    parser.add_argument("--quality-warn-speed", type=float, default=0.35, help="speed cap while video/network is in WARN")
    parser.add_argument("--auto-detach", action="store_true", help="on sustained DANGER, stop and detach nodes in --detach-order")
    parser.add_argument("--detach-order", default="node2,node1", help="comma-separated released-node order for automatic relay placement, e.g. node3,node2,node1")
    parser.add_argument("--detach-cooldown", type=float, default=6.0, help="minimum seconds between automatic detach actions")
    parser.add_argument(
        "--auto-camera-profile",
        action="store_true",
        help="ask head to restart camera stream with quality-selected profile",
    )
    parser.add_argument("--camera-dest-ip", default="192.168.60.2", help="laptop IP that receives the head camera stream")
    parser.add_argument("--camera-dest-port", type=int, default=5600, help="laptop UDP port that receives the head camera stream")
    parser.add_argument("--camera-transport", choices=("rtp", "raw"), default="rtp", help="head camera transport")
    parser.add_argument(
        "--camera-profile-cmd",
        default=None,
        help="optional local shell command run when quality selects a new camera profile; use {profile}",
    )
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
