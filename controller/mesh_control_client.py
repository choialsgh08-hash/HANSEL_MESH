#!/usr/bin/env python3
"""Operator UDP client that sends directly to mesh node IPs."""

import argparse
import json
import socket
import time


COMMANDS = {
    "w": "forward",
    "s": "backward",
    "a": "left",
    "d": "right",
    "x": "stop",
}

TARGETS = {
    "head": "192.168.50.10",
    "node1": "192.168.50.11",
    "node2": "192.168.50.12",
}


def send(sock: socket.socket, ip: str, port: int, seq: int, target: str, command: str) -> None:
    message = {
        "seq": seq,
        "target": target,
        "command": command,
        "source": "operator",
        "time": time.time(),
    }
    sock.sendto(json.dumps(message).encode("utf-8"), (ip, port))
    print(f"sent seq={seq} target={target} ip={ip} command={command}")


def run(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = args.target
    seq = 1

    print("End-to-end mesh control client")
    print("commands: w=forward s=backward a=left d=right x=stop q=quit")
    print("change target: t head | t node1 | t node2")

    while True:
        raw = input(f"[{target}]> ").strip()
        if not raw:
            continue
        if raw == "q":
            break
        if raw.startswith("t "):
            next_target = raw.split(maxsplit=1)[1]
            if next_target not in TARGETS:
                print(f"unknown target: {next_target}")
                continue
            target = next_target
            continue

        command = COMMANDS.get(raw, raw)
        send(sock, TARGETS[target], args.port, seq, target, command)
        seq += 1

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send UDP commands directly to mesh node IPs.")
    parser.add_argument("--target", default="head", choices=sorted(TARGETS), help="initial target")
    parser.add_argument("--port", type=int, default=7000, help="node UDP control port")
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
