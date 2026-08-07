#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=7100)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((args.host, args.port))
        print(f"listening on udp://{args.host}:{args.port}")
        while True:
            payload, address = sock.recvfrom(65535)
            try:
                data = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                print(f"{address}: invalid packet ({exc})")
                continue
            print(f"{address}: {json.dumps(data, ensure_ascii=False, sort_keys=True)}")


if __name__ == "__main__":
    main()
