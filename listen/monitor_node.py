#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HANSEL_MESH 모니터링 - 노드(라즈베리파이)측 프로그램.

역할
----
1. 노트북/다른 노드가 보낸 컨트롤 신호(start/stop)를 UDP 로 수신한다.
2. 처음 보는 (session, action) 이면 메시 브로드캐스트로 "릴레이"해서 다른 노드에 전파한다.
3. start -> 전방 유닛 RSSI 를 주기적으로
      (a) 로컬 CSV 에 로깅하고
      (b) 노트북으로 실시간 전송한다.
4. stop  -> 샘플링을 멈추고, 즉시 자기 CSV 로그를 노트북(report 폴더)으로 TCP 전송한다.

실행 예
-------
    sudo python3 robot/monitor_node.py --name node1
    sudo python3 robot/monitor_node.py --name head  --forward-mac aa:bb:cc:dd:ee:ff
    sudo python3 robot/monitor_node.py --name node2 --laptop-ip 192.168.60.2

전방 유닛 MAC 확인 방법 (각 Pi 에서):
    iw dev wlan0 station dump | grep -A1 Station   # peer MAC 와 signal 확인
    sudo batctl n                                   # 메시 이웃 확인
구분이 안 되거나 이웃이 1개뿐이면 --forward-mac 생략 시 자동(가장 강한 신호)으로 잡는다.
"""

import argparse
import csv
import os
import socket
import sys
import threading
import time

# 저장소 루트를 import 경로에 추가 (common 패키지 사용)
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from common.monitor_common import (  # noqa: E402
    kr_now_str, encode_msg, decode_msg, get_forward_rssi, get_forward_rtt,
    DEFAULT_CONTROL_PORT, DEFAULT_TELEMETRY_PORT, DEFAULT_FILE_PORT,
    DEFAULT_LAPTOP_IP, MESH_BROADCAST, NODE_IP, FORWARD_UNIT,
)


def load_env_file(path):
    """
    configs/<role>.env 형식(KEY="value")의 단순 env 파일을 dict 로 읽는다.
    주석(#), 빈 줄, 선택적 'export ' 접두어, 양끝 따옴표를 처리한다.
    파일이 없으면 빈 dict 반환.
    """
    data = {}
    if not path or not os.path.exists(path):
        return data
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("export "):
                line = line[len("export "):]
            if "=" not in line:
                continue
            key, val = line.split("=", 1)
            key = key.strip()
            # 줄 끝 인라인 주석 제거 후 양끝 따옴표 제거
            val = val.split(" #", 1)[0].strip().strip('"').strip("'")
            if key:
                data[key] = val
    return data


class MonitorNode:
    def __init__(self, args):
        self.name = args.name
        self.iface = args.mesh_if
        self.forward_unit = args.forward_unit
        self.forward_mac = args.forward_mac
        self.forward_ip = args.forward_ip
        self.laptop_ip = args.laptop_ip
        self.control_port = args.control_port
        self.telemetry_port = args.telemetry_port
        self.file_port = args.file_port
        self.interval = args.interval
        self.workdir = os.path.abspath(args.workdir)
        self.config_path = getattr(args, "config_path", None)
        self.config_loaded = getattr(args, "config_loaded", False)
        os.makedirs(self.workdir, exist_ok=True)

        self.lock = threading.Lock()
        self.session = None
        self.sampling = False
        self.sample_thread = None
        self.csv_path = None
        self.handled = set()       # (session, action) 중복 제거용

        # 텔레메트리 전송 전용 소켓 (샘플 스레드 1개에서만 사용)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ---- 메인 루프: 컨트롤 신호 수신 ----------------------------------------
    def run(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rx.bind(("0.0.0.0", self.control_port))
        src = "로드됨" if self.config_loaded else "없음->기본값"
        print(f"[{self.name}] 설정파일: {self.config_path} ({src})")
        print(f"[{self.name}] 컨트롤 대기 (UDP :{self.control_port}) | "
              f"forward={self.forward_unit} mac={self.forward_mac or '(auto)'} "
              f"ip={self.forward_ip or '(없음)'}")
        print(f"[{self.name}] 로그 작업폴더: {self.workdir}  노트북: {self.laptop_ip}")
        while True:
            try:
                data, _ = rx.recvfrom(4096)
            except OSError:
                continue
            try:
                msg = decode_msg(data)
            except Exception:
                continue
            if msg.get("type") != "monitor_ctrl":
                continue
            self.handle_ctrl(msg)

    def handle_ctrl(self, msg):
        action = msg.get("action")
        session = msg.get("session")
        if action not in ("start", "stop") or not session:
            return
        key = (session, action)
        with self.lock:
            if key in self.handled:
                return  # 이미 처리+릴레이함 -> 중복/루프 방지
            self.handled.add(key)

        # 1) 릴레이: 메시 브로드캐스트로 다른 노드에 한 번 더 전파
        self.relay(msg)
        # 2) 동작
        if action == "start":
            self.start_monitor(session)
        else:
            self.stop_monitor(session)

    def relay(self, msg):
        """bat0 메시 브로드캐스트로 컨트롤 신호 1회 재전파."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.sendto(encode_msg(msg), (MESH_BROADCAST, self.control_port))
            s.close()
        except OSError as e:
            print(f"[{self.name}] 릴레이 실패: {e}")

    # ---- 모니터링 시작 ------------------------------------------------------
    def start_monitor(self, session):
        with self.lock:
            if self.sampling:
                return
            self.session = session
            self.sampling = True
            self.csv_path = os.path.join(self.workdir, f"rssi_{self.name}_{session}.csv")

        with open(self.csv_path, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(
                ["timestamp_kr", "node", "forward_unit",
                 "forward_mac", "rssi_dbm", "rssi_avg_dbm", "rtt_ms"]
            )

        self.sample_thread = threading.Thread(
            target=self.sample_loop, args=(session,), daemon=True)
        self.sample_thread.start()
        print(f"[{self.name}] 모니터링 시작 session={session} -> {self.csv_path}")

    def sample_loop(self, session):
        while True:
            with self.lock:
                if not self.sampling or self.session != session:
                    break
            ts = kr_now_str()
            mac, sig, avg = get_forward_rssi(self.iface, self.forward_mac)
            rtt = get_forward_rtt(self.forward_ip)

            # (a) 로컬 CSV 로깅 (RSSI + RTT 한 줄)
            row = [ts, self.name, self.forward_unit or "",
                   mac or "", "" if sig is None else sig,
                   "" if avg is None else avg,
                   "" if rtt is None else rtt]
            try:
                with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(row)
            except OSError as e:
                print(f"[{self.name}] CSV 기록 실패: {e}")

            # (b) 노트북으로 실시간 전송
            tele = {
                "type": "rssi",
                "node": self.name,
                "session": session,
                "ts_kr": ts,
                "forward_unit": self.forward_unit,
                "forward_mac": mac,
                "rssi_dbm": sig,
                "rssi_avg_dbm": avg,
                "rtt_ms": rtt,
            }
            try:
                self.tx.sendto(encode_msg(tele), (self.laptop_ip, self.telemetry_port))
            except OSError:
                pass  # 일시적 라우팅 문제는 무시(로컬 로그는 계속 남음)

            time.sleep(self.interval)

    # ---- 모니터링 종료 + 로그 전송 -----------------------------------------
    def stop_monitor(self, session):
        with self.lock:
            was = self.sampling
            self.sampling = False
        if self.sample_thread:
            self.sample_thread.join(timeout=2)
        if was:
            print(f"[{self.name}] 모니터링 종료 session={session}")
        if self.csv_path and os.path.exists(self.csv_path):
            self.send_log(self.csv_path)

    def send_log(self, path):
        """CSV 로그를 노트북으로 TCP 전송. (헤더 1줄 + 본문 바이트)"""
        fname = os.path.basename(path)
        try:
            with open(path, "rb") as f:
                payload = f.read()
        except OSError as e:
            print(f"[{self.name}] 로그 읽기 실패: {e}")
            return
        header = encode_msg({
            "type": "log", "node": self.name,
            "filename": fname, "size": len(payload),
        }) + b"\n"
        try:
            with socket.create_connection((self.laptop_ip, self.file_port), timeout=10) as s:
                s.sendall(header)
                s.sendall(payload)
            print(f"[{self.name}] 로그 전송 완료: {fname} ({len(payload)} bytes)")
        except OSError as e:
            print(f"[{self.name}] 로그 전송 실패({self.laptop_ip}:{self.file_port}): {e}")


def parse_args():
    p = argparse.ArgumentParser(description="HANSEL_MESH 노드측 RSSI/RTT 모니터")
    p.add_argument("--name", required=True, choices=list(NODE_IP.keys()),
                   help="이 Pi 의 역할 이름")
    p.add_argument("--config", default=None,
                   help="설정 env 파일 경로(기본 configs/<name>.env)")
    p.add_argument("--mesh-if", default=None, help="무선 메시 인터페이스(기본 wlan0)")
    p.add_argument("--forward-unit", default=None, help="전방 유닛 이름")
    p.add_argument("--forward-mac", default=None, help="전방 유닛 wlan0 MAC(RSSI용)")
    p.add_argument("--forward-ip", default=None, help="전방 유닛 bat0 IP(RTT용)")
    p.add_argument("--laptop-ip", default=DEFAULT_LAPTOP_IP)
    p.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    p.add_argument("--telemetry-port", type=int, default=DEFAULT_TELEMETRY_PORT)
    p.add_argument("--file-port", type=int, default=DEFAULT_FILE_PORT)
    p.add_argument("--interval", type=float, default=1.0, help="샘플 주기(초)")
    p.add_argument("--workdir", default="report", help="로컬 로그 저장 폴더")
    a = p.parse_args()

    # 설정 파일 로드: 기본은 configs/<name>.env (없으면 빈 dict)
    cfg_path = a.config or os.path.join(_REPO_ROOT, "configs", f"{a.name}.env")
    cfg = load_env_file(cfg_path)
    a.config_path = cfg_path
    a.config_loaded = bool(cfg)

    # 우선순위: CLI 인자 > 설정파일 > 환경변수 > 기본값/맵
    a.mesh_if = a.mesh_if or cfg.get("MESH_IF") or os.environ.get("MESH_IF") or "wlan0"
    a.forward_unit = (a.forward_unit or cfg.get("FORWARD_UNIT")
                      or FORWARD_UNIT.get(a.name))
    a.forward_mac = (a.forward_mac or cfg.get("FORWARD_MAC")
                     or os.environ.get("FORWARD_MAC"))
    # RTT 대상 IP: 명시값 없으면 전방 유닛 이름으로 bat0 IP 자동 계산
    a.forward_ip = (a.forward_ip or cfg.get("FORWARD_IP")
                    or NODE_IP.get(a.forward_unit))
    return a


if __name__ == "__main__":
    node = MonitorNode(parse_args())
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n종료합니다.")
