#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HANSEL_MESH 모니터링 - 노트북(구조자측) 메인 프로그램.

하는 일
--------
1. 모니터링 START 신호를 각 노드 IP 로 시드(unicast, 반복) -> 노드끼리 메시 릴레이로 전파.
2. (동시에) 터미널 대시보드 기동:
     - 각 노드가 보내는 RSSI 텔레메트리를 실시간 텍스트로 표시
     - H.264(UDP) 디코딩 FPS 를 실시간 표시 + CSV 로깅
3. 종료(Ctrl-C 또는 --duration 만료) 시 STOP 신호 전파.
4. 각 노드가 보내오는 RSSI 로그 CSV 를 ./report 폴더에 수신/저장.

실행 예
-------
    python3 controller/monitor_laptop.py
    python3 controller/monitor_laptop.py --duration 60
    python3 controller/monitor_laptop.py --nodes head node1 node2 base --camera-port 5600

종료: Ctrl-C (STOP 신호 전파 + 노드 로그 수집 후 정리)
"""

import argparse
import os
import socket
import sys
import threading
import time

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from common.monitor_common import (  # noqa: E402
    kr_now, kr_now_str, encode_msg, decode_msg,
    DEFAULT_CONTROL_PORT, DEFAULT_TELEMETRY_PORT, DEFAULT_FILE_PORT,
    DEFAULT_CAMERA_PORT, NODE_IP,
)
from controller.h264_decode_fps import decode_fps_worker  # noqa: E402


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.rssi = {}      # node -> {ts_kr, rssi, avg, fwd, count, last_epoch}
        self.fps = {"instant": 0.0, "avg": 0.0, "total": 0,
                    "running": False, "error_msg": None, "last_epoch": None}
        self.logs = []      # 수신한 로그 파일명
        self.session = None
        self.started = None


# --- 컨트롤 신호 송신 --------------------------------------------------------
def send_control(action, session, node_ips, port, repeat=6, gap=0.25):
    """각 노드 IP 로 start/stop 을 반복 시드(unicast). 노드끼리는 메시 릴레이로 전파됨."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload = encode_msg({
        "type": "monitor_ctrl", "action": action,
        "session": session, "origin": "laptop", "ts": time.time(),
    })
    for _ in range(repeat):
        for ip in node_ips:
            try:
                s.sendto(payload, (ip, port))
            except OSError:
                pass
        time.sleep(gap)
    s.close()


# --- RSSI 텔레메트리 수신 ----------------------------------------------------
def telemetry_worker(state, port, stop_event):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))
    s.settimeout(0.5)
    while not stop_event.is_set():
        try:
            data, _ = s.recvfrom(8192)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            msg = decode_msg(data)
        except Exception:
            continue
        if msg.get("type") != "rssi":
            continue
        node = msg.get("node", "?")
        with state.lock:
            cur = state.rssi.get(node, {"count": 0})
            state.rssi[node] = {
                "ts_kr": msg.get("ts_kr"),
                "rssi": msg.get("rssi_dbm"),
                "avg": msg.get("rssi_avg_dbm"),
                "rtt": msg.get("rtt_ms"),
                "fwd": msg.get("forward_unit"),
                "count": cur.get("count", 0) + 1,
                "last_epoch": time.time(),
            }
    s.close()


# --- 노드 로그 파일 수신(TCP) ------------------------------------------------
def file_server_worker(state, port, report_dir, stop_event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(8)
    srv.settimeout(0.5)
    while not stop_event.is_set():
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        threading.Thread(target=_handle_log_conn,
                         args=(conn, state, report_dir), daemon=True).start()
    srv.close()


def _handle_log_conn(conn, state, report_dir):
    with conn:
        buf = b""
        # 헤더(개행 종료) 읽기
        while b"\n" not in buf:
            chunk = conn.recv(4096)
            if not chunk:
                return
            buf += chunk
        line, rest = buf.split(b"\n", 1)
        try:
            hdr = decode_msg(line)
        except Exception:
            return
        size = int(hdr.get("size", 0))
        fname = os.path.basename(hdr.get("filename", "unknown.csv"))
        data = rest
        while len(data) < size:
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        out = os.path.join(report_dir, fname)
        try:
            with open(out, "wb") as f:
                f.write(data[:size])
            with state.lock:
                if fname not in state.logs:
                    state.logs.append(fname)
            print(f"[laptop] 노드 로그 수신: {fname} ({len(data)} bytes) -> {out}")
        except OSError as e:
            print(f"[laptop] 로그 저장 실패: {e}")


# --- 터미널 대시보드 ---------------------------------------------------------
def rssi_bar(dbm):
    """RSSI(dBm)를 간단한 막대/등급으로 표시."""
    if dbm is None:
        return "[ 신호없음 ]"
    if dbm >= -55:
        level, mark = "강", "█████"
    elif dbm >= -67:
        level, mark = "양호", "████ "
    elif dbm >= -75:
        level, mark = "보통", "███  "
    elif dbm >= -85:
        level, mark = "약함", "██   "
    else:
        level, mark = "위험", "█    "
    return f"{mark} {level}"


def render_dashboard(state, args, stop_event, end_epoch):
    is_tty = sys.stdout.isatty()
    while not stop_event.is_set():
        now = time.time()
        with state.lock:
            session = state.session
            started = state.started
            rssi = dict(state.rssi)
            fps = dict(state.fps)
            logs = list(state.logs)

        elapsed = int(now - started) if started else 0
        remain = ("" if end_epoch is None
                  else f" / 남은 {max(0, int(end_epoch - now))}s")

        lines = []
        lines.append("=" * 64)
        lines.append(f" HANSEL_MESH 모니터링  session={session}")
        lines.append(f" 경과 {elapsed}s{remain}    KR {kr_now_str()}")
        lines.append("=" * 64)
        lines.append(" [RSSI / RTT 모니터]  (전방 유닛 기준)")
        lines.append(f"  {'노드':<7}{'전방':<7}{'RSSI':>6}{'avg':>5}{'RTT':>9}  {'상태':<12}{'갱신':>6}{'샘플':>5}")
        if not rssi:
            lines.append("   (아직 수신된 텔레메트리 없음...)")
        else:
            for node in sorted(rssi.keys()):
                d = rssi[node]
                age = now - d["last_epoch"]
                dbm = d["rssi"]
                avg = d["avg"]
                rtt = d.get("rtt")
                lines.append(
                    f"  {node:<7}{(d['fwd'] or '-'):<7}"
                    f"{('--' if dbm is None else dbm):>6}"
                    f"{('--' if avg is None else avg):>5}"
                    f"{('--' if rtt is None else f'{rtt}ms'):>9}  "
                    f"{rssi_bar(dbm):<12}{age:>5.1f}s{d['count']:>5}"
                )
        lines.append("-" * 64)
        lines.append(" [디코딩 FPS 모니터]  (H.264 over UDP)")
        if fps.get("error_msg"):
            lines.append(f"   ! {fps['error_msg']}")
        fps_age = ("-" if not fps.get("last_epoch")
                   else f"{now - fps['last_epoch']:.1f}s 전")
        status = "수신중" if fps.get("running") else "대기/끊김"
        lines.append(f"   상태:{status}  순간:{fps['instant']:.1f} fps  "
                     f"평균:{fps['avg']:.1f} fps  누적:{fps['total']} 프레임  (갱신 {fps_age})")
        lines.append("-" * 64)
        lines.append(f" 수신된 노드 로그: {len(logs)}개 {logs if logs else ''}")
        lines.append(" 종료: Ctrl-C")
        lines.append("=" * 64)

        out = "\n".join(lines)
        if is_tty:
            sys.stdout.write("\033[2J\033[H")  # 화면 지우고 커서 홈
        sys.stdout.write(out + "\n")
        sys.stdout.flush()

        if end_epoch is not None and now >= end_epoch:
            break
        time.sleep(0.5)


def main():
    p = argparse.ArgumentParser(description="HANSEL_MESH 노트북측 모니터")
    p.add_argument("--nodes", nargs="+", default=["base", "node2", "node1", "head"],
                   help="START/STOP 을 시드할 노드 이름들")
    p.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    p.add_argument("--telemetry-port", type=int, default=DEFAULT_TELEMETRY_PORT)
    p.add_argument("--file-port", type=int, default=DEFAULT_FILE_PORT)
    p.add_argument("--camera-port", type=int, default=DEFAULT_CAMERA_PORT)
    p.add_argument("--decoder", choices=["ffmpeg", "custom"], default="ffmpeg",
                   help="fps 를 얻어올 디코더 백엔드(직접 디코딩 안 함)")
    p.add_argument("--show-video", action="store_true",
                   help="ffmpeg 백엔드에서 영상도 창으로 표시(sdl2)")
    p.add_argument("--decoder-cmd", default=None,
                   help="custom 백엔드: 직접 쓰는 디코더 명령(예: gstreamer fpsdisplaysink)")
    p.add_argument("--report-dir", default="report",
                   help="컨트롤 코드를 실행한 디렉터리 하위 report 폴더")
    p.add_argument("--duration", type=float, default=None,
                   help="자동 종료 시간(초). 미지정시 Ctrl-C 까지 계속")
    args = p.parse_args()

    node_ips = [NODE_IP[n] for n in args.nodes if n in NODE_IP]
    report_dir = os.path.abspath(args.report_dir)
    os.makedirs(report_dir, exist_ok=True)

    session = kr_now().strftime("%Y%m%d_%H%M%S")
    state = SharedState()
    state.session = session
    state.started = time.time()

    stop_event = threading.Event()
    threads = []

    # 1) 백그라운드 수신부: 텔레메트리 / 로그서버 / 디코더
    threads.append(threading.Thread(
        target=telemetry_worker, args=(state, args.telemetry_port, stop_event), daemon=True))
    threads.append(threading.Thread(
        target=file_server_worker, args=(state, args.file_port, report_dir, stop_event), daemon=True))
    fps_csv = os.path.join(report_dir, f"decode_fps_{session}.csv")
    threads.append(threading.Thread(
        target=decode_fps_worker, args=(state, fps_csv, stop_event),
        kwargs=dict(port=args.camera_port, backend=args.decoder,
                    show_video=args.show_video, custom_cmd=args.decoder_cmd),
        daemon=True))
    for t in threads:
        t.start()

    print(f"[laptop] session={session}  report={report_dir}")
    print(f"[laptop] 노드 시드 IP: {node_ips}")

    # 2) START 신호 송신 + (동시에) 대시보드 기동
    send_control("start", session, node_ips, args.control_port)
    end_epoch = None if args.duration is None else time.time() + args.duration

    try:
        render_dashboard(state, args, stop_event, end_epoch)
    except KeyboardInterrupt:
        print("\n[laptop] 사용자 종료 요청...")

    # 3) STOP 신호 전파
    print("[laptop] STOP 신호 전파...")
    send_control("stop", session, node_ips, args.control_port)

    # 4) 노드 로그 수집 대기 (파일서버/디코더는 잠시 더 살려둠)
    print("[laptop] 노드 로그 수집 대기 (최대 15s)...")
    deadline = time.time() + 15
    expected = len(node_ips)
    while time.time() < deadline:
        with state.lock:
            got = len(state.logs)
        if got >= expected:
            break
        time.sleep(0.5)

    stop_event.set()
    for t in threads:
        t.join(timeout=3)

    with state.lock:
        logs = list(state.logs)
    print("=" * 60)
    print(f"[laptop] 완료. report 폴더: {report_dir}")
    print(f"  - 디코딩 FPS 로그: {os.path.basename(fps_csv)}")
    print(f"  - 수신 노드 RSSI 로그({len(logs)}): {logs}")
    print("=" * 60)


if __name__ == "__main__":
    main()
