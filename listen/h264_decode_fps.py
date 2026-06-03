#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
디코딩 FPS 로거 — "직접 디코드하지 않는다".

이 모듈은 프레임을 파이썬에서 디코딩하지 않는다.
대신 **실제 디코더(ffmpeg / gstreamer / ffplay 등)를 서브프로세스로 띄워서**,
그 디코더가 표준에러/표준출력에 찍어주는 fps 수치만 파싱해서
KR 시각과 함께 CSV 로 로깅하고 대시보드 공유상태(state.fps)를 갱신한다.

지원 백엔드
-----------
- ffmpeg (기본): UDP H.264 를 디코드. stderr 의 `fps=`, `frame=` 를 파싱.
    * 기본은 measure-only (`-f null -`, 화면표시 없음)
    * --show-video 시 ffmpeg sdl2 창으로 표시(빌드가 sdl2 미지원이면 measure-only 로 자동 폴백)
- custom: 사용자가 이미 쓰는 디코더 명령을 그대로 감싼다.
    예) gstreamer fpsdisplaysink, ffplay -stats 등.
    출력에서 fps 를 자동 인식(ffmpeg `fps=`, gst `current:`/`average:`).

UDP 포트는 디코더 하나만 점유할 수 있다(unicast). 즉 이 도구가 :5600 을 받는 동안에는
별도 receive_camera_stream.sh 를 같은 포트로 동시에 돌릴 수 없다. 영상을 같이 보려면
--show-video 를 쓰거나, custom 백엔드로 '표시+측정'이 되는 파이프라인을 넘겨라.

단독 실행:
    python3 controller/h264_decode_fps.py --port 5600 --report-dir report
    python3 controller/h264_decode_fps.py --decoder custom \
        --decoder-cmd "gst-launch-1.0 -v udpsrc port=5600 caps=video/x-h264,stream-format=byte-stream \
                       ! h264parse ! avdec_h264 ! videoconvert ! fpsdisplaysink sync=false"
"""

import argparse
import csv
import os
import re
import select
import shlex
import shutil
import subprocess
import sys
import time

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from common.monitor_common import kr_now_str, kr_now, DEFAULT_CAMERA_PORT  # noqa: E402


# --- 디코더 출력에서 fps 수치 인식 ------------------------------------------
_RE_FFMPEG_FPS = re.compile(r"\bfps=\s*([0-9.]+)")
_RE_FFMPEG_FRAME = re.compile(r"\bframe=\s*([0-9]+)")
_RE_GST_CURRENT = re.compile(r"current:\s*([0-9.]+)")
_RE_GST_AVERAGE = re.compile(r"average:\s*([0-9.]+)")
_RE_GST_RENDERED = re.compile(r"rendered:\s*([0-9]+)")


def parse_fps_line(line):
    """디코더 한 줄에서 {instant, avg, total} 중 인식되는 값만 뽑아 반환."""
    out = {}
    m = _RE_GST_CURRENT.search(line)
    if m:
        out["instant"] = float(m.group(1))
    m = _RE_GST_AVERAGE.search(line)
    if m:
        out["avg"] = float(m.group(1))
    m = _RE_GST_RENDERED.search(line)
    if m:
        out["total"] = int(m.group(1))
    if "instant" not in out:
        m = _RE_FFMPEG_FPS.search(line)
        if m:
            out["instant"] = float(m.group(1))
    if "total" not in out:
        m = _RE_FFMPEG_FRAME.search(line)
        if m:
            out["total"] = int(m.group(1))
    return out


# --- 디코더 명령 구성 --------------------------------------------------------
def build_ffmpeg_cmd(port, show_video):
    url = f"udp://@:{port}?overrun_nonfatal=1&fifo_size=5000000&timeout=5000000"
    cmd = ["ffmpeg", "-hide_banner",
           "-fflags", "nobuffer", "-flags", "low_delay",
           "-probesize", "32", "-analyzeduration", "0",
           "-i", url, "-an"]
    if show_video:
        cmd += ["-f", "sdl2", f"HANSEL camera :{port}"]
    else:
        cmd += ["-f", "null", "-"]
    return cmd


# --- 메인 워커 ---------------------------------------------------------------
def decode_fps_worker(state, csv_path, stop_event, *,
                      port=DEFAULT_CAMERA_PORT, backend="ffmpeg",
                      show_video=False, custom_cmd=None, log_interval=1.0):
    """
    state      : SharedState (state.fps / state.lock). 단독 실행 시 None 허용.
    csv_path   : FPS 로그 CSV
    stop_event : threading.Event
    backend    : "ffmpeg" | "custom"
    """
    def set_fps(**kw):
        if state is None:
            return
        with state.lock:
            state.fps.update(kw)

    # 명령 결정
    if backend == "custom":
        if not custom_cmd:
            set_fps(running=False, error_msg="custom 백엔드인데 --decoder-cmd 없음")
            return
        cmd = shlex.split(custom_cmd) if isinstance(custom_cmd, str) else list(custom_cmd)
    else:
        if shutil.which("ffmpeg") is None:
            set_fps(running=False, error_msg="ffmpeg 미설치: apt install ffmpeg")
            print("[decode] ffmpeg 미설치")
            return
        cmd = build_ffmpeg_cmd(port, show_video)

    # CSV 준비
    new_file = not os.path.exists(csv_path)
    csvf = open(csv_path, "a", newline="", encoding="utf-8")
    writer = csv.writer(csvf)
    if new_file:
        writer.writerow(["timestamp_kr", "fps_instant", "fps_avg",
                         "total_frames", "backend"])
        csvf.flush()

    print(f"[decode] 디코더 실행: {' '.join(cmd)}")
    set_fps(running=False, error_msg=None)

    fell_back = False
    while not stop_event.is_set():
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, bufsize=0)
        except FileNotFoundError as e:
            set_fps(running=False, error_msg=f"디코더 실행 실패: {e}")
            time.sleep(1.0)
            continue

        fd = proc.stderr.fileno()
        buf = b""
        latest_instant = None
        latest_total = None
        first_seen = None
        last_log = 0.0
        saw_fps = False

        while not stop_event.is_set():
            ready, _, _ = select.select([fd], [], [], 0.5)
            if not ready:
                if proc.poll() is not None:
                    break  # 프로세스 종료 -> 재시도
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            buf += chunk
            # ffmpeg 통계는 '\r' 로 갱신되므로 \r, \n 둘 다로 분할
            parts = re.split(rb"[\r\n]", buf)
            buf = parts.pop()
            for raw in parts:
                line = raw.decode("utf-8", "ignore")
                vals = parse_fps_line(line)
                if "instant" in vals:
                    latest_instant = vals["instant"]
                    saw_fps = True
                if "total" in vals:
                    latest_total = vals["total"]

            now = time.monotonic()
            if latest_instant is not None and now - last_log >= log_interval:
                if first_seen is None:
                    first_seen = now
                avg = (latest_total / max(now - first_seen, 1e-6)
                       if latest_total else latest_instant)
                ts = kr_now_str()
                writer.writerow([ts, round(latest_instant, 2), round(avg, 2),
                                 latest_total if latest_total is not None else "",
                                 backend])
                csvf.flush()
                set_fps(running=True, error_msg=None,
                        instant=round(latest_instant, 2), avg=round(avg, 2),
                        total=latest_total or 0, last_epoch=time.time())
                last_log = now

        # 정리 / show-video 실패시 measure-only 폴백
        ret = proc.poll()
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

        if (backend == "ffmpeg" and show_video and not saw_fps
                and not fell_back and not stop_event.is_set()):
            print("[decode] sdl2 표시 실패로 추정 -> measure-only 로 폴백")
            set_fps(error_msg="영상표시(sdl2) 불가 -> measure-only")
            show_video = False
            cmd = build_ffmpeg_cmd(port, False)
            fell_back = True
            continue

        if not saw_fps:
            set_fps(running=False, error_msg="영상 미수신 또는 fps 미인식")
        else:
            set_fps(running=False)
        if not stop_event.is_set():
            time.sleep(0.5)

    csvf.close()
    print("[decode] FPS 로거 종료")


# --- 단독 실행 ---------------------------------------------------------------
def _standalone():
    p = argparse.ArgumentParser(description="디코딩 FPS 로거(단독)")
    p.add_argument("--port", type=int, default=DEFAULT_CAMERA_PORT)
    p.add_argument("--report-dir", default="report")
    p.add_argument("--decoder", choices=["ffmpeg", "custom"], default="ffmpeg")
    p.add_argument("--show-video", action="store_true")
    p.add_argument("--decoder-cmd", default=None)
    args = p.parse_args()

    import threading
    report_dir = os.path.abspath(args.report_dir)
    os.makedirs(report_dir, exist_ok=True)
    session = kr_now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(report_dir, f"decode_fps_{session}.csv")

    stop_event = threading.Event()
    th = threading.Thread(
        target=decode_fps_worker, args=(None, csv_path, stop_event),
        kwargs=dict(port=args.port, backend=args.decoder,
                    show_video=args.show_video, custom_cmd=args.decoder_cmd),
        daemon=True)
    th.start()
    print(f"FPS 로그: {csv_path}  (Ctrl-C 종료)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()
        th.join(timeout=3)


if __name__ == "__main__":
    _standalone()
