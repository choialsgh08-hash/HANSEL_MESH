#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HANSEL_MESH 모니터링 공통 모듈.

노드(robot/monitor_node.py)와 노트북(controller/monitor_laptop.py) 양쪽이
같이 쓰는 상수, 시각 헬퍼, JSON 메시지 헬퍼, RSSI 파서를 모아둔다.

설계 메모
---------
- 컨트롤 신호(start/stop)는 노트북이 각 노드 IP 로 직접 시드(unicast)하고,
  각 노드는 메시 브로드캐스트(192.168.50.255)로 한 번 더 릴레이한다.
  (세션+액션 조합으로 중복 제거 -> 무한 루프 없음)
- 라우팅상 노트북(192.168.60.x)에서 50.255 브로드캐스트는 Base 라우터가
  넘겨주지 않으므로, 노트북은 항상 unicast 로 시드한다.
- 노드끼리(bat0, 같은 L2 메시)는 브로드캐스트가 전파되므로 릴레이가 동작한다.
"""

import json
import re
import socket
import subprocess
from datetime import datetime, timezone, timedelta

# --- KR(Asia/Seoul) 시각 -----------------------------------------------------
try:
    # Python 3.9+ 표준 라이브러리. (Pi OS bookworm / 최신 Ubuntu 모두 OK)
    from zoneinfo import ZoneInfo
    KST = ZoneInfo("Asia/Seoul")
except Exception:  # tzdata 미설치 환경 대비 고정 UTC+9 fallback
    KST = timezone(timedelta(hours=9))


def kr_now():
    """KR 기준 timezone-aware datetime."""
    return datetime.now(KST)


def kr_now_str():
    """KR 기준 'YYYY-MM-DD HH:MM:SS.mmm' 문자열(밀리초까지)."""
    return datetime.now(KST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


# --- 네트워크 상수 -----------------------------------------------------------
DEFAULT_CONTROL_PORT = 7100     # 노트북->노드 / 노드<->노드 (start/stop, UDP)
DEFAULT_TELEMETRY_PORT = 7101   # 노드->노트북 실시간 RSSI (UDP)
DEFAULT_FILE_PORT = 7102        # 노드->노트북 로그 파일 전송 (TCP)
DEFAULT_CAMERA_PORT = 5600      # Head->노트북 H.264 영상 (UDP) - PDF 기본값

MESH_BROADCAST = "192.168.50.255"   # bat0 메시 서브넷 브로드캐스트
DEFAULT_LAPTOP_IP = "192.168.60.2"  # 구조자 노트북 유선 LAN IP - PDF 기본값

# bat0 IP 맵 (PDF 토폴로지 기준)
NODE_IP = {
    "base": "192.168.50.1",
    "head": "192.168.50.10",
    "node1": "192.168.50.11",
    "node2": "192.168.50.12",
    "node3": "192.168.50.13",
}

# "전방 유닛" = 헤드(붕괴현장 진입 방향)로 한 칸 앞에 있는 유닛.
#   물리 체인:  Base - Node2 - Node1 - Head(선두)
#   각 노드는 자기보다 한 칸 앞(헤드 쪽) 유닛으로부터 수신하는 RSSI 를 기록한다.
#   head 는 최선두라 전방 유닛이 없으므로(None) 자동 선택(가장 강한 이웃)으로 떨어진다.
#   ※ 방향을 뒤(Base 쪽)로 바꾸고 싶으면 이 맵만 수정하면 된다.
FORWARD_UNIT = {
    "base": "node2",
    "node2": "node1",
    "node1": "head",
    "head": None,
    "node3": "node2",
}


# --- JSON 메시지 헬퍼 --------------------------------------------------------
def encode_msg(d):
    return json.dumps(d, ensure_ascii=False).encode("utf-8")


def decode_msg(b):
    return json.loads(b.decode("utf-8"))


# --- RSSI 취득 (노드 전용) ---------------------------------------------------
def read_station_signals(iface="wlan0"):
    """
    `iw dev <iface> station dump` 를 파싱해서
    { mac(소문자): {"signal": dBm, "signal_avg": dBm} } 를 반환.

    802.11s mesh point / IBSS 둘 다 station dump 에 peer 별 signal 이 찍힌다.
    권한 문제로 실패하면 빈 dict 를 반환한다(상위에서 경고 처리).
    """
    try:
        out = subprocess.run(
            ["iw", "dev", iface, "station", "dump"],
            capture_output=True, text=True, timeout=3,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return {}
    if out.returncode != 0:
        return {}

    stations = {}
    cur = None
    for line in out.stdout.splitlines():
        stripped = line.strip()
        m = re.match(r"Station ([0-9a-fA-F:]{17})", stripped)
        if m:
            cur = m.group(1).lower()
            stations[cur] = {"signal": None, "signal_avg": None}
            continue
        if cur is None:
            continue
        if stripped.startswith("signal:"):
            v = re.search(r"-?\d+", stripped.split("signal:", 1)[1])
            if v:
                stations[cur]["signal"] = int(v.group())
        elif stripped.startswith("signal avg:"):
            v = re.search(r"-?\d+", stripped.split("signal avg:", 1)[1])
            if v:
                stations[cur]["signal_avg"] = int(v.group())
    return stations


def get_forward_rssi(iface, forward_mac=None):
    """
    전방 유닛의 RSSI 를 반환한다.

    반환: (mac, signal_dbm, signal_avg_dbm)
      - forward_mac 이 지정되면 그 MAC 의 signal 을 반환(없으면 None 값).
      - 미지정이면 현재 보이는 station 중 가장 강한 신호를 자동 선택.
      - station 이 하나도 안 보이면 (None, None, None).
    """
    sigs = read_station_signals(iface)
    if not sigs:
        return None, None, None

    if forward_mac:
        fm = forward_mac.lower()
        if fm in sigs:
            return fm, sigs[fm]["signal"], sigs[fm]["signal_avg"]
        # 설정은 됐는데 지금 안 보임 -> 링크 끊김(중요 신호)
        return fm, None, None

    # 자동: 가장 강한(=값이 큰, 0 에 가까운) signal 선택
    best = None
    for mac, d in sigs.items():
        if d["signal"] is None:
            continue
        if best is None or d["signal"] > sigs[best]["signal"]:
            best = mac
    if best is None:
        return None, None, None
    return best, sigs[best]["signal"], sigs[best]["signal_avg"]


def get_forward_rtt(ip, count=1, timeout=1):
    """
    전방 유닛 IP 로 ping 해서 왕복 시간(RTT, ms)을 반환한다.
    - 성공: 평균 RTT(float, ms)
    - 실패(미응답/타임아웃/unreachable): None
    RSSI 와 같은 방식으로, 매 샘플마다 한 번 호출한다.
    """
    if not ip:
        return None
    try:
        out = subprocess.run(
            ["ping", "-c", str(count), "-W", str(timeout), ip],
            capture_output=True, text=True, timeout=count * timeout + 2,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0:
        return None
    times = re.findall(r"time=([\d.]+)", out.stdout)
    if not times:
        return None
    vals = [float(t) for t in times]
    return round(sum(vals) / len(vals), 2)
