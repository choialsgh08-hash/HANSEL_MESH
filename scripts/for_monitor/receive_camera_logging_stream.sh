#!/bin/bash
#
# HANSEL_MESH 카메라 수신 (+ 디코딩 FPS 로깅 통합)
#
# 영상 표시와 fps 로깅을 "같은 디코더 한 프로세스"로 수행한다.
# -> UDP 를 한 번만 받아 바로 디코드/표시하므로 fan-out 같은 추가 홉이 없고 지연이 안 늘어난다.
#
# 사용법:
#   ./receive_camera_stream.sh [PORT] [옵션]
#     PORT                기본 5600
#     --log               디코딩 fps 를 CSV 로 기록 (표시 + 로깅)
#     --csv PATH          기록할 CSV 경로 직접 지정 (--log 자동 적용)
#     --report-dir DIR    --log 시 CSV 가 들어갈 폴더 (기본 report)
#     --decoder gst|ffmpeg|auto   로깅용 디코더 (기본 auto: gst 우선)
#
# 예)
#   ./receive_camera_stream.sh 5600                 # 표시만 (기존과 동일, 저지연)
#   ./receive_camera_stream.sh 5600 --log           # 표시 + fps 로깅
#   ./receive_camera_stream.sh 5600 --csv report/decode_fps_20260603_153000.csv
#
# 로깅은 gstreamer fpsdisplaysink(표시+측정) 또는 ffmpeg sdl2(-progress)로 한다.
# ffplay 는 표시는 되지만 fps 를 못 뽑으므로 --log 모드에서는 쓰지 않는다.

set -e

PORT=5600
DO_LOG=0
CSV=""
REPORT_DIR="report"
DECODER="auto"

# 첫 인자가 숫자면 PORT 로 해석
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then PORT="$1"; shift; fi
while [[ $# -gt 0 ]]; do
    case "$1" in
        --log) DO_LOG=1; shift;;
        --csv) CSV="$2"; DO_LOG=1; shift 2;;
        --report-dir) REPORT_DIR="$2"; shift 2;;
        --decoder) DECODER="$2"; shift 2;;
        *) echo "무시된 인자: $1"; shift;;
    esac
done

echo "========================================"
echo " HANSEL_MESH camera receiver"
echo " UDP port : $PORT"
echo " logging  : $([[ $DO_LOG -eq 1 ]] && echo on || echo off)"
echo "========================================"

# ---------------------------------------------------------------------------
# 표시 전용 모드 (기존 동작) — 저지연, ffplay 우선
# ---------------------------------------------------------------------------
if [[ "$DO_LOG" -eq 0 ]]; then
    if command -v ffplay >/dev/null 2>&1; then
        exec ffplay -fflags nobuffer -flags low_delay -framedrop "udp://0.0.0.0:$PORT"
    fi
    if command -v gst-launch-1.0 >/dev/null 2>&1; then
        exec gst-launch-1.0 -v \
            udpsrc port="$PORT" caps="application/x-h264,stream-format=(string)byte-stream,alignment=(string)au" \
            ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
    fi
    if command -v vlc >/dev/null 2>&1; then
        exec vlc "udp/h264://@:$PORT"
    fi
    echo "[ERROR] Need one receiver: ffplay, gst-launch-1.0, or vlc."
    exit 1
fi

# ---------------------------------------------------------------------------
# 표시 + fps 로깅 모드
# ---------------------------------------------------------------------------
if [[ -z "$CSV" ]]; then
    mkdir -p "$REPORT_DIR"
    SESSION="$(TZ='Asia/Seoul' date '+%Y%m%d_%H%M%S')"
    CSV="$REPORT_DIR/decode_fps_${SESSION}.csv"
else
    mkdir -p "$(dirname "$CSV")"
fi
echo "timestamp_kr,fps_current,fps_average,rendered" > "$CSV"
echo " FPS log  : $CSV"
echo "========================================"

# gstreamer: 표시(fpsdisplaysink) + 측정. -v 의 last-message 에서 current/average 파싱
log_from_gst() {
    gst-launch-1.0 -e -v \
        udpsrc port="$PORT" caps="application/x-h264,stream-format=(string)byte-stream,alignment=(string)au" \
        ! h264parse ! avdec_h264 ! videoconvert \
        ! fpsdisplaysink sync=false fps-update-interval=1000 2>&1 \
    | while IFS= read -r line; do
        case "$line" in
            *current:*)
                cur=$(printf '%s' "$line" | sed -n 's/.*current:[[:space:]]*\([0-9.]*\).*/\1/p')
                avg=$(printf '%s' "$line" | sed -n 's/.*average:[[:space:]]*\([0-9.]*\).*/\1/p')
                rnd=$(printf '%s' "$line" | sed -n 's/.*rendered:[[:space:]]*\([0-9]*\).*/\1/p')
                if [[ -n "$cur" ]]; then
                    ts="$(TZ='Asia/Seoul' date '+%Y-%m-%d %H:%M:%S.%3N')"
                    echo "$ts,$cur,${avg:-},${rnd:-}" >> "$CSV"
                    printf '\r[fps] current=%-7s average=%-7s rendered=%-8s' "$cur" "${avg:-}" "${rnd:-}"
                fi
                ;;
        esac
    done
}

# ffmpeg: sdl2 창으로 표시 + -progress 로 fps 출력(개행 구분이라 파싱 안정)
log_from_ffmpeg() {
    ffmpeg -hide_banner -nostats -fflags nobuffer -flags low_delay \
        -i "udp://0.0.0.0:$PORT" -an \
        -f sdl2 "HANSEL camera :$PORT" \
        -progress pipe:1 2>/dev/null \
    | while IFS= read -r line; do
        case "$line" in
            fps=*)   fps="${line#fps=}";;
            frame=*) frm="${line#frame=}";;
            progress=*)
                if [[ -n "${fps:-}" ]]; then
                    ts="$(TZ='Asia/Seoul' date '+%Y-%m-%d %H:%M:%S.%3N')"
                    echo "$ts,$fps,$fps,${frm:-}" >> "$CSV"
                    printf '\r[fps] current=%-7s rendered=%-8s' "$fps" "${frm:-}"
                fi
                ;;
        esac
    done
}

# 디코더 선택
HAVE="none"
case "$DECODER" in
    gst)    command -v gst-launch-1.0 >/dev/null 2>&1 && HAVE="gst";;
    ffmpeg) command -v ffmpeg         >/dev/null 2>&1 && HAVE="ffmpeg";;
    auto)
        if   command -v gst-launch-1.0 >/dev/null 2>&1; then HAVE="gst"
        elif command -v ffmpeg         >/dev/null 2>&1; then HAVE="ffmpeg"
        fi;;
esac

case "$HAVE" in
    gst)    echo "[decoder] gstreamer (표시 + fps)";       log_from_gst;;
    ffmpeg) echo "[decoder] ffmpeg sdl2 (표시 + fps)";     log_from_ffmpeg;;
    none)   echo "[ERROR] 로깅 가능한 디코더(gstreamer 또는 ffmpeg)가 없습니다."; exit 1;;
esac
