#!/bin/bash

set -e

PROFILE="${1:-0}"
DEST_IP="${2:-192.168.60.2}"
DEST_PORT="${3:-5600}"
CAMERA_TRANSPORT="${CAMERA_TRANSPORT:-rtp}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
LOG_DIR="${LOG_DIR:-$REPO_DIR/logs}"
PID_FILE="${PID_FILE:-$LOG_DIR/camera_stream.pid}"
LOG_FILE="${LOG_FILE:-$LOG_DIR/camera_stream.log}"

echo "========================================"
echo " HANSEL_MESH camera profile restart"
echo "========================================"
echo "Profile     : $PROFILE"
echo "Transport   : $CAMERA_TRANSPORT"
echo "Destination : udp://$DEST_IP:$DEST_PORT"
echo "Repo        : $REPO_DIR"

if [ ! -f "$REPO_DIR/scripts/start_camera_stream.sh" ]; then
    echo "[ERROR] Missing file: $REPO_DIR/scripts/start_camera_stream.sh"
    exit 1
fi

mkdir -p "$LOG_DIR"

if [ -f "$PID_FILE" ]; then
    OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo "[1/3] Stopping previous camera stream pid=$OLD_PID..."
        kill "$OLD_PID" 2>/dev/null || true
        sleep 0.5
        kill -9 "$OLD_PID" 2>/dev/null || true
    fi
fi

echo "[2/3] Releasing any leftover camera stream process..."
pkill -f "$REPO_DIR/scripts/start_camera_stream.sh" 2>/dev/null || true
pkill -f "rpicam-vid.*udp://$DEST_IP:$DEST_PORT" 2>/dev/null || true
pkill -f "libcamera-vid.*udp://$DEST_IP:$DEST_PORT" 2>/dev/null || true
pkill -f "gst-launch-1.0.*rtph264pay.*$DEST_PORT" 2>/dev/null || true
pkill -f "ffmpeg.*rtp://$DEST_IP:$DEST_PORT" 2>/dev/null || true

echo "[3/3] Starting camera stream..."
CAMERA_TRANSPORT="$CAMERA_TRANSPORT" PROFILE="$PROFILE" nohup bash "$REPO_DIR/scripts/start_camera_stream.sh" "$DEST_IP" "$DEST_PORT" >"$LOG_FILE" 2>&1 &
NEW_PID="$!"
echo "$NEW_PID" > "$PID_FILE"

echo "Started pid  : $NEW_PID"
echo "Log file     : $LOG_FILE"
echo "========================================"
