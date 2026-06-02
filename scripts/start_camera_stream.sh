#!/bin/bash

set -e

DEST_IP="${1:-192.168.60.2}"
DEST_PORT="${2:-5600}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-15}"
BITRATE="${BITRATE:-1200000}"
PREVIEW="${PREVIEW:-no}"

if command -v rpicam-vid >/dev/null 2>&1; then
    CAMERA_CMD="rpicam-vid"
elif command -v libcamera-vid >/dev/null 2>&1; then
    CAMERA_CMD="libcamera-vid"
else
    echo "[ERROR] rpicam-vid/libcamera-vid not found."
    echo "Install Raspberry Pi camera tools first."
    exit 1
fi

echo "========================================"
echo " HANSEL_MESH camera stream"
echo "========================================"
echo "Camera command : $CAMERA_CMD"
echo "Destination    : udp://$DEST_IP:$DEST_PORT"
echo "Resolution     : ${WIDTH}x${HEIGHT}@${FPS}"
echo "Bitrate        : $BITRATE"
echo "Preview        : $PREVIEW"
echo "========================================"

PREVIEW_ARGS=()
if [ "$PREVIEW" = "no" ]; then
    PREVIEW_ARGS=(--nopreview)
fi

exec "$CAMERA_CMD" \
    -t 0 \
    "${PREVIEW_ARGS[@]}" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --framerate "$FPS" \
    --codec h264 \
    --inline \
    --bitrate "$BITRATE" \
    -o "udp://$DEST_IP:$DEST_PORT"
