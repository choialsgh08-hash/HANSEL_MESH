#!/bin/bash

set -e

PORT="${1:-5600}"

echo "========================================"
echo " HANSEL_MESH camera receiver"
echo "========================================"
echo "Listening UDP port: $PORT"
echo "========================================"

if command -v ffplay >/dev/null 2>&1; then
    exec ffplay -fflags nobuffer -flags low_delay -framedrop "udp://0.0.0.0:$PORT"
fi

if command -v gst-launch-1.0 >/dev/null 2>&1; then
    exec gst-launch-1.0 -v \
        udpsrc port="$PORT" caps="application/x-h264,stream-format=(string)byte-stream,alignment=(string)au" \
        ! h264parse \
        ! avdec_h264 \
        ! autovideosink sync=false
fi

if command -v vlc >/dev/null 2>&1; then
    exec vlc "udp/h264://@:$PORT"
fi

echo "[ERROR] Need one receiver: ffplay, gst-launch-1.0, or vlc."
exit 1
