#!/bin/bash

set -e
set -o pipefail

DEST_IP="${1:-192.168.60.2}"
DEST_PORT="${2:-5600}"
CAMERA_TRANSPORT="${CAMERA_TRANSPORT:-rtp}"
RTP_IMPL="${RTP_IMPL:-auto}"
RTP_MTU="${RTP_MTU:-1200}"
RTP_PAYLOAD_TYPE="${RTP_PAYLOAD_TYPE:-96}"
PROFILE="${PROFILE:-custom}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-15}"
BITRATE="${BITRATE:-1200000}"
PREVIEW="${PREVIEW:-no}"
INTRA="${INTRA:-}"

case "$PROFILE" in
    0|high)
        WIDTH=640
        HEIGHT=480
        FPS=15
        BITRATE=1200000
        ;;
    1|medium)
        WIDTH=480
        HEIGHT=360
        FPS=12
        BITRATE=800000
        ;;
    2|low)
        WIDTH=320
        HEIGHT=240
        FPS=10
        BITRATE=600000
        ;;
    3|survival)
        WIDTH=320
        HEIGHT=240
        FPS=8
        BITRATE=400000
        ;;
    custom)
        ;;
    *)
        echo "[ERROR] Unknown PROFILE: $PROFILE"
        echo "Allowed: custom, 0/high, 1/medium, 2/low, 3/survival"
        exit 1
        ;;
esac

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
echo "Transport      : $CAMERA_TRANSPORT"
echo "Destination    : $DEST_IP:$DEST_PORT"
echo "Resolution     : ${WIDTH}x${HEIGHT}@${FPS}"
echo "Bitrate        : $BITRATE"
echo "Profile        : $PROFILE"
echo "RTP MTU        : $RTP_MTU"
echo "Preview        : $PREVIEW"
echo "========================================"

PREVIEW_ARGS=()
if [ "$PREVIEW" = "no" ]; then
    PREVIEW_ARGS=(--nopreview)
fi

INTRA_ARGS=()
if [ -n "$INTRA" ]; then
    INTRA_ARGS=(--intra "$INTRA")
elif [ -n "$FPS" ]; then
    INTRA_ARGS=(--intra "$FPS")
fi

CAMERA_ARGS=(
    "$CAMERA_CMD"
    -t 0
    "${PREVIEW_ARGS[@]}"
    --width "$WIDTH"
    --height "$HEIGHT"
    --framerate "$FPS"
    --codec h264
    --inline
    --bitrate "$BITRATE"
    "${INTRA_ARGS[@]}"
)

cleanup_pipeline() {
    jobs -p | xargs -r kill 2>/dev/null || true
}

case "$CAMERA_TRANSPORT" in
    raw|udp)
        echo "[INFO] Sending raw H.264 over UDP."
        exec "${CAMERA_ARGS[@]}" -o "udp://$DEST_IP:$DEST_PORT"
        ;;
    rtp)
        ;;
    *)
        echo "[ERROR] Unknown CAMERA_TRANSPORT: $CAMERA_TRANSPORT"
        echo "Allowed: rtp, raw"
        exit 1
        ;;
esac

trap cleanup_pipeline EXIT INT TERM

if [ "$RTP_IMPL" = "auto" ] && command -v gst-launch-1.0 >/dev/null 2>&1; then
    RTP_IMPL="gstreamer"
elif [ "$RTP_IMPL" = "auto" ] && command -v ffmpeg >/dev/null 2>&1; then
    RTP_IMPL="ffmpeg"
fi

case "$RTP_IMPL" in
    gstreamer|gst)
        if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
            echo "[ERROR] gst-launch-1.0 not found. Install GStreamer or set RTP_IMPL=ffmpeg."
            exit 1
        fi
        echo "[INFO] Sending RTP/H.264 over UDP with GStreamer."
        "${CAMERA_ARGS[@]}" -o - | gst-launch-1.0 -q \
            fdsrc fd=0 do-timestamp=true \
            ! video/x-h264,stream-format=byte-stream,alignment=au \
            ! h264parse config-interval=1 \
            ! rtph264pay pt="$RTP_PAYLOAD_TYPE" config-interval=1 mtu="$RTP_MTU" \
            ! udpsink host="$DEST_IP" port="$DEST_PORT" sync=false async=false
        ;;
    ffmpeg)
        if ! command -v ffmpeg >/dev/null 2>&1; then
            echo "[ERROR] ffmpeg not found. Install ffmpeg or GStreamer for RTP mode."
            exit 1
        fi
        echo "[INFO] Sending RTP/H.264 over UDP with ffmpeg."
        "${CAMERA_ARGS[@]}" -o - | ffmpeg -hide_banner -loglevel warning \
            -fflags nobuffer \
            -f h264 -i pipe:0 \
            -an -c:v copy \
            -flush_packets 1 \
            -max_delay 0 \
            -f rtp -payload_type "$RTP_PAYLOAD_TYPE" \
            "rtp://$DEST_IP:$DEST_PORT?pkt_size=$RTP_MTU"
        ;;
    *)
        echo "[ERROR] No RTP packetizer found."
        echo "Install one of these on head:"
        echo "  sudo apt install -y ffmpeg"
        echo "  sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad"
        echo "Fallback for old behavior: CAMERA_TRANSPORT=raw bash scripts/start_camera_stream.sh $DEST_IP $DEST_PORT"
        exit 1
        ;;
esac
