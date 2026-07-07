#!/bin/bash

set -e

PORT="${1:-5600}"
CAMERA_TRANSPORT="${CAMERA_TRANSPORT:-rtp}"
RTP_PAYLOAD_TYPE="${RTP_PAYLOAD_TYPE:-96}"
RTP_LATENCY_MS="${RTP_LATENCY_MS:-80}"
SDP_FILE=""

cleanup() {
    if [ -n "$SDP_FILE" ] && [ -f "$SDP_FILE" ]; then
        rm -f "$SDP_FILE"
    fi
}

make_rtp_sdp() {
    SDP_FILE="$(mktemp /tmp/hansel-camera-rtp-XXXXXX.sdp)"
    {
        printf "v=0\n"
        printf "o=- 0 0 IN IP4 127.0.0.1\n"
        printf "s=HANSEL_MESH camera\n"
        printf "c=IN IP4 0.0.0.0\n"
        printf "t=0 0\n"
        printf "m=video %s RTP/AVP %s\n" "$PORT" "$RTP_PAYLOAD_TYPE"
        printf "a=rtpmap:%s H264/90000\n" "$RTP_PAYLOAD_TYPE"
        printf "a=fmtp:%s packetization-mode=1\n" "$RTP_PAYLOAD_TYPE"
    } > "$SDP_FILE"
}

trap cleanup EXIT INT TERM

echo "========================================"
echo " HANSEL_MESH camera receiver"
echo "========================================"
echo "Transport         : $CAMERA_TRANSPORT"
echo "Listening UDP port: $PORT"
echo "========================================"

case "$CAMERA_TRANSPORT" in
    raw|udp)
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
        ;;
    rtp)
        if command -v ffplay >/dev/null 2>&1; then
            make_rtp_sdp
            ffplay \
                -protocol_whitelist file,udp,rtp \
                -fflags nobuffer \
                -flags low_delay \
                -framedrop \
                "$SDP_FILE"
            exit "$?"
        fi

        if command -v gst-launch-1.0 >/dev/null 2>&1; then
            exec gst-launch-1.0 -v \
                udpsrc port="$PORT" caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)$RTP_PAYLOAD_TYPE" \
                ! rtpjitterbuffer latency="$RTP_LATENCY_MS" drop-on-latency=true \
                ! rtph264depay \
                ! h264parse \
                ! avdec_h264 \
                ! autovideosink sync=false
        fi

        if command -v vlc >/dev/null 2>&1; then
            make_rtp_sdp
            vlc "$SDP_FILE"
            exit "$?"
        fi
        ;;
    *)
        echo "[ERROR] Unknown CAMERA_TRANSPORT: $CAMERA_TRANSPORT"
        echo "Allowed: rtp, raw"
        exit 1
        ;;
esac

echo "[ERROR] Need one receiver: ffplay, gst-launch-1.0, or vlc."
exit 1
