#!/bin/bash

set -e

REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
CAMERA_CONFIG="${HANSEL_CAMERA_CONFIG:-/etc/hansel-mesh/camera.env}"

if [ ! -f "$CAMERA_CONFIG" ]; then
    echo "[ERROR] Camera autostart is opt-in. Missing config: $CAMERA_CONFIG"
    echo "[ERROR] Copy configs/camera.env.example there and review it first."
    exit 1
fi

# This file is administrator-controlled and installed outside the repo.
# shellcheck disable=SC1090
source "$CAMERA_CONFIG"

if [ "${CAMERA_ENABLED:-no}" != "yes" ]; then
    echo "[ERROR] Set CAMERA_ENABLED=yes in $CAMERA_CONFIG to opt in."
    exit 1
fi

if [ -z "${CAMERA_DEST_IP:-}" ]; then
    echo "[ERROR] CAMERA_DEST_IP is required in $CAMERA_CONFIG."
    exit 1
fi

CAMERA_DEST_PORT="${CAMERA_DEST_PORT:-5600}"
CAMERA_TRANSPORT="${CAMERA_TRANSPORT:-rtp}"
PROFILE="${PROFILE:-medium}"
CAMERA_PROFILE_FILE="${HANSEL_CAMERA_PROFILE_FILE:-/run/hansel-camera-profile}"

# The production control server writes one validated profile token here and
# asks systemd to restart this service.  This keeps camera ownership inside
# hansel-camera.service and never evaluates remote data as shell code.
if [ -f "$CAMERA_PROFILE_FILE" ]; then
    IFS= read -r MANAGED_PROFILE < "$CAMERA_PROFILE_FILE" || true
    case "$MANAGED_PROFILE" in
        custom|0|1|2|3|high|medium|low|survival)
            PROFILE="$MANAGED_PROFILE"
            ;;
        *)
            echo "[ERROR] Invalid managed camera profile in $CAMERA_PROFILE_FILE."
            exit 1
            ;;
    esac
fi

case "$CAMERA_DEST_PORT" in
    ''|*[!0-9]*)
        echo "[ERROR] CAMERA_DEST_PORT must be numeric."
        exit 1
        ;;
esac
if [ "$CAMERA_DEST_PORT" -lt 1 ] || [ "$CAMERA_DEST_PORT" -gt 65535 ]; then
    echo "[ERROR] CAMERA_DEST_PORT must be between 1 and 65535."
    exit 1
fi

case "$CAMERA_TRANSPORT" in
    rtp|raw)
        ;;
    *)
        echo "[ERROR] CAMERA_TRANSPORT must be rtp or raw."
        exit 1
        ;;
esac

export CAMERA_TRANSPORT PROFILE
exec "$REPO_DIR/scripts/start_camera_stream.sh" "$CAMERA_DEST_IP" "$CAMERA_DEST_PORT"
