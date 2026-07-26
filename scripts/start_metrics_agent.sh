#!/bin/bash

set -e

ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
METRICS_CONFIG="${HANSEL_METRICS_CONFIG:-/etc/hansel-mesh/metrics.env}"

if [ -z "$ROLE" ]; then
    echo "[ERROR] Role is required: base, head, node1, node2, or node3"
    exit 1
fi

ROLE_CONFIG="$REPO_DIR/configs/$ROLE.env"
if [ ! -f "$ROLE_CONFIG" ]; then
    echo "[ERROR] Role config not found: $ROLE_CONFIG"
    exit 1
fi

# shellcheck disable=SC1090
source "$ROLE_CONFIG"

if [ -f "$METRICS_CONFIG" ]; then
    # This file is administrator-controlled and installed outside the repo.
    # shellcheck disable=SC1090
    source "$METRICS_CONFIG"
else
    echo "[WARN] Metrics config not found: $METRICS_CONFIG"
    echo "[WARN] Metrics will be written only to the systemd journal."
fi

METRICS_INTERVAL="${METRICS_INTERVAL:-5}"
METRICS_DEST="${METRICS_DEST:-}"
METRICS_PING_TARGETS="${METRICS_PING_TARGETS:-base head node1 node2 node3}"

if ! [[ "$METRICS_INTERVAL" =~ ^[0-9]+([.][0-9]+)?$ ]] \
    || ! awk -v interval="$METRICS_INTERVAL" 'BEGIN { exit !(interval > 0) }'
then
    echo "[ERROR] METRICS_INTERVAL must be a positive number."
    exit 1
fi

ARGS=(
    --self "$ROLE"
    --mesh-if "${MESH_IF:-wlan1}"
    --bat-if "${BAT_IF:-bat0}"
    --loop
    --interval "$METRICS_INTERVAL"
)

if [ -n "$METRICS_DEST" ]; then
    METRICS_HOST="${METRICS_DEST%:*}"
    METRICS_PORT="${METRICS_DEST##*:}"
    if [ -z "$METRICS_HOST" ] || ! [[ "$METRICS_PORT" =~ ^[0-9]+$ ]] \
        || [ "$METRICS_PORT" -lt 1 ] || [ "$METRICS_PORT" -gt 65535 ]
    then
        echo "[ERROR] METRICS_DEST must use valid host:port form: $METRICS_DEST"
        exit 1
    fi
    ARGS+=(--send "$METRICS_DEST")
fi

if [ -n "$METRICS_PING_TARGETS" ]; then
    read -r -a PING_TARGETS <<< "$METRICS_PING_TARGETS"
    ARGS+=(--ping "${PING_TARGETS[@]}")
fi

exec /usr/bin/python3 "$REPO_DIR/monitor/metrics_agent.py" "${ARGS[@]}"
