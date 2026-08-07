#!/bin/bash
set -e
ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
METRICS_CONFIG="${HANSEL_METRICS_CONFIG:-/etc/hansel-mesh/metrics.env}"
[ -n "$ROLE" ] || { echo "[ERROR] Role required"; exit 1; }
ROLE_CONFIG="$REPO_DIR/configs/$ROLE.env"
[ -f "$ROLE_CONFIG" ] || { echo "[ERROR] Missing $ROLE_CONFIG"; exit 1; }
# shellcheck disable=SC1090
source "$ROLE_CONFIG"
[ ! -f "$METRICS_CONFIG" ] || source "$METRICS_CONFIG"
METRICS_INTERVAL="${METRICS_INTERVAL:-5}"
METRICS_DEST="${METRICS_DEST:-}"
METRICS_PING_TARGETS="${METRICS_PING_TARGETS:-base head node1 node2 node3}"
ARGS=(--self "$ROLE" --mesh-if "${MESH_IF:-wlan1}" --bat-if "${BAT_IF:-bat0}" --loop --interval "$METRICS_INTERVAL")
[ -z "$METRICS_DEST" ] || ARGS+=(--send "$METRICS_DEST")
read -r -a PINGS <<< "$METRICS_PING_TARGETS"
[ "${#PINGS[@]}" -eq 0 ] || ARGS+=(--ping "${PINGS[@]}")
exec /usr/bin/python3 "$REPO_DIR/monitor/metrics_agent.py" "${ARGS[@]}"
