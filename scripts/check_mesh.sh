#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_ARG="${1:-}"
if [ -n "$CONFIG_ARG" ]; then
  CONFIG_FILE="$CONFIG_ARG"
  [ -f "$CONFIG_FILE" ] || CONFIG_FILE="$REPO_ROOT/configs/$CONFIG_ARG.env"
  [ -f "$CONFIG_FILE" ] || { echo "[ERROR] Config/role not found: $CONFIG_ARG"; exit 1; }
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi
BAT_IF="${BAT_IF:-bat0}"; MESH_IF="${MESH_IF:-wlan1}"
echo "=== HANSEL_MESH status ==="
ip -brief addr
echo "--- $BAT_IF ---"; ip addr show "$BAT_IF" 2>/dev/null || true
echo "--- $MESH_IF ---"; iw dev "$MESH_IF" info 2>/dev/null || true
echo "--- batctl interfaces ---"; sudo batctl -m "$BAT_IF" if 2>/dev/null || true
echo "--- neighbors ---"; sudo batctl -m "$BAT_IF" n 2>/dev/null || true
echo "--- originators ---"; sudo batctl -m "$BAT_IF" o 2>/dev/null || true
echo "--- routes ---"; ip route
