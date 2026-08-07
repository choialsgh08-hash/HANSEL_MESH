#!/bin/bash
set -e
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
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
ip link set "$BAT_IF" down 2>/dev/null || true
ip link delete "$BAT_IF" type batadv 2>/dev/null || true
if ip link show "$MESH_IF" >/dev/null 2>&1; then
  ip link set "$MESH_IF" down || true
  iw dev "$MESH_IF" mesh leave 2>/dev/null || true
  iw dev "$MESH_IF" ibss leave 2>/dev/null || true
  ip addr flush dev "$MESH_IF" || true
  iw dev "$MESH_IF" set type managed 2>/dev/null || true
  ip link set "$MESH_IF" up || true
  command -v nmcli >/dev/null && nmcli dev set "$MESH_IF" managed yes 2>/dev/null || true
fi
echo "[OK] Mesh stopped"
