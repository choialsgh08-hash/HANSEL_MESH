#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_ARG="${1:-}"

if [ -n "$CONFIG_ARG" ]; then
    CONFIG_FILE="$CONFIG_ARG"
    if [ ! -f "$CONFIG_FILE" ] && [ -f "$REPO_ROOT/configs/$CONFIG_ARG.env" ]; then
        CONFIG_FILE="$REPO_ROOT/configs/$CONFIG_ARG.env"
    fi
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "[ERROR] Config file or role not found: $CONFIG_ARG"
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
fi

BAT_IF="${BAT_IF:-bat0}"
MESH_IF="${MESH_IF:-wlan0}"
RESTART_GLOBAL_NETWORK_SERVICES="${RESTART_GLOBAL_NETWORK_SERVICES:-no}"

case "$RESTART_GLOBAL_NETWORK_SERVICES" in
    yes|no)
        ;;
    *)
        echo "[ERROR] RESTART_GLOBAL_NETWORK_SERVICES must be yes or no."
        exit 1
        ;;
esac

echo "========================================"
echo " HANSEL_MESH stop script"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/stop_mesh.sh [configs/role.env|role]"
    exit 1
fi

echo "[INFO] Mesh IF: $MESH_IF"
echo "[INFO] BAT IF : $BAT_IF"

echo "[1/4] Bringing down BATMAN interface..."
if ip link show "$BAT_IF" >/dev/null 2>&1; then
    ip link set "$BAT_IF" down || true
    ip link delete "$BAT_IF" type batadv || true
else
    echo "[INFO] $BAT_IF does not exist."
fi

echo "[2/4] Resetting Wi-Fi interface..."
if ip link show "$MESH_IF" >/dev/null 2>&1; then
    ip link set "$MESH_IF" down || true
    iw dev "$MESH_IF" mesh leave 2>/dev/null || true
    iw dev "$MESH_IF" ibss leave 2>/dev/null || true
    ip addr flush dev "$MESH_IF" || true
    iw dev "$MESH_IF" set type managed || true
    ip link set "$MESH_IF" up || true
else
    echo "[INFO] $MESH_IF does not exist."
fi

echo "[3/4] Returning only the mesh interface to network management..."
if command -v nmcli >/dev/null 2>&1; then
    nmcli dev set "$MESH_IF" managed yes 2>/dev/null || true
fi
if [ "$RESTART_GLOBAL_NETWORK_SERVICES" = "yes" ]; then
    echo "[WARN] Explicit opt-in enabled: restarting global NetworkManager/dhcpcd."
    systemctl restart NetworkManager 2>/dev/null || true
    systemctl restart dhcpcd 2>/dev/null || true
else
    echo "[INFO] Preserving global network services and any separate rescue AP."
fi

echo "[4/4] Done."
echo "Mesh stopped."
