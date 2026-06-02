#!/bin/bash

set -e

BAT_IF="bat0"
MESH_IF="wlan0"

echo "========================================"
echo " HANSEL_MESH stop script"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/stop_mesh.sh"
    exit 1
fi

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

echo "[3/4] Restarting networking services if available..."
if command -v nmcli >/dev/null 2>&1; then
    nmcli dev set "$MESH_IF" managed yes 2>/dev/null || true
fi
systemctl restart NetworkManager 2>/dev/null || true
systemctl restart dhcpcd 2>/dev/null || true

echo "[4/4] Done."
echo "Mesh stopped."
