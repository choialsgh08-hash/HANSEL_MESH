#!/bin/bash

set -e

LAPTOP_IF="${1:-}"
LAPTOP_IP_CIDR="${LAPTOP_IP_CIDR:-192.168.60.2/24}"
BASE_ETH_IP="${BASE_ETH_IP:-192.168.60.1}"
MESH_CIDR="${MESH_CIDR:-192.168.50.0/24}"

echo "========================================"
echo " HANSEL_MESH laptop route setup"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root on the laptop:"
    echo "sudo ./scripts/setup_laptop_mesh_routes.sh <ethernet-if>"
    exit 1
fi

if [ -z "$LAPTOP_IF" ]; then
    echo "[ERROR] Ethernet interface is required."
    echo "Find it with: ip -brief addr"
    exit 1
fi

if ! ip link show "$LAPTOP_IF" >/dev/null 2>&1; then
    echo "[ERROR] Interface not found: $LAPTOP_IF"
    exit 1
fi

echo "[1/3] Configuring $LAPTOP_IF as $LAPTOP_IP_CIDR..."
ip addr flush dev "$LAPTOP_IF"
ip addr add "$LAPTOP_IP_CIDR" dev "$LAPTOP_IF"
ip link set "$LAPTOP_IF" up

echo "[2/3] Routing mesh network $MESH_CIDR through base $BASE_ETH_IP..."
ip route replace "$MESH_CIDR" via "$BASE_ETH_IP" dev "$LAPTOP_IF"

if ip -brief addr | grep -v "^$LAPTOP_IF " | grep -q "192\.168\.50\."; then
    echo "[WARN] Another laptop interface also has a 192.168.50.x address."
    echo "[WARN] Turn laptop Wi-Fi off during this test if ping/video goes to the wrong path."
fi

echo "[3/3] Done."
ip -brief addr show "$LAPTOP_IF"
ip route get 192.168.50.10 || true
