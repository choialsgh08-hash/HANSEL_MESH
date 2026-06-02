#!/bin/bash

set -e

PC_CIDR="${PC_CIDR:-192.168.60.0/24}"
BASE_BAT_IP="${BASE_BAT_IP:-192.168.50.1}"
BAT_IF="${BAT_IF:-bat0}"

echo "========================================"
echo " HANSEL_MESH route to laptop network"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root on head/node:"
    echo "sudo ./scripts/setup_mesh_route_to_laptop.sh"
    exit 1
fi

if ! ip link show "$BAT_IF" >/dev/null 2>&1; then
    echo "[ERROR] BATMAN interface not found: $BAT_IF"
    echo "Start mesh first."
    exit 1
fi

echo "[1/2] Routing $PC_CIDR through base $BASE_BAT_IP..."
ip route replace "$PC_CIDR" via "$BASE_BAT_IP" dev "$BAT_IF"

echo "[2/2] Done."
ip route get 192.168.60.2 || true
