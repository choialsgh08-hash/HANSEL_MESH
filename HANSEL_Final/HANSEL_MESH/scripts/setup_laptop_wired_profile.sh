#!/bin/bash

set -e

LAPTOP_IF="${1:-}"
CON_NAME="${CON_NAME:-HANSEL_BASE_LINK}"
LAPTOP_IP_CIDR="${LAPTOP_IP_CIDR:-192.168.60.2/24}"
BASE_ETH_IP="${BASE_ETH_IP:-192.168.60.1}"
MESH_CIDR="${MESH_CIDR:-192.168.50.0/24}"

echo "========================================"
echo " HANSEL_MESH laptop wired profile setup"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root on the laptop:"
    echo "sudo ./scripts/setup_laptop_wired_profile.sh <ethernet-if>"
    exit 1
fi

if ! command -v nmcli >/dev/null 2>&1; then
    echo "[ERROR] nmcli not found. Use scripts/setup_laptop_mesh_routes.sh manually instead."
    exit 1
fi

if [ -z "$LAPTOP_IF" ]; then
    echo "[ERROR] Ethernet interface is required."
    echo "Find it with: ip -brief link"
    exit 1
fi

if ! ip link show "$LAPTOP_IF" >/dev/null 2>&1; then
    echo "[ERROR] Interface not found: $LAPTOP_IF"
    exit 1
fi

echo "[1/4] Replacing NetworkManager connection: $CON_NAME"
nmcli connection delete "$CON_NAME" 2>/dev/null || true

echo "[2/4] Creating static wired profile on $LAPTOP_IF..."
nmcli connection add type ethernet ifname "$LAPTOP_IF" con-name "$CON_NAME"
nmcli connection modify "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "$LAPTOP_IP_CIDR" \
    ipv4.routes "$MESH_CIDR $BASE_ETH_IP" \
    ipv4.never-default yes \
    ipv6.method ignore \
    connection.autoconnect yes \
    connection.autoconnect-priority 50

echo "[3/4] Bringing connection up..."
nmcli connection up "$CON_NAME"

echo "[4/4] Done."
ip -brief addr show "$LAPTOP_IF"
ip route get 192.168.50.10 || true
echo "========================================"
echo "After this, plugging the laptop into base should auto-use:"
echo "  laptop: 192.168.60.2/24"
echo "  base   : 192.168.60.1"
echo "========================================"
