#!/bin/bash

set -e

ETH_IF="${ETH_IF:-eth0}"
ETH_IP_CIDR="${ETH_IP_CIDR:-192.168.60.1/24}"
BAT_IF="${BAT_IF:-bat0}"

echo "========================================"
echo " HANSEL_MESH base gateway setup"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/setup_base_gateway.sh"
    exit 1
fi

if ! ip link show "$ETH_IF" >/dev/null 2>&1; then
    echo "[ERROR] Ethernet interface not found: $ETH_IF"
    echo "Set ETH_IF if your adapter has a different name."
    exit 1
fi

if ! ip link show "$BAT_IF" >/dev/null 2>&1; then
    echo "[ERROR] BATMAN interface not found: $BAT_IF"
    echo "Start mesh first."
    exit 1
fi

echo "[1/4] Configuring $ETH_IF as $ETH_IP_CIDR..."
ip addr flush dev "$ETH_IF"
ip addr add "$ETH_IP_CIDR" dev "$ETH_IF"
ip link set "$ETH_IF" up

echo "[2/4] Enabling IPv4 forwarding..."
sysctl -w net.ipv4.ip_forward=1

echo "[3/4] Checking interfaces..."
ip -brief addr show "$ETH_IF"
ip -brief addr show "$BAT_IF"

echo "[4/4] Done."
echo "Laptop default management IP: 192.168.60.2/24"
echo "Laptop route to mesh:"
echo "sudo ip route replace 192.168.50.0/24 via 192.168.60.1"
