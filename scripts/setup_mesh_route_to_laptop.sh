#!/bin/bash
set -e
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
LAPTOP_NET="${LAPTOP_NET:-192.168.60.0/24}"
BASE_BAT_IP="${BASE_BAT_IP:-192.168.50.1}"
BAT_IF="${BAT_IF:-bat0}"
ip route replace "$LAPTOP_NET" via "$BASE_BAT_IP" dev "$BAT_IF"
echo "[OK] Route to laptop network via base: $LAPTOP_NET -> $BASE_BAT_IP"
