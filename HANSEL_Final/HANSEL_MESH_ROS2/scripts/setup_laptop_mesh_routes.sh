#!/bin/bash
set -e
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
IFACE="${1:-${WIRED_IF:-}}"
[ -n "$IFACE" ] || { echo "Usage: $0 <laptop-wired-interface>"; exit 1; }
BASE_WIRED_IP="${BASE_WIRED_IP:-192.168.60.1}"
MESH_NET="${MESH_NET:-192.168.50.0/24}"
ip route replace "$MESH_NET" via "$BASE_WIRED_IP" dev "$IFACE"
echo "[OK] Laptop route installed: $MESH_NET via $BASE_WIRED_IP dev $IFACE"
