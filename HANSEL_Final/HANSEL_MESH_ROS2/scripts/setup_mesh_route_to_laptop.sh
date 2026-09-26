#!/bin/bash
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root" >&2; exit 1; }
LAPTOP_NET="${LAPTOP_NET:-192.168.60.0/24}"
BASE_BAT_IP="${BASE_BAT_IP:-192.168.50.1}"
BAT_IF="${BAT_IF:-bat0}"
ip link show "$BAT_IF" >/dev/null 2>&1 || { echo "[ERROR] Missing mesh interface: $BAT_IF" >&2; exit 1; }

# Normal operation needs only the operator-laptop subnet via Base.
ip route replace "$LAPTOP_NET" via "$BASE_BAT_IP" dev "$BAT_IF"

# A default route via Base belongs only to temporary Internet-sharing mode.
ip route del default via "$BASE_BAT_IP" dev "$BAT_IF" 2>/dev/null || true

echo "[OK] Normal route installed: $LAPTOP_NET via $BASE_BAT_IP dev $BAT_IF; default route unchanged/removed"
