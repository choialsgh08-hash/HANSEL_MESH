#!/bin/bash
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root" >&2; exit 1; }
BASE_BAT_IP="${BASE_BAT_IP:-192.168.50.1}"
BAT_IF="${BAT_IF:-bat0}"
ip route del default via "$BASE_BAT_IP" dev "$BAT_IF" 2>/dev/null || true
echo "[OK] Temporary Internet default route removed; normal laptop-subnet route remains."
