#!/bin/bash
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root" >&2; exit 1; }
WIRED_IF="${WIRED_IF:-eth0}"
LAPTOP_WIRED_IP="${LAPTOP_WIRED_IP:-192.168.60.2}"
ip route del default via "$LAPTOP_WIRED_IP" dev "$WIRED_IF" 2>/dev/null || true
if command -v nft >/dev/null 2>&1; then
  nft list table ip hansel_temp_internet >/dev/null 2>&1 && nft delete table ip hansel_temp_internet || true
fi
echo "[OK] Temporary Base Internet sharing disabled; normal HANSEL routing remains."
