#!/bin/bash
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root" >&2; exit 1; }
BAT_IF="${BAT_IF:-bat0}"
WIRED_IF="${WIRED_IF:-eth0}"
BASE_WIRED_ADDR="${BASE_WIRED_ADDR:-192.168.60.1/24}"
LAPTOP_WIRED_IP="${LAPTOP_WIRED_IP:-192.168.60.2}"

ip link show "$BAT_IF" >/dev/null 2>&1 || { echo "[ERROR] Missing mesh interface: $BAT_IF" >&2; exit 1; }
ip link show "$WIRED_IF" >/dev/null 2>&1 || { echo "[ERROR] Missing wired interface: $WIRED_IF" >&2; exit 1; }

# Normal HANSEL operation is routed, not NATed:
#   mesh 192.168.50.0/24 <-> Base <-> operator link 192.168.60.0/24
ip link set "$WIRED_IF" up
ip addr replace "$BASE_WIRED_ADDR" dev "$WIRED_IF"
sysctl -w net.ipv4.ip_forward=1 >/dev/null

# If temporary Internet sharing was used previously, return to normal routing.
ip route del default via "$LAPTOP_WIRED_IP" dev "$WIRED_IF" 2>/dev/null || true
if command -v nft >/dev/null 2>&1; then
  nft list table ip hansel_temp_internet >/dev/null 2>&1 && nft delete table ip hansel_temp_internet || true
fi

echo "[OK] Base normal routing ready: $BAT_IF <-> $WIRED_IF ($BASE_WIRED_ADDR); NAT disabled, no laptop default route"
