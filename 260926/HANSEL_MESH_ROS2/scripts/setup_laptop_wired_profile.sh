#!/bin/bash
set -e
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
IFACE="${1:-}"
[ -n "$IFACE" ] || { echo "Usage: $0 <wired-interface>"; exit 1; }
PROFILE="${PROFILE:-HANSEL-BASE-LINK}"
ADDR="${LAPTOP_ADDR:-192.168.60.2/24}"
nmcli con delete "$PROFILE" >/dev/null 2>&1 || true
nmcli con add type ethernet ifname "$IFACE" con-name "$PROFILE" ipv4.method manual ipv4.addresses "$ADDR" ipv6.method disabled
nmcli con up "$PROFILE"
echo "[OK] Laptop wired profile ready: $ADDR on $IFACE"
