#!/bin/bash
set -e
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
MESH_NET="${MESH_NET:-192.168.50.0/24}"
LAPTOP_NET="${LAPTOP_NET:-192.168.60.0/24}"
BAT_IF="${BAT_IF:-bat0}"
WIRED_IF="${WIRED_IF:-eth0}"
sysctl -w net.ipv4.ip_forward=1 >/dev/null
mkdir -p /etc/sysctl.d
printf 'net.ipv4.ip_forward=1\n' >/etc/sysctl.d/90-hansel-forward.conf
iptables -C FORWARD -i "$WIRED_IF" -o "$BAT_IF" -j ACCEPT 2>/dev/null || iptables -A FORWARD -i "$WIRED_IF" -o "$BAT_IF" -j ACCEPT
iptables -C FORWARD -i "$BAT_IF" -o "$WIRED_IF" -j ACCEPT 2>/dev/null || iptables -A FORWARD -i "$BAT_IF" -o "$WIRED_IF" -j ACCEPT
echo "[OK] Base forwarding enabled between $LAPTOP_NET and $MESH_NET"
