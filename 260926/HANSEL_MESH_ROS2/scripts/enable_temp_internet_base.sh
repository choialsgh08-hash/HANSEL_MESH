#!/bin/bash
# TEMPORARY DEVELOPMENT UTILITY ONLY.
# Enables Internet egress for mesh nodes through Base -> operator laptop.
# It is never called by start_role_network.sh or systemd services.
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root" >&2; exit 1; }
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MESH_NET="${MESH_NET:-192.168.50.0/24}"
WIRED_IF="${WIRED_IF:-eth0}"
LAPTOP_WIRED_IP="${LAPTOP_WIRED_IP:-192.168.60.2}"
command -v nft >/dev/null 2>&1 || { echo "[ERROR] nft not found. Install temporarily with: sudo apt install nftables" >&2; exit 1; }

"$SCRIPT_DIR/setup_base_routing.sh"
ip route replace default via "$LAPTOP_WIRED_IP" dev "$WIRED_IF"
nft list table ip hansel_temp_internet >/dev/null 2>&1 && nft delete table ip hansel_temp_internet || true
nft add table ip hansel_temp_internet
nft 'add chain ip hansel_temp_internet postrouting { type nat hook postrouting priority srcnat; policy accept; }'
nft add rule ip hansel_temp_internet postrouting oifname "$WIRED_IF" ip saddr "$MESH_NET" masquerade

echo "[TEMP] Internet sharing enabled on Base. This state is intentionally non-persistent."
echo "[TEMP] Disable with: sudo $SCRIPT_DIR/disable_temp_internet_base.sh"
