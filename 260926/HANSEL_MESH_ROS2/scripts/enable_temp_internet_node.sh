#!/bin/bash
# TEMPORARY DEVELOPMENT UTILITY ONLY. Run on head/node1/node2 when apt/git Internet is needed.
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root" >&2; exit 1; }
BASE_BAT_IP="${BASE_BAT_IP:-192.168.50.1}"
BAT_IF="${BAT_IF:-bat0}"
ip link show "$BAT_IF" >/dev/null 2>&1 || { echo "[ERROR] Missing mesh interface: $BAT_IF" >&2; exit 1; }
ip route replace default via "$BASE_BAT_IP" dev "$BAT_IF"
echo "[TEMP] Default route enabled via Base for Internet access."
echo "[TEMP] Disable with: sudo $(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/disable_temp_internet_node.sh"
