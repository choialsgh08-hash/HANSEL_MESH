#!/bin/bash
set -euo pipefail
CONFIG_FILE="${1:-}"
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
[ -n "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ] || { echo "Usage: $0 configs/<role>.env"; exit 1; }
# shellcheck disable=SC1090
source "$CONFIG_FILE"
: "${MESH_IF:?}" "${BAT_IF:?}" "${MESH_ID:?}" "${MESH_FREQ:?}" "${IP_ADDR:?}" "${NETMASK_CIDR:?}"
MESH_MODE="${MESH_MODE:-auto}"
IBSS_BSSID="${IBSS_BSSID:-02:12:34:56:78:9a}"
WIFI_POWER_SAVE="${WIFI_POWER_SAVE:-off}"
STOP_GLOBAL_WIFI_SERVICES="${STOP_GLOBAL_WIFI_SERVICES:-no}"

modprobe batman-adv
rfkill unblock wifi || true
command -v nmcli >/dev/null && nmcli dev set "$MESH_IF" managed no 2>/dev/null || true
if [ "$STOP_GLOBAL_WIFI_SERVICES" = yes ]; then
  systemctl stop wpa_supplicant@"$MESH_IF".service 2>/dev/null || true
fi
ip link set "$MESH_IF" down || true
iw dev "$MESH_IF" mesh leave 2>/dev/null || true
iw dev "$MESH_IF" ibss leave 2>/dev/null || true
ip addr flush dev "$MESH_IF" || true

supports_mesh=no; supports_ibss=no
iw list 2>/dev/null | grep -q 'mesh point' && supports_mesh=yes || true
iw list 2>/dev/null | grep -q 'IBSS' && supports_ibss=yes || true
mode="$MESH_MODE"
if [ "$mode" = auto ]; then
  if [ "$supports_mesh" = yes ]; then mode=mesh; elif [ "$supports_ibss" = yes ]; then mode=ibss; else echo "[ERROR] Neither mesh point nor IBSS supported"; exit 1; fi
fi
case "$mode" in
  mesh)
    iw dev "$MESH_IF" set type mp
    ip link set "$MESH_IF" up
    iw dev "$MESH_IF" mesh join "$MESH_ID" freq "$MESH_FREQ"
    ;;
  ibss)
    iw dev "$MESH_IF" set type ibss
    ip link set "$MESH_IF" up
    iw dev "$MESH_IF" ibss join "$MESH_ID" "$MESH_FREQ" fixed-freq "$IBSS_BSSID"
    ;;
  *) echo "[ERROR] MESH_MODE must be auto, mesh, or ibss"; exit 1 ;;
esac
[ "$WIFI_POWER_SAVE" != off ] || iw dev "$MESH_IF" set power_save off 2>/dev/null || true

if ip link show "$BAT_IF" >/dev/null 2>&1; then
  ip link set "$BAT_IF" down || true
  ip link delete "$BAT_IF" type batadv 2>/dev/null || true
fi
ip link add name "$BAT_IF" type batadv
batctl -m "$BAT_IF" if add "$MESH_IF"
[ -z "${BATMAN_HOP_PENALTY:-}" ] || batctl -m "$BAT_IF" hop_penalty "$BATMAN_HOP_PENALTY"
[ -z "${BATMAN_ORIG_INTERVAL:-}" ] || batctl -m "$BAT_IF" orig_interval "$BATMAN_ORIG_INTERVAL"
ip link set "$BAT_IF" up
ip addr flush dev "$BAT_IF"
ip addr add "$IP_ADDR/$NETMASK_CIDR" dev "$BAT_IF"
echo "[OK] $NODE_NAME mesh ready: $BAT_IF=$IP_ADDR/$NETMASK_CIDR mode=$mode via $MESH_IF"
