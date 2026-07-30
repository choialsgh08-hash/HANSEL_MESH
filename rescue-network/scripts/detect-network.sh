#!/usr/bin/env bash
# Read-only network detection. Changes NOTHING. Suggests which interface should
# host the victim AP and which should carry the mesh — never applies anything.
#
#   ./scripts/detect-network.sh
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

section() { printf '\n%s== %s ==%s\n' "$_c_blu" "$*" "$_c_reset"; }

section "Wireless interfaces (driver / bus)"
ifaces=$(wireless_interfaces)
[[ -n "$ifaces" ]] || die "no wireless interfaces found"

internal=""; usb=""
for i in $ifaces; do
  drv="?"; bus="?"
  if [[ -e "/sys/class/net/$i/device/driver" ]]; then
    drv="$(basename "$(readlink -f "/sys/class/net/$i/device/driver")")"
  fi
  # brcmfmac == Raspberry Pi internal Wi-Fi; usb path == dongle.
  if readlink -f "/sys/class/net/$i/device" 2>/dev/null | grep -q usb; then
    bus="usb"; usb="${usb} $i"
  else
    bus="internal"; [[ "$drv" == brcmfmac* || -z "$internal" ]] && internal="${internal} $i"
  fi
  info "${i}: driver=${drv} bus=${bus}"
done

section "Current addresses / routes (read-only)"
if have ip; then
  ip -brief addr 2>/dev/null | sed 's/^/    /'
  dim "default route(s):"
  ip route show default 2>/dev/null | sed 's/^/    /'
else
  warn "iproute2 'ip' not available"
fi

section "Suggestion (NOT applied)"
sug_ap="$(echo $internal | awk '{print $1}')"
sug_mesh="$(echo $usb | awk '{print $1}')"
[[ -n "$sug_ap" ]]   || sug_ap="$(echo $ifaces | awk '{print $1}')"
[[ -n "$sug_mesh" ]] || sug_mesh="(none)"
ok  "Victim AP  -> AP_INTERFACE=${sug_ap}   (internal Wi-Fi preferred)"
dim "Set AP_INTERFACE in /etc/rescue-network/ap.env."
dim "The B.A.T.M.A.N. mesh (likely on ${sug_mesh}) is managed by the existing"
dim "HANSEL scripts (scripts/*mesh*.sh, configs/*.env) — rescue-network does not"
dim "configure it. Point RECEIVER_URL at the receiver's existing bat0 IP."
warn "Verify with ./scripts/check-capabilities.sh that the chosen AP iface supports 'AP' mode."
