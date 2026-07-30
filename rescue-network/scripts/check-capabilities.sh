#!/usr/bin/env bash
# Read-only capability probe. Changes NOTHING. Run this first to see whether a
# node can host an AP and/or a B.A.T.M.A.N. mesh with its current hardware.
#
#   ./scripts/check-capabilities.sh
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

section() { printf '\n%s== %s ==%s\n' "$_c_blu" "$*" "$_c_reset"; }

section "Operating system"
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  info "OS: ${PRETTY_NAME:-unknown} (${ID:-?} ${VERSION_ID:-?})"
else
  warn "/etc/os-release not found (not a Linux distro?)"
fi
info "Kernel: $(uname -srm 2>/dev/null || echo unknown)"

section "Raspberry Pi detection"
if [[ -r /proc/device-tree/model ]]; then
  ok "Model: $(tr -d '\0' </proc/device-tree/model)"
elif grep -qi raspberry /proc/cpuinfo 2>/dev/null; then
  ok "Raspberry Pi (via /proc/cpuinfo)"
else
  warn "Not detected as a Raspberry Pi"
fi

section "Network management"
detect_mgr() {
  if have systemctl; then
    systemctl is-active --quiet NetworkManager 2>/dev/null && { echo "NetworkManager"; return; }
    systemctl is-active --quiet systemd-networkd 2>/dev/null && { echo "systemd-networkd"; return; }
    systemctl is-active --quiet dhcpcd 2>/dev/null && { echo "dhcpcd"; return; }
  fi
  echo "unknown"
}
info "Active manager: $(detect_mgr)"
for svc in hostapd dnsmasq; do
  if have systemctl; then
    info "${svc}: $(systemctl is-enabled "$svc" 2>/dev/null || echo n/a) / $(systemctl is-active "$svc" 2>/dev/null || echo inactive)"
  fi
done

section "Wireless interfaces"
ifaces=$(wireless_interfaces)
if [[ -z "$ifaces" ]]; then
  warn "No wireless interfaces found"
else
  for i in $ifaces; do info "iface: $i"; done
fi

section "iw list (PHY capabilities)"
if have iw; then
  iw list 2>/dev/null | sed -n '1,40p'
  dim "... (truncated; run 'iw list' for full output)"
else
  warn "iw not installed (apt install iw)"
fi

section "Supported interface modes per iface"
for i in $ifaces; do
  for mode in "AP" "IBSS" "mesh point"; do
    if iface_supports_mode "$i" "$mode"; then
      ok "${i}: ${mode} supported"
    else
      case $? in
        2) warn "${i}: ${mode} — cannot determine (iw missing?)";;
        *) warn "${i}: ${mode} NOT supported";;
      esac
    fi
  done
done

section "B.A.T.M.A.N.-adv"
if have modinfo && modinfo batman_adv >/dev/null 2>&1; then
  ok "batman_adv kernel module available"
elif lsmod 2>/dev/null | grep -q batman_adv; then
  ok "batman_adv currently loaded"
else
  warn "batman_adv module not found (needed for Phase 5)"
fi
if have batctl; then ok "batctl installed: $(batctl -v 2>/dev/null | head -n1)"; else warn "batctl not installed"; fi

section "Drivers & USB IDs"
for i in $ifaces; do
  drv="?"; [[ -e "/sys/class/net/$i/device/driver" ]] && drv="$(basename "$(readlink -f "/sys/class/net/$i/device/driver")")"
  usb="$(cat "/sys/class/net/$i/device/uevent" 2>/dev/null | grep -E '^PRODUCT=' || true)"
  info "${i}: driver=${drv} ${usb:+($usb)}"
done
if have lsusb; then dim "lsusb:"; lsusb 2>/dev/null | sed 's/^/    /'; fi

section "rfkill status"
if have rfkill; then rfkill list 2>/dev/null | sed 's/^/    /'; else warn "rfkill not installed"; fi

printf '\n'
ok "Capability probe complete (nothing was changed)."
