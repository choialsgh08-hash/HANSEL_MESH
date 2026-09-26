#!/usr/bin/env bash
# Configure the victim Wi-Fi AP (hostapd + dnsmasq) from ap.env.
#
# SAFETY (see README §"네트워크 설정 안전 규칙"):
#   * Default mode is DRY-RUN — nothing is changed, configs are only rendered.
#   * Existing files are BACKED UP before being replaced (rollback-network.sh).
#   * Interface names are never hardcoded — they come from ap.env.
#   * Services are NEVER auto-restarted; bring-up is a separate explicit --up.
#   * No IP forwarding, no NAT, no iptables/nftables flush, no route deletion.
#
# Usage:
#   ./scripts/configure-ap.sh [--env FILE]            # dry-run: render + plan
#   sudo ./scripts/configure-ap.sh --env FILE --apply  # write configs (+backup)
#   sudo ./scripts/configure-ap.sh --env FILE --up     # bring AP up (start svcs)
#   sudo ./scripts/configure-ap.sh --env FILE --down   # bring AP down
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ENV_FILE=""
DO_UP=0; DO_DOWN=0; DO_COMMIT=0; COMMIT_TIMEOUT=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --env)   ENV_FILE="$2"; shift 2;;
    --apply) APPLY=1; shift;;
    --up)    APPLY=1; DO_UP=1; shift;;
    --down)  APPLY=1; DO_DOWN=1; shift;;
    --commit) APPLY=1; DO_COMMIT=1; shift;;
    --commit-timeout) COMMIT_TIMEOUT="$2"; shift 2;;
    -h|--help) sed -n '2,20p' "$0"; exit 0;;
    *) die "unknown argument: $1";;
  esac
done
export APPLY
SELF="${SCRIPT_DIR}/configure-ap.sh"

# --commit just cancels a pending auto-revert; no env needed.
if [[ "$DO_COMMIT" -eq 1 ]]; then commit_changes "ap"; exit 0; fi
[[ "$COMMIT_TIMEOUT" =~ ^[0-9]+$ ]] || die "--commit-timeout needs an integer (seconds)"

# --- locate + load env ----------------------------------------------------
if [[ -z "$ENV_FILE" ]]; then
  if [[ -r /etc/rescue-network/ap.env ]]; then ENV_FILE=/etc/rescue-network/ap.env
  else
    ENV_FILE="${REPO_DIR}/config/examples/ap.env.example"
    warn "no --env given and /etc/rescue-network/ap.env missing; using EXAMPLE ${ENV_FILE}"
    [[ "$APPLY" -eq 1 ]] && die "refusing to --apply with the example env; copy it and pass --env"
  fi
fi
[[ -r "$ENV_FILE" ]] || die "env file not readable: $ENV_FILE"
# Never mutate the system from a committed example file (it has a placeholder
# passphrase). Copy it to /etc/rescue-network/ap.env and edit first.
if [[ "$APPLY" -eq 1 && "$ENV_FILE" == *.example ]]; then
  die "refusing to apply from an .example env; copy it to /etc/rescue-network/ap.env and edit"
fi
info "env: $ENV_FILE"
# shellcheck disable=SC1090
source "$ENV_FILE"

# --- validate -------------------------------------------------------------
: "${AP_INTERFACE:?AP_INTERFACE required in env}"
: "${AP_SSID:?}"; : "${AP_COUNTRY:?}"; : "${AP_BAND:?}"; : "${AP_CHANNEL:?}"
: "${AP_ADDRESS:?}"; : "${DHCP_RANGE_START:?}"; : "${DHCP_RANGE_END:?}"
: "${DHCP_LEASE:=12h}"; : "${AP_SECURITY:=wpa2}"
: "${HOSTAPD_CONF:=/etc/hostapd/hostapd.conf}"
: "${DNSMASQ_CONF:=/etc/dnsmasq.d/rescue-ap.conf}"

case "$AP_BAND" in
  2.4) HW_MODE=g;;
  5)   HW_MODE=a;;
  *)   die "AP_BAND must be 2.4 or 5 (got '$AP_BAND')";;
esac
[[ "$AP_CHANNEL" =~ ^[0-9]+$ ]] || die "AP_CHANNEL must be numeric"
AP_IP="${AP_ADDRESS%%/*}"
[[ "$AP_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "AP_ADDRESS must be CIDR like 192.168.10.1/24"

WPA_BLOCK=""
if [[ "$AP_SECURITY" == "wpa2" ]]; then
  : "${AP_PASSPHRASE:?wpa2 selected but AP_PASSPHRASE not set}"
  len=${#AP_PASSPHRASE}
  (( len >= 8 && len <= 63 )) || die "AP_PASSPHRASE must be 8..63 chars (got $len)"
  WPA_BLOCK=$'wpa=2\nwpa_key_mgmt=WPA-PSK\nrsn_pairwise=CCMP\nwpa_passphrase='"$AP_PASSPHRASE"
elif [[ "$AP_SECURITY" == "open" ]]; then
  warn "AP_SECURITY=open — unencrypted network"
else
  die "AP_SECURITY must be wpa2 or open"
fi

# Interface sanity (hard-fail only when actually changing the system).
if [[ ! -e "/sys/class/net/${AP_INTERFACE}" ]]; then
  msg="AP_INTERFACE '${AP_INTERFACE}' not present on this host"
  [[ "$APPLY" -eq 1 ]] && die "$msg" || warn "$msg (dry-run continues)"
elif ! iface_supports_mode "$AP_INTERFACE" "AP"; then
  msg="'${AP_INTERFACE}' may not support AP mode (check-capabilities.sh)"
  [[ "$DO_UP" -eq 1 ]] && die "$msg" || warn "$msg"
fi

# --- render templates into a staging dir ---------------------------------
STAGE="$(mktemp -d "${TMPDIR:-/tmp}/rescue-ap.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT

render() {  # render <template> <out>
  sed -e "s|__AP_INTERFACE__|${AP_INTERFACE}|g" \
      -e "s|__AP_SSID__|${AP_SSID}|g" \
      -e "s|__AP_COUNTRY__|${AP_COUNTRY}|g" \
      -e "s|__HW_MODE__|${HW_MODE}|g" \
      -e "s|__AP_CHANNEL__|${AP_CHANNEL}|g" \
      -e "s|__AP_IP__|${AP_IP}|g" \
      -e "s|__DHCP_RANGE_START__|${DHCP_RANGE_START}|g" \
      -e "s|__DHCP_RANGE_END__|${DHCP_RANGE_END}|g" \
      -e "s|__DHCP_LEASE__|${DHCP_LEASE}|g" \
      "$1" >"$2"
  # WPA block replaced separately (may contain slashes/newlines).
  if grep -q '__WPA_BLOCK__' "$2"; then
    awk -v blk="$WPA_BLOCK" '{gsub(/__WPA_BLOCK__/, blk)}1' "$2" >"$2.tmp" && mv "$2.tmp" "$2"
  fi
}

render "${REPO_DIR}/config/hostapd/hostapd.conf.template"        "${STAGE}/hostapd.conf"
render "${REPO_DIR}/config/dnsmasq/dnsmasq-rescue.conf.template" "${STAGE}/rescue-ap.conf"
chmod 600 "${STAGE}/hostapd.conf"

# Captive portal: wild-card every DNS name to the AP so OS probes hit our form.
if [[ "${AP_CAPTIVE:-0}" == "1" ]]; then
  printf '\n# Captive portal: resolve all names to the AP (AP_CAPTIVE=1).\naddress=/#/%s\n' \
    "$AP_IP" >>"${STAGE}/rescue-ap.conf"
  warn "AP_CAPTIVE=1: run the field app with CAPTIVE_PORTAL=1 to serve the portal."
fi

section() { printf '\n%s== %s ==%s\n' "$_c_blu" "$*" "$_c_reset"; }
mask() { sed -E 's/(wpa_passphrase=).*/\1********/'; }

section "Rendered hostapd.conf (passphrase masked)"
mask <"${STAGE}/hostapd.conf" | sed 's/^/    /'
section "Rendered dnsmasq (${DNSMASQ_CONF##*/})"
sed 's/^/    /' <"${STAGE}/rescue-ap.conf"

# --- down -----------------------------------------------------------------
if [[ "$DO_DOWN" -eq 1 ]]; then
  section "Bringing AP DOWN"
  require_root
  have systemctl && { run systemctl stop hostapd; run systemctl stop dnsmasq; }
  run ip addr del "$AP_ADDRESS" dev "$AP_INTERFACE"
  ok "AP down (services stopped, address removed). Configs left in place."
  exit 0
fi

# --- apply (write configs; NO restart) -----------------------------------
if [[ "$APPLY" -eq 1 ]]; then
  section "Applying configuration files (backup first, no service restart)"
  require_root
  backup_file "$HOSTAPD_CONF"
  run install -D -m 600 "${STAGE}/hostapd.conf" "$HOSTAPD_CONF"
  backup_file "$DNSMASQ_CONF"
  run install -D -m 644 "${STAGE}/rescue-ap.conf" "$DNSMASQ_CONF"

  # Debian hostapd reads DAEMON_CONF from /etc/default/hostapd.
  if [[ -e /etc/default/hostapd ]]; then
    backup_file /etc/default/hostapd
    run sed -i "s|^#\?DAEMON_CONF=.*|DAEMON_CONF=\"${HOSTAPD_CONF}\"|" /etc/default/hostapd
  fi

  # If NetworkManager is active, stop it from managing the AP interface so
  # hostapd/dnsmasq can own it. We do NOT restart NM here.
  if have nmcli && systemctl is-active --quiet NetworkManager 2>/dev/null; then
    nm_conf="/etc/NetworkManager/conf.d/rescue-unmanaged.conf"
    backup_file "$nm_conf"
    if [[ "$APPLY" -eq 1 ]]; then
      printf '[keyfile]\nunmanaged-devices=interface-name:%s\n' "$AP_INTERFACE" >"$STAGE/nm.conf"
      run install -D -m 644 "$STAGE/nm.conf" "$nm_conf"
    fi
    warn "NetworkManager left running. Apply the unmanaged rule with: nmcli general reload"
  fi
  ok "Config files written and backed up."
fi

# --- up (explicit bring-up) ----------------------------------------------
if [[ "$DO_UP" -eq 1 ]]; then
  section "Bringing AP UP (explicit)"
  require_root
  have rfkill && run rfkill unblock wlan
  run ip link set "$AP_INTERFACE" up
  # Assign the static AP address (idempotent: replace).
  run ip addr replace "$AP_ADDRESS" dev "$AP_INTERFACE"
  if have systemctl; then
    run systemctl unmask hostapd
    run systemctl enable hostapd dnsmasq
    run systemctl restart dnsmasq
    run systemctl restart hostapd
  fi
  ok "AP bring-up commands issued. Verify with ./scripts/verify-network.sh"
  # Optional safety net: auto-revert unless confirmed in time.
  if [[ "$COMMIT_TIMEOUT" -gt 0 ]]; then
    arm_commit_timeout "ap" "$COMMIT_TIMEOUT" "$SELF" --env "$ENV_FILE" --down
  fi
fi

# --- plan-only footer -----------------------------------------------------
if [[ "$APPLY" -eq 0 ]]; then
  section "Dry-run only — nothing changed"
  cat <<EOF
Next steps:
  1) Copy + edit env:   sudo cp config/examples/ap.env.example /etc/rescue-network/ap.env
  2) Write configs:     sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --apply
  3) Bring AP up:       sudo ./scripts/configure-ap.sh --env /etc/rescue-network/ap.env --up
  4) Verify:            ./scripts/verify-network.sh
  5) Roll back anytime: sudo ./scripts/rollback-network.sh
EOF
fi
