#!/usr/bin/env bash
# Read-only verification of the victim AP and/or the B.A.T.M.A.N. mesh.
# Changes NOTHING. Each section self-guards, so it is safe on field / receiver /
# relay nodes alike.
#
#   ./scripts/verify-network.sh [--env FILE]
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

# Load whatever config is available; --env FILE overrides last.
# (rescue-network configures the AP only; the mesh is the existing HANSEL one.)
ENV_OVERRIDE=""
[[ "${1:-}" == "--env" && -n "${2:-}" ]] && ENV_OVERRIDE="$2"
for f in /etc/rescue-network/ap.env "$ENV_OVERRIDE"; do
  # shellcheck disable=SC1090
  [[ -n "$f" && -r "$f" ]] && source "$f"
done

fails=0
ok_or_fail() { local label="$1"; shift; if "$@" >/dev/null 2>&1; then ok "$label"; else err "$label"; ((fails++)); fi; }
section() { printf '\n%s== %s ==%s\n' "$_c_blu" "$*" "$_c_reset"; }

# --- Victim AP ------------------------------------------------------------
AP_INTERFACE="${AP_INTERFACE:-}"; AP_IP="${AP_ADDRESS:-}"; AP_IP="${AP_IP%%/*}"
if [[ -n "$AP_INTERFACE" && -e "/sys/class/net/${AP_INTERFACE}" ]]; then
  section "Victim AP (${AP_INTERFACE})"
  ok_or_fail "interface is UP" bash -c "ip link show '${AP_INTERFACE}' 2>/dev/null | grep -q ',UP'"
  ok_or_fail "AP address ${AP_IP} set" bash -c "ip addr show '${AP_INTERFACE}' 2>/dev/null | grep -qw '${AP_IP}'"
  if have iw; then
    t="$(iw dev "${AP_INTERFACE}" info 2>/dev/null | awk '/type/ {print $2}')"
    [[ "$t" == "AP" ]] && ok "iw type is AP" || { err "iw type is '${t:-unknown}' (expected AP)"; ((fails++)); }
  fi
  if have systemctl; then
    ok_or_fail "hostapd active" systemctl is-active --quiet hostapd
    ok_or_fail "dnsmasq active" systemctl is-active --quiet dnsmasq
  fi
  if have curl && [[ -n "$AP_IP" ]]; then
    code="$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 "http://${AP_IP}/" 2>/dev/null || echo 000)"
    [[ "$code" == "200" ]] && ok "GET http://${AP_IP}/ -> 200" || { err "GET http://${AP_IP}/ -> ${code}"; ((fails++)); }
  fi
else
  dim "(no victim AP configured on this node — skipping AP checks)"
fi

# --- B.A.T.M.A.N. mesh ----------------------------------------------------
MESH_INTERFACE="${MESH_INTERFACE:-}"; BAT_IP="${BAT_ADDRESS:-}"; BAT_IP="${BAT_IP%%/*}"; BAT_PEER="${BAT_PEER:-}"
if [[ -e /sys/class/net/bat0 || -n "$MESH_INTERFACE" ]]; then
  section "B.A.T.M.A.N. mesh (bat0) — existing HANSEL mesh, read-only check"
  ok_or_fail "batman_adv loaded" bash -c "lsmod 2>/dev/null | grep -q batman_adv"
  ok_or_fail "bat0 exists" test -e /sys/class/net/bat0
  ok_or_fail "bat0 is UP" bash -c "ip link show bat0 2>/dev/null | grep -q ',UP'"
  [[ -n "$BAT_IP" ]] && ok_or_fail "bat0 address ${BAT_IP} set" bash -c "ip addr show bat0 2>/dev/null | grep -qw '${BAT_IP}'"
  if have batctl; then
    dim "batctl if:";  batctl if 2>/dev/null | sed 's/^/    /'
    dim "batctl n (neighbours):"; batctl n 2>/dev/null | sed 's/^/    /'
    dim "batctl o (originators):"; batctl o 2>/dev/null | sed 's/^/    /'
    [[ -n "$MESH_INTERFACE" ]] && ok_or_fail "${MESH_INTERFACE} attached to bat0" \
      bash -c "batctl if 2>/dev/null | grep -q '${MESH_INTERFACE}'"
  else
    warn "batctl not installed — cannot inspect mesh"
  fi
  if [[ -n "$BAT_PEER" ]]; then
    ok_or_fail "ping peer bat0 ${BAT_PEER}" ping -c1 -W2 "$BAT_PEER"
  else
    dim "(export BAT_PEER=<peer bat0 IP> before running to ping a mesh peer)"
  fi
else
  dim "(no mesh configured on this node — skipping mesh checks)"
fi

printf '\n'
if [[ "$fails" -eq 0 ]]; then ok "All checks passed."; else err "${fails} check(s) failed."; exit 1; fi
