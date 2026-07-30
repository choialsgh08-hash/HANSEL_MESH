#!/usr/bin/env bash
# Configure the B.A.T.M.A.N.-adv mesh link (802.11s or IBSS) from mesh.env.
#
# SAFETY (same rules as configure-ap.sh):
#   * Default is DRY-RUN — nothing changes, the bring-up plan is only printed.
#   * Persistent files are BACKED UP before replacement (rollback-network.sh).
#   * MESH_INTERFACE is never hardcoded — it comes from mesh.env.
#   * Only bat0 receives an IP; the radio sub-interface never gets one.
#   * The chosen backend is checked against `iw`; an unsupported mode is refused.
#   * No route deletion, no iptables/nftables flush, no IP forwarding, no NAT.
#
# Usage:
#   ./scripts/configure-batman.sh [--env FILE]            # dry-run plan
#   sudo ./scripts/configure-batman.sh --env FILE --apply  # persist module load
#   sudo ./scripts/configure-batman.sh --env FILE --up     # bring mesh + bat0 up
#   sudo ./scripts/configure-batman.sh --env FILE --down   # tear mesh down
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ENV_FILE=""; DO_UP=0; DO_DOWN=0; DO_COMMIT=0; COMMIT_TIMEOUT=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --env)   ENV_FILE="$2"; shift 2;;
    --apply) APPLY=1; shift;;
    --up)    APPLY=1; DO_UP=1; shift;;
    --down)  APPLY=1; DO_DOWN=1; shift;;
    --commit) APPLY=1; DO_COMMIT=1; shift;;
    --commit-timeout) COMMIT_TIMEOUT="$2"; shift 2;;
    -h|--help) sed -n '2,22p' "$0"; exit 0;;
    *) die "unknown argument: $1";;
  esac
done
export APPLY
SELF="${SCRIPT_DIR}/configure-batman.sh"

if [[ "$DO_COMMIT" -eq 1 ]]; then commit_changes "batman"; exit 0; fi
[[ "$COMMIT_TIMEOUT" =~ ^[0-9]+$ ]] || die "--commit-timeout needs an integer (seconds)"

# --- locate + load env ----------------------------------------------------
if [[ -z "$ENV_FILE" ]]; then
  if [[ -r /etc/rescue-network/mesh.env ]]; then ENV_FILE=/etc/rescue-network/mesh.env
  else ENV_FILE="${REPO_DIR}/config/examples/mesh.env.example"
       warn "no --env and /etc/rescue-network/mesh.env missing; using EXAMPLE ${ENV_FILE}"; fi
fi
[[ -r "$ENV_FILE" ]] || die "env file not readable: $ENV_FILE"
if [[ "$APPLY" -eq 1 && "$ENV_FILE" == *.example ]]; then
  die "refusing to apply from an .example env; copy it to /etc/rescue-network/mesh.env and edit"
fi
info "env: $ENV_FILE"
# shellcheck disable=SC1090
source "$ENV_FILE"

# --- validate -------------------------------------------------------------
: "${MESH_INTERFACE:?MESH_INTERFACE required}"
: "${MESH_BACKEND:?}"; : "${MESH_ID:?}"; : "${MESH_FREQ:?}"; : "${BAT_ADDRESS:?}"
: "${MESH_MTU:=1560}"; : "${MESH_COUNTRY:=}"; : "${MESH_BSSID:=}"
: "${BATMAN_MODULES_LOAD:=/etc/modules-load.d/batman-adv.conf}"

case "$MESH_BACKEND" in
  80211s) MODE_NAME="mesh point";;
  ibss)   MODE_NAME="IBSS";;
  *) die "MESH_BACKEND must be '80211s' or 'ibss' (got '$MESH_BACKEND')";;
esac
[[ "$MESH_FREQ" =~ ^[0-9]+$ ]] || die "MESH_FREQ must be numeric MHz (e.g. 2412)"
BAT_IP="${BAT_ADDRESS%%/*}"
[[ "$BAT_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "BAT_ADDRESS must be CIDR like 10.10.0.1/24"
# Guard the core rule: never assign an IP to the radio, only to bat0.
[[ "$MESH_INTERFACE" != "bat0" ]] || die "MESH_INTERFACE must be the radio, not bat0"

# Interface presence + mode support.
if [[ ! -e "/sys/class/net/${MESH_INTERFACE}" ]]; then
  msg="MESH_INTERFACE '${MESH_INTERFACE}' not present on this host"
  [[ "$APPLY" -eq 1 ]] && die "$msg" || warn "$msg (dry-run continues)"
else
  if iface_supports_mode "$MESH_INTERFACE" "$MODE_NAME"; then
    ok "'${MESH_INTERFACE}' supports ${MODE_NAME}"
  else
    case $? in
      2) warn "cannot confirm ${MODE_NAME} support on '${MESH_INTERFACE}' (iw missing?)";;
      *) msg="dongle '${MESH_INTERFACE}' does NOT support ${MODE_NAME} — pick the other MESH_BACKEND or a different dongle"
         [[ "$DO_UP" -eq 1 ]] && die "$msg" || warn "$msg";;
    esac
  fi
fi

section() { printf '\n%s== %s ==%s\n' "$_c_blu" "$*" "$_c_reset"; }

# batctl interface attach differs across versions; try new syntax then legacy.
mesh_iface_add() {
  if [[ "$APPLY" -eq 1 ]]; then
    printf '%s  $ batctl meshif bat0 interface add %s  (|| batctl if add)%s\n' "$_c_dim" "$MESH_INTERFACE" "$_c_reset"
    batctl meshif bat0 interface add "$MESH_INTERFACE" 2>/dev/null || batctl if add "$MESH_INTERFACE"
  else
    run batctl meshif bat0 interface add "$MESH_INTERFACE"
    dim "    (older batctl: batctl if add ${MESH_INTERFACE})"
  fi
}
mesh_iface_del() {
  if [[ "$APPLY" -eq 1 ]]; then
    batctl meshif bat0 interface del "$MESH_INTERFACE" 2>/dev/null || batctl if del "$MESH_INTERFACE" 2>/dev/null || true
  else
    run batctl meshif bat0 interface del "$MESH_INTERFACE"
  fi
}

# --- down -----------------------------------------------------------------
if [[ "$DO_DOWN" -eq 1 ]]; then
  section "Tearing mesh DOWN"
  require_root
  run ip addr del "$BAT_ADDRESS" dev bat0
  run ip link set bat0 down
  mesh_iface_del
  if [[ "$MESH_BACKEND" == "80211s" ]]; then run ip link set "$MESH_INTERFACE" down
  else run bash -c "iw dev ${MESH_INTERFACE} ibss leave || true"; fi
  ok "Mesh down. batman-adv module + module-load file left in place."
  exit 0
fi

# --- apply (persist module load only; no live change) --------------------
if [[ "$APPLY" -eq 1 && "$DO_UP" -eq 0 ]]; then
  section "Persisting batman-adv module load (no live change)"
  require_root
  backup_file "$BATMAN_MODULES_LOAD"
  run install -D -m 644 "${REPO_DIR}/config/batman/batman-adv.modules-load.conf" "$BATMAN_MODULES_LOAD"
  ok "Wrote ${BATMAN_MODULES_LOAD}. Bring the mesh up with --up."
fi

# --- plan / up: build the bring-up sequence ------------------------------
section "Mesh bring-up plan (${MESH_BACKEND}, id=${MESH_ID}, freq=${MESH_FREQ}MHz)"
[[ "$DO_UP" -eq 1 ]] && require_root

# 1) regulatory + radio ready
[[ -n "$MESH_COUNTRY" ]] && have iw && run iw reg set "$MESH_COUNTRY"
have rfkill && run rfkill unblock wlan
run ip link set "$MESH_INTERFACE" down

# 2) join the mesh cell (radio only — NO IP here)
if [[ "$MESH_BACKEND" == "80211s" ]]; then
  run iw dev "$MESH_INTERFACE" set type mp
  run ip link set "$MESH_INTERFACE" mtu "$MESH_MTU"
  run ip link set "$MESH_INTERFACE" up
  run iw dev "$MESH_INTERFACE" mesh join "$MESH_ID" freq "$MESH_FREQ" HT20
else
  run iw dev "$MESH_INTERFACE" set type ibss
  run ip link set "$MESH_INTERFACE" mtu "$MESH_MTU"
  run ip link set "$MESH_INTERFACE" up
  if [[ -n "$MESH_BSSID" ]]; then
    run iw dev "$MESH_INTERFACE" ibss join "$MESH_ID" "$MESH_FREQ" fixed-freq "$MESH_BSSID"
  else
    run iw dev "$MESH_INTERFACE" ibss join "$MESH_ID" "$MESH_FREQ"
  fi
fi

# 3) attach to batman-adv → bat0, then IP on bat0 ONLY
run modprobe batman_adv
mesh_iface_add
run ip link set "$MESH_INTERFACE" up
run ip link set bat0 up
run ip addr replace "$BAT_ADDRESS" dev bat0

if [[ "$DO_UP" -eq 1 ]]; then
  ok "Mesh up. Verify: ./scripts/verify-network.sh --env ${ENV_FILE}"
  if [[ "$COMMIT_TIMEOUT" -gt 0 ]]; then
    arm_commit_timeout "batman" "$COMMIT_TIMEOUT" "$SELF" --env "$ENV_FILE" --down
  fi
elif [[ "$APPLY" -eq 0 ]]; then
  section "Dry-run only — nothing changed"
  cat <<EOF
Next steps (per node; bat0 IP must be UNIQUE, receiver = 10.10.0.254):
  1) sudo cp config/examples/mesh.env.example /etc/rescue-network/mesh.env && edit
  2) sudo ./scripts/configure-batman.sh --env /etc/rescue-network/mesh.env --apply
  3) sudo ./scripts/configure-batman.sh --env /etc/rescue-network/mesh.env --up
  4) ./scripts/verify-network.sh --env /etc/rescue-network/mesh.env
  5) rollback: sudo ./scripts/configure-batman.sh --env … --down ; sudo ./scripts/rollback-network.sh --apply
EOF
fi
