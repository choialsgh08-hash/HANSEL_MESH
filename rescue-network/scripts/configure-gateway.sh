#!/usr/bin/env bash
# Configure a RECEIVER node to serve a wired rescue laptop from gateway.env.
#
# SAFETY (same rules as the other scripts):
#   * Default is DRY-RUN; files are backed up before replacement.
#   * LAN_INTERFACE is never hardcoded — it comes from gateway.env.
#   * IP forwarding is enabled ONLY when GATEWAY_FORWARD=1, ONLY on this node,
#     via a backed-up sysctl drop-in, and is undone on --down / rollback.
#   * No iptables/nftables flush, no route deletion.
#
# Usage:
#   ./scripts/configure-gateway.sh [--env FILE]            # dry-run plan
#   sudo ./scripts/configure-gateway.sh --env FILE --apply  # write configs
#   sudo ./scripts/configure-gateway.sh --env FILE --up     # bring LAN up
#   sudo ./scripts/configure-gateway.sh --env FILE --down   # bring LAN down
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ENV_FILE=""; DO_UP=0; DO_DOWN=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --env)   ENV_FILE="$2"; shift 2;;
    --apply) APPLY=1; shift;;
    --up)    APPLY=1; DO_UP=1; shift;;
    --down)  APPLY=1; DO_DOWN=1; shift;;
    -h|--help) sed -n '2,20p' "$0"; exit 0;;
    *) die "unknown argument: $1";;
  esac
done
export APPLY

if [[ -z "$ENV_FILE" ]]; then
  if [[ -r /etc/rescue-network/gateway.env ]]; then ENV_FILE=/etc/rescue-network/gateway.env
  else ENV_FILE="${REPO_DIR}/config/examples/gateway.env.example"
       warn "no --env and /etc/rescue-network/gateway.env missing; using EXAMPLE"; fi
fi
[[ -r "$ENV_FILE" ]] || die "env file not readable: $ENV_FILE"
if [[ "$APPLY" -eq 1 && "$ENV_FILE" == *.example ]]; then
  die "refusing to apply from an .example env; copy it to /etc/rescue-network/gateway.env"
fi
info "env: $ENV_FILE"
# shellcheck disable=SC1090
source "$ENV_FILE"

: "${LAN_INTERFACE:?LAN_INTERFACE required}"; : "${LAN_ADDRESS:?}"
: "${LAN_DHCP_START:?}"; : "${LAN_DHCP_END:?}"; : "${LAN_DHCP_LEASE:=12h}"
: "${GATEWAY_FORWARD:=0}"
: "${LAN_DNSMASQ_CONF:=/etc/dnsmasq.d/rescue-lan.conf}"
: "${SYSCTL_CONF:=/etc/sysctl.d/99-rescue-gateway.conf}"
LAN_IP="${LAN_ADDRESS%%/*}"
[[ "$LAN_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "LAN_ADDRESS must be CIDR like 10.20.0.1/24"

section() { printf '\n%s== %s ==%s\n' "$_c_blu" "$*" "$_c_reset"; }

STAGE="$(mktemp -d "${TMPDIR:-/tmp}/rescue-gw.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT
sed -e "s|__LAN_INTERFACE__|${LAN_INTERFACE}|g" \
    -e "s|__LAN_DHCP_START__|${LAN_DHCP_START}|g" \
    -e "s|__LAN_DHCP_END__|${LAN_DHCP_END}|g" \
    -e "s|__LAN_DHCP_LEASE__|${LAN_DHCP_LEASE}|g" \
    -e "s|__LAN_IP__|${LAN_IP}|g" \
    "${REPO_DIR}/config/dnsmasq/dnsmasq-lan.conf.template" >"${STAGE}/rescue-lan.conf"

section "Rendered LAN dnsmasq (${LAN_DNSMASQ_CONF##*/})"
sed 's/^/    /' <"${STAGE}/rescue-lan.conf"
[[ "$GATEWAY_FORWARD" == "1" ]] && warn "GATEWAY_FORWARD=1: IP forwarding will be enabled on THIS node only."

# --- down -----------------------------------------------------------------
if [[ "$DO_DOWN" -eq 1 ]]; then
  section "Bringing LAN gateway DOWN"
  require_root
  have systemctl && run systemctl stop dnsmasq
  run ip addr del "$LAN_ADDRESS" dev "$LAN_INTERFACE"
  [[ "$GATEWAY_FORWARD" == "1" ]] && run sysctl -w net.ipv4.ip_forward=0
  ok "LAN gateway down. Configs remain; use rollback-network.sh to restore files."
  exit 0
fi

# --- apply (write configs; no service restart) ---------------------------
if [[ "$APPLY" -eq 1 ]]; then
  section "Applying gateway config files (backup first, no restart)"
  require_root
  backup_file "$LAN_DNSMASQ_CONF"
  run install -D -m 644 "${STAGE}/rescue-lan.conf" "$LAN_DNSMASQ_CONF"
  if [[ "$GATEWAY_FORWARD" == "1" ]]; then
    backup_file "$SYSCTL_CONF"
    if [[ "$APPLY" -eq 1 ]]; then
      printf '# rescue-network gateway: route the wired laptop into the mesh.\nnet.ipv4.ip_forward=1\n' >"${STAGE}/sysctl.conf"
      run install -D -m 644 "${STAGE}/sysctl.conf" "$SYSCTL_CONF"
    fi
  fi
  ok "Config files written and backed up."
fi

# --- up -------------------------------------------------------------------
if [[ "$DO_UP" -eq 1 ]]; then
  section "Bringing LAN gateway UP"
  require_root
  run ip link set "$LAN_INTERFACE" up
  run ip addr replace "$LAN_ADDRESS" dev "$LAN_INTERFACE"
  if [[ "$GATEWAY_FORWARD" == "1" ]]; then
    run sysctl -w net.ipv4.ip_forward=1
    warn "Mesh nodes need a route back to ${MESH_SUBNET:-<lan>} via this node's bat0 for return traffic."
  fi
  have systemctl && run systemctl restart dnsmasq
  ok "LAN gateway up. Laptop should get a ${LAN_IP%.*}.x address and reach the dashboard."
fi

if [[ "$APPLY" -eq 0 ]]; then
  section "Dry-run only — nothing changed"
  cat <<EOF
Next steps:
  1) sudo cp config/examples/gateway.env.example /etc/rescue-network/gateway.env && edit
  2) sudo ./scripts/configure-gateway.sh --env /etc/rescue-network/gateway.env --apply
  3) sudo ./scripts/configure-gateway.sh --env /etc/rescue-network/gateway.env --up
  4) Plug in the laptop; open http://${LAN_IP}:8080/dashboard
EOF
fi
