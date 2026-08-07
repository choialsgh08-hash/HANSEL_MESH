#!/bin/bash
set -e
ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
IFACE_WAIT_TIMEOUT="${IFACE_WAIT_TIMEOUT:-45}"
MESH_STARTED_FOR_ROLE="no"

if [ "$EUID" -ne 0 ]; then echo "[ERROR] Run as root"; exit 1; fi
if [ -z "$ROLE" ]; then echo "[ERROR] Role required: base, head, node1, node2, node3"; exit 1; fi
CONFIG_FILE="$REPO_DIR/configs/$ROLE.env"
[ -f "$CONFIG_FILE" ] || { echo "[ERROR] Config not found: $CONFIG_FILE"; exit 1; }
# shellcheck disable=SC1090
source "$CONFIG_FILE"
cleanup_failed_start() {
    local status="$?"; trap - EXIT
    if [ "$status" -ne 0 ] && [ "$MESH_STARTED_FOR_ROLE" = "yes" ]; then
        "$REPO_DIR/scripts/stop_mesh.sh" "$CONFIG_FILE" || true
    fi
    exit "$status"
}
trap cleanup_failed_start EXIT
waited=0
while ! ip link show "$MESH_IF" >/dev/null 2>&1; do
    [ "$waited" -lt "$IFACE_WAIT_TIMEOUT" ] || { echo "[ERROR] Interface not found: $MESH_IF"; exit 1; }
    sleep 1; waited=$((waited + 1))
done
sleep 2
"$REPO_DIR/scripts/start_mesh.sh" "$CONFIG_FILE"
MESH_STARTED_FOR_ROLE="yes"
case "$ROLE" in
  base) "$REPO_DIR/scripts/setup_base_gateway.sh" ;;
  head|node1|node2|node3) "$REPO_DIR/scripts/setup_mesh_route_to_laptop.sh" ;;
  *) echo "[WARN] No role-specific route for $ROLE" ;;
esac
echo "[OK] Role network ready: $ROLE"
