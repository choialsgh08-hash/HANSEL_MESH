#!/bin/bash

set -e

ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
IFACE_WAIT_TIMEOUT="${IFACE_WAIT_TIMEOUT:-45}"

echo "========================================"
echo " HANSEL_MESH role network start"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/start_role_network.sh base"
    exit 1
fi

if [ -z "$ROLE" ]; then
    echo "[ERROR] Role is required: base, head, node1, node2, node3"
    exit 1
fi

CONFIG_FILE="$REPO_DIR/configs/$ROLE.env"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "[ERROR] Config file not found: $CONFIG_FILE"
    exit 1
fi

# shellcheck disable=SC1090
source "$CONFIG_FILE"

wait_for_interface() {
    local iface="$1"
    local timeout="$2"
    local waited=0

    if [ -z "$iface" ]; then
        echo "[ERROR] MESH_IF is not set in $CONFIG_FILE"
        exit 1
    fi

    echo "[INFO] Waiting for interface $iface up to ${timeout}s..."
    while ! ip link show "$iface" >/dev/null 2>&1; do
        if [ "$waited" -ge "$timeout" ]; then
            echo "[ERROR] Interface not found after ${timeout}s: $iface"
            ip link || true
            exit 1
        fi
        sleep 1
        waited=$((waited + 1))
    done

    # Give Wi-Fi firmware/udev a short settle window before changing interface type.
    sleep 2
}

echo "[INFO] Role : $ROLE"
echo "[INFO] Repo : $REPO_DIR"

echo "[1/2] Starting BATMAN mesh..."
wait_for_interface "$MESH_IF" "$IFACE_WAIT_TIMEOUT"
"$REPO_DIR/scripts/start_mesh.sh" "$CONFIG_FILE"

echo "[2/2] Applying role-specific routing..."
case "$ROLE" in
    base)
        "$REPO_DIR/scripts/setup_base_gateway.sh"
        ;;
    head|node1|node2|node3)
        "$REPO_DIR/scripts/setup_mesh_route_to_laptop.sh"
        ;;
    *)
        echo "[WARN] No role-specific routing rule for role=$ROLE"
        ;;
esac

echo "========================================"
echo " Role network ready: $ROLE"
echo "========================================"
