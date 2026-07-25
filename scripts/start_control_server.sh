#!/bin/bash

set -euo pipefail

ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"

case "$ROLE" in
    head|node1|node2|node3)
        ;;
    *)
        echo "[ERROR] Motor role must be head, node1, node2, or node3."
        exit 1
        ;;
esac

CONFIG_FILE="$REPO_DIR/configs/$ROLE.env"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "[ERROR] Missing role config: $CONFIG_FILE"
    exit 1
fi

# shellcheck disable=SC1090
source "$CONFIG_FILE"

if [ -z "${IP_ADDR:-}" ]; then
    echo "[ERROR] IP_ADDR is missing from $CONFIG_FILE"
    exit 1
fi

cd "$REPO_DIR"
exec /usr/bin/python3 -m robot.mesh_control_server \
    --role "$ROLE" \
    --host "$IP_ADDR" \
    --require-source-allowlist
