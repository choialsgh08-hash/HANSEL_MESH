#!/bin/bash

set -e

ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
SERVICE_SRC="$REPO_DIR/services/hansel-mesh@.service"
SERVICE_DST="/etc/systemd/system/hansel-mesh@.service"

usage() {
    echo "Usage:"
    echo "  sudo ./scripts/enable_mesh_autostart.sh <base|head|node1|node2|node3>"
    echo ""
    echo "Run this once on each Pi after the repo is deployed."
}

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    usage
    exit 1
fi

if [ -z "$ROLE" ]; then
    ROLE="$(hostname | tr '[:upper:]' '[:lower:]')"
fi

case "$ROLE" in
    base|head|node1|node2|node3)
        ;;
    *)
        echo "[ERROR] Unknown role: $ROLE"
        usage
        exit 1
        ;;
esac

if [ ! -f "$REPO_DIR/configs/$ROLE.env" ]; then
    echo "[ERROR] Missing config: $REPO_DIR/configs/$ROLE.env"
    exit 1
fi

if [ ! -f "$SERVICE_SRC" ]; then
    echo "[ERROR] Missing service template: $SERVICE_SRC"
    exit 1
fi

echo "========================================"
echo " HANSEL_MESH autostart enable"
echo "========================================"
echo "Role : $ROLE"
echo "Repo : $REPO_DIR"

chmod +x "$REPO_DIR"/scripts/*.sh
if [ -d "$REPO_DIR/scripts/for_monitor" ]; then
    chmod +x "$REPO_DIR"/scripts/for_monitor/*.sh 2>/dev/null || true
fi

echo "[1/4] Installing systemd service template..."
install -m 0644 "$SERVICE_SRC" "$SERVICE_DST"
systemctl daemon-reload

echo "[2/4] Disabling other HANSEL mesh role instances on this Pi..."
for other in base head node1 node2 node3; do
    if [ "$other" != "$ROLE" ]; then
        systemctl disable "hansel-mesh@$other" 2>/dev/null || true
    fi
done

echo "[3/4] Enabling hansel-mesh@$ROLE..."
systemctl enable "hansel-mesh@$ROLE"

echo "[4/4] Done."
echo "Start now:"
echo "  sudo systemctl start hansel-mesh@$ROLE"
echo ""
echo "Check after reboot:"
echo "  systemctl status hansel-mesh@$ROLE --no-pager"
echo "  journalctl -u hansel-mesh@$ROLE -n 80 --no-pager"
echo "========================================"
