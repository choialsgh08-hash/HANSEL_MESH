#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "========================================"
echo " HANSEL_MESH install script"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/install_mesh.sh"
    exit 1
fi

echo "[1/6] Updating apt package index..."
apt update

echo "[2/6] Installing required packages..."
apt install -y batctl iw wireless-tools net-tools iproute2 rfkill traceroute

echo "[3/6] Enabling batman-adv kernel module at boot..."
echo "batman-adv" > /etc/modules-load.d/batman-adv.conf

echo "[4/6] Loading batman-adv now..."
modprobe batman-adv

echo "[5/6] Installing systemd service template..."
if [ -f "$REPO_ROOT/services/hansel-mesh@.service" ]; then
    install -m 0644 "$REPO_ROOT/services/hansel-mesh@.service" /etc/systemd/system/hansel-mesh@.service
    systemctl daemon-reload
else
    echo "[WARN] Service template not found: $REPO_ROOT/services/hansel-mesh@.service"
fi

echo "[6/6] Checking installed tools..."
command -v batctl >/dev/null 2>&1 || { echo "[ERROR] batctl not found"; exit 1; }
command -v iw >/dev/null 2>&1 || { echo "[ERROR] iw not found"; exit 1; }

echo "========================================"
echo " Install complete."
echo "========================================"
echo "Next:"
echo "sudo ./scripts/start_mesh.sh configs/base.env"
echo "or:"
echo "sudo systemctl enable hansel-mesh@base"
echo "sudo systemctl start hansel-mesh@base"
