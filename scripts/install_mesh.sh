#!/bin/bash

set -e

echo "========================================"
echo " HANSEL_MESH install script"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/install_mesh.sh"
    exit 1
fi

echo "[1/5] Updating apt package index..."
apt update

echo "[2/5] Installing required packages..."
apt install -y batctl iw wireless-tools net-tools iproute2 rfkill

echo "[3/5] Enabling batman-adv kernel module at boot..."
echo "batman-adv" > /etc/modules-load.d/batman-adv.conf

echo "[4/5] Loading batman-adv now..."
modprobe batman-adv

echo "[5/5] Checking installed tools..."
command -v batctl >/dev/null 2>&1 || { echo "[ERROR] batctl not found"; exit 1; }
command -v iw >/dev/null 2>&1 || { echo "[ERROR] iw not found"; exit 1; }

echo "========================================"
echo " Install complete."
echo "========================================"
echo "Next:"
echo "sudo ./scripts/start_mesh.sh configs/base.env"