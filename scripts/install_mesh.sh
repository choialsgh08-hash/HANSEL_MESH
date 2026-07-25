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

echo "[1/7] Updating apt package index..."
apt update

echo "[2/7] Installing required packages..."
apt install -y \
    batctl \
    iw \
    wireless-tools \
    net-tools \
    iproute2 \
    rfkill \
    traceroute \
    ffmpeg \
    python3 \
    python3-rpi.gpio \
    rsync

echo "[3/7] Enabling batman-adv kernel module at boot..."
echo "batman-adv" > /etc/modules-load.d/batman-adv.conf

echo "[4/7] Loading batman-adv now..."
modprobe batman-adv

echo "[5/7] Installing systemd service files..."
for service_name in \
    hansel-mesh@.service \
    hansel-control@.service \
    hansel-metrics@.service \
    hansel-camera.service
do
    service_src="$REPO_ROOT/services/$service_name"
    if [ ! -f "$service_src" ]; then
        echo "[ERROR] Service file not found: $service_src"
        exit 1
    fi
    install -m 0644 "$service_src" "/etc/systemd/system/$service_name"
done
systemctl daemon-reload

echo "[6/7] Installing safe service configuration examples..."
install -d -m 0755 /etc/hansel-mesh
install -m 0644 "$REPO_ROOT/configs/metrics.env.example" /etc/hansel-mesh/metrics.env.example
install -m 0644 "$REPO_ROOT/configs/camera.env.example" /etc/hansel-mesh/camera.env.example
install -m 0644 "$REPO_ROOT/configs/control.env.example" /etc/hansel-mesh/control.env.example
if [ ! -f /etc/hansel-mesh/control.env ]; then
    install -m 0644 "$REPO_ROOT/configs/control.env.example" /etc/hansel-mesh/control.env
    echo "[INFO] Installed default control allowlist: 192.168.60.2/32"
else
    echo "[INFO] Preserving existing control allowlist: /etc/hansel-mesh/control.env"
fi

echo "[7/7] Checking installed tools and scripts..."
command -v batctl >/dev/null 2>&1 || { echo "[ERROR] batctl not found"; exit 1; }
command -v iw >/dev/null 2>&1 || { echo "[ERROR] iw not found"; exit 1; }
chmod +x "$REPO_ROOT"/scripts/*.sh
if [ -d "$REPO_ROOT/scripts/for_monitor" ]; then
    chmod +x "$REPO_ROOT"/scripts/for_monitor/*.sh 2>/dev/null || true
fi

echo "========================================"
echo " Install complete."
echo "========================================"
echo "Next:"
echo "Manual one-shot:"
echo "  sudo ./scripts/start_role_network.sh base"
echo "Autostart once per Pi:"
echo "  sudo ./scripts/enable_mesh_autostart.sh base"
echo "  sudo systemctl start hansel-mesh@base"
echo ""
echo "Camera autostart remains disabled. On the head only, review:"
echo "  /etc/hansel-mesh/camera.env.example"
