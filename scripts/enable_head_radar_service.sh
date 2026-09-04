#!/bin/bash
set -euo pipefail

if [ "${EUID:-$(id -u)}" -ne 0 ]; then
    echo "[ERROR] Run with sudo: sudo ./scripts/enable_head_radar_service.sh" >&2
    exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SOURCE="$ROOT/services/hansel-radar-head.service"
CONFIG_SOURCE="$ROOT/configs/radar/head_mesh.env.example"
SERVICE_TARGET=/etc/systemd/system/hansel-radar-head.service
CONFIG_DIR=/etc/hansel-mesh
CONFIG_TARGET="$CONFIG_DIR/radar-head.env"

if [ ! -f "$SERVICE_SOURCE" ] || [ ! -f "$CONFIG_SOURCE" ]; then
    echo "[ERROR] Radar service files are missing from $ROOT." >&2
    exit 1
fi

install -d -m 0755 "$CONFIG_DIR"
install -m 0644 "$SERVICE_SOURCE" "$SERVICE_TARGET"
if [ ! -f "$CONFIG_TARGET" ]; then
    install -m 0644 "$CONFIG_SOURCE" "$CONFIG_TARGET"
    echo "[INFO] Created $CONFIG_TARGET"
else
    echo "[INFO] Preserved existing $CONFIG_TARGET"
fi

systemctl daemon-reload
systemctl enable hansel-radar-head.service

echo "========================================"
echo " HANSEL Head radar service enabled"
echo "========================================"
echo "Review config : sudo nano $CONFIG_TARGET"
echo "Start now     : sudo systemctl start hansel-radar-head"
echo "Status        : systemctl status hansel-radar-head --no-pager"
echo "Logs          : journalctl -u hansel-radar-head -f"
echo "========================================"
