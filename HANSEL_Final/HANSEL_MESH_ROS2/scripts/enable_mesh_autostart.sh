#!/bin/bash
set -euo pipefail
ROLE="${1:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
case "$ROLE" in base|head|node1|node2) ;; *) echo "Usage: $0 base|head|node1|node2"; exit 1 ;; esac
[ -d "$REPO_DIR" ] || { echo "[ERROR] Missing REPO_DIR: $REPO_DIR"; exit 1; }

# Substitute the actual repository location into the systemd templates.
sed "s#@HANSEL_REPO_DIR@#$REPO_DIR#g" \
  "$REPO_DIR/services/hansel-mesh@.service" \
  > /etc/systemd/system/hansel-mesh@.service
sed "s#@HANSEL_REPO_DIR@#$REPO_DIR#g" \
  "$REPO_DIR/services/hansel-metrics@.service" \
  > /etc/systemd/system/hansel-metrics@.service
chmod 0644 /etc/systemd/system/hansel-mesh@.service /etc/systemd/system/hansel-metrics@.service

mkdir -p /etc/hansel-mesh
if [ ! -f /etc/hansel-mesh/metrics.env ]; then
  install -m 0600 "$REPO_DIR/configs/metrics.env.example" /etc/hansel-mesh/metrics.env
fi
systemctl daemon-reload
systemctl enable "hansel-mesh@$ROLE.service"
systemctl enable "hansel-metrics@$ROLE.service"
echo "[OK] Enabled hansel-mesh@$ROLE and hansel-metrics@$ROLE"
echo "[INFO] Installed service WorkingDirectory=$REPO_DIR"
