#!/bin/bash
set -euo pipefail
ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
case "$ROLE" in base|head|node1|node2|node3) ;; *) echo "Usage: $0 base|head|node1|node2|node3"; exit 1 ;; esac
[ -d "$REPO_DIR" ] || { echo "[ERROR] Missing REPO_DIR: $REPO_DIR"; exit 1; }

# The upstream service templates assume /home/hansel/HANSEL_MESH. Substitute
# the actual install location so non-'hansel' accounts such as 'ngt' work.
sed "s#/home/hansel/HANSEL_MESH#$REPO_DIR#g" \
  "$REPO_DIR/services/hansel-mesh@.service" \
  > /etc/systemd/system/hansel-mesh@.service
sed "s#/home/hansel/HANSEL_MESH#$REPO_DIR#g" \
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
