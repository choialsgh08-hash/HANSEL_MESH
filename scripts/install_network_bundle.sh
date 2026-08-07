#!/bin/bash
set -euo pipefail
ROLE="${1:-}"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_USER="${SUDO_USER:-hansel}"
TARGET_DIR="${TARGET_DIR:-/home/${DEFAULT_USER}/HANSEL_MESH}"
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
case "$ROLE" in base|head|node1|node2|node3) ;; *) echo "Usage: $0 base|head|node1|node2|node3"; exit 1 ;; esac

mkdir -p "$TARGET_DIR"
rm -rf "$TARGET_DIR/configs" "$TARGET_DIR/scripts" "$TARGET_DIR/services" "$TARGET_DIR/monitor"
cp -a "$SOURCE_DIR/configs" "$TARGET_DIR/"
cp -a "$SOURCE_DIR/scripts" "$TARGET_DIR/"
cp -a "$SOURCE_DIR/services" "$TARGET_DIR/"
cp -a "$SOURCE_DIR/monitor" "$TARGET_DIR/"
chmod +x "$TARGET_DIR/scripts/"*.sh "$TARGET_DIR/monitor/metrics_agent.py"

if id "$DEFAULT_USER" >/dev/null 2>&1; then
  chown -R "$DEFAULT_USER":"$DEFAULT_USER" "$TARGET_DIR"
fi
REPO_DIR="$TARGET_DIR" "$TARGET_DIR/scripts/install_mesh.sh"
REPO_DIR="$TARGET_DIR" "$TARGET_DIR/scripts/enable_mesh_autostart.sh" "$ROLE"
echo "[OK] Network bundle installed to $TARGET_DIR for role=$ROLE"
echo "[NEXT] Edit $TARGET_DIR/configs/$ROLE.env and /etc/hansel-mesh/metrics.env, then run:"
echo "       sudo systemctl start hansel-mesh@$ROLE hansel-metrics@$ROLE"
