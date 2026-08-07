#!/bin/bash
set -euo pipefail
# Fetch an exact read-only upstream snapshot without overwriting the integrated
# wrappers and path fixes in this directory.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-$ROOT/upstream_snapshot/HANSEL_MESH}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
command -v git >/dev/null 2>&1 || { echo "[ERROR] git is required"; exit 1; }
command -v rsync >/dev/null 2>&1 || { echo "[ERROR] rsync is required"; exit 1; }

git clone --depth 1 https://github.com/choialsgh08-hash/HANSEL_MESH.git "$TMP/HANSEL_MESH"
rm -rf "$DEST"
mkdir -p "$DEST/monitor"
rsync -a "$TMP/HANSEL_MESH/configs/" "$DEST/configs/"
rsync -a "$TMP/HANSEL_MESH/scripts/" "$DEST/scripts/"
rsync -a "$TMP/HANSEL_MESH/services/" "$DEST/services/"
cp "$TMP/HANSEL_MESH/monitor/metrics_agent.py" "$DEST/monitor/metrics_agent.py"
cp "$TMP/HANSEL_MESH/LICENSE" "$DEST/LICENSE"
git -C "$TMP/HANSEL_MESH" rev-parse HEAD > "$DEST/UPSTREAM_COMMIT.txt"
printf '%s\n' \
  'This directory is an exact upstream network snapshot.' \
  'The parent directory contains ROS-integration wrappers and account-path fixes.' \
  > "$DEST/README_INTEGRATION.txt"
echo "[OK] Exact upstream network snapshot written to $DEST"
echo "[INFO] Commit: $(cat "$DEST/UPSTREAM_COMMIT.txt")"
