#!/bin/bash
# Build deployment artifacts only. Does not connect to or start any robot.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/runtime/deployment/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
for target in base-ros head-ros node-ros nano-firmware; do
  tar -C "$ROOT" -czf "$OUT/$target.tar.gz" -T "$ROOT/deployment/$target.txt"
done
tar -C "$ROOT/../HANSEL_MESH" -czf "$OUT/head-radar.tar.gz" -T "$ROOT/deployment/head-radar.txt"
cp "$ROOT"/deployment/*.txt "$OUT/"
printf 'Deployment archives (not uploaded): %s\n' "$OUT"
