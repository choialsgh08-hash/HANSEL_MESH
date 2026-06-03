#!/bin/bash

set -e

REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
TARGETS="${TARGETS:-192.168.50.10 192.168.50.11 192.168.50.12}"
REMOTE_USER="${REMOTE_USER:-hansel}"
DO_PULL="no"
DRY_RUN="no"

usage() {
    echo "Usage:"
    echo "  ./scripts/deploy_from_base.sh [--pull] [--dry-run]"
    echo ""
    echo "Run this on base after the mesh is up."
    echo ""
    echo "Environment overrides:"
    echo "  REPO_DIR=/home/hansel/HANSEL_MESH"
    echo "  TARGETS=\"192.168.50.10 192.168.50.11 192.168.50.12\""
    echo "  REMOTE_USER=hansel"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --pull)
            DO_PULL="yes"
            ;;
        --dry-run)
            DRY_RUN="yes"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $1"
            usage
            exit 1
            ;;
    esac
    shift
done

if [ ! -d "$REPO_DIR/.git" ]; then
    echo "[ERROR] Repo not found: $REPO_DIR"
    exit 1
fi

cd "$REPO_DIR"

echo "========================================"
echo " HANSEL_MESH deploy from base"
echo "========================================"
echo "[INFO] Repo    : $REPO_DIR"
echo "[INFO] Targets : $TARGETS"
echo "[INFO] Pull    : $DO_PULL"
echo "[INFO] Dry run : $DRY_RUN"

if [ "$DO_PULL" = "yes" ]; then
    echo "[1/4] Pulling latest code on base..."
    git pull --ff-only
else
    echo "[1/4] Skipping git pull. Use --pull to update base first."
fi

echo "[2/4] Base version:"
git log -1 --oneline

echo "[3/4] Checking required files..."
test -f robot/motor_driver.py
test -f robot/mesh_control_server.py
test -f controller/mesh_control_client.py
test -f scripts/start_camera_stream.sh
test -d monitor

RSYNC_FLAGS="-az --delete"
if [ "$DRY_RUN" = "yes" ]; then
    RSYNC_FLAGS="$RSYNC_FLAGS --dry-run"
fi

EXCLUDES=(
    "--exclude=.git/"
    "--exclude=__pycache__/"
    "--exclude=*.pyc"
    "--exclude=.pytest_cache/"
    "--exclude=.mypy_cache/"
    "--exclude=.ruff_cache/"
)

echo "[4/4] Deploying repo snapshot to mesh targets..."
for target in $TARGETS; do
    echo "----------------------------------------"
    echo "[INFO] Target: $target"

    if ! ping -c 1 -W 2 "$target" >/dev/null 2>&1; then
        echo "[WARN] $target is not reachable by ping. Skipping."
        continue
    fi

    ssh "$REMOTE_USER@$target" "mkdir -p '$REPO_DIR'"

    if command -v rsync >/dev/null 2>&1; then
        rsync $RSYNC_FLAGS "${EXCLUDES[@]}" "$REPO_DIR/" "$REMOTE_USER@$target:$REPO_DIR/"
    else
        echo "[ERROR] rsync is not installed on base."
        echo "Install it on base, or use scp manually."
        exit 1
    fi

    ssh "$REMOTE_USER@$target" "cd '$REPO_DIR' && python3 -m py_compile robot/motor_driver.py robot/mesh_control_server.py controller/mesh_control_client.py"
    echo "[OK] $target deployed"
done

echo "========================================"
echo " Deploy finished."
echo "========================================"
