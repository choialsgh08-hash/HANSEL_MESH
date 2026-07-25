#!/bin/bash

set -eu
set -o pipefail

REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
TARGETS="${TARGETS:-192.168.50.10 192.168.50.11 192.168.50.12}"
REMOTE_USER="${REMOTE_USER:-hansel}"
DO_PULL="no"
DRY_RUN="no"
DEPLOY_ATTEMPTED="no"
DEPLOY_COMPLETE="no"
DEPLOY_SOURCE=""
DEPLOY_PARENT=""

report_incomplete_deploy() {
    EXIT_CODE="$?"
    trap - EXIT
    if [ "$EXIT_CODE" -ne 0 ] && \
        [ "$DEPLOY_ATTEMPTED" = "yes" ] && \
        [ "$DEPLOY_COMPLETE" = "no" ]
    then
        echo "[SAFETY] Coordinated deployment did not finish on every target."
        echo "[SAFETY] Keep all hansel-control services stopped and rerun the"
        echo "         deployment before enabling motor control."
    fi
    if [ -n "$DEPLOY_SOURCE" ] && [ -n "$DEPLOY_PARENT" ]; then
        case "$DEPLOY_SOURCE" in
            "$DEPLOY_PARENT"/hansel-deploy.*)
                rm -rf -- "$DEPLOY_SOURCE"
                ;;
            *)
                echo "[WARN] Refusing to remove unexpected temp path: $DEPLOY_SOURCE"
                ;;
        esac
    fi
    exit "$EXIT_CODE"
}

trap report_incomplete_deploy EXIT

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

if [ -n "$(git status --porcelain --untracked-files=normal)" ]; then
    echo "[ERROR] Refusing to deploy a dirty working tree."
    echo "[ERROR] Commit, stash, or remove every local change first."
    exit 1
fi

if [ "$DO_PULL" = "yes" ]; then
    echo "[1/8] Pulling latest code on base..."
    git pull --ff-only
else
    echo "[1/8] Skipping git pull. Use --pull to update base first."
fi

if [ -n "$(git status --porcelain --untracked-files=normal)" ]; then
    echo "[ERROR] Pull left a dirty working tree; nothing was deployed."
    exit 1
fi

DEPLOY_REVISION="$(git rev-parse --verify HEAD)"

echo "[2/8] Base version:"
git log -1 --oneline
echo "[INFO] Exact revision: $DEPLOY_REVISION"

echo "[3/8] Checking local tools and required files..."
command -v git >/dev/null 2>&1
command -v mktemp >/dev/null 2>&1
command -v ping >/dev/null 2>&1
command -v ssh >/dev/null 2>&1
command -v tar >/dev/null 2>&1
if ! command -v rsync >/dev/null 2>&1; then
    echo "[ERROR] rsync is not installed on base."
    exit 1
fi

test -f common/control_protocol.py
test -f common/sensor_contract.py
test -f robot/motor_driver.py
test -f robot/mesh_control_server.py
test -f controller/mesh_control_client.py
test -f scripts/start_camera_stream.sh
test -f scripts/start_camera_service.sh
test -f scripts/start_control_server.sh
test -f scripts/start_metrics_agent.sh
test -f scripts/start_role_network.sh
test -f scripts/enable_mesh_autostart.sh
test -f requirements-rpi.txt
test -f requirements-sensors.txt
test -f services/hansel-camera.service
test -f services/hansel-control@.service
test -f services/hansel-mesh@.service
test -f services/hansel-metrics@.service
test -d monitor

echo "[4/8] Building a tracked-only archive of the exact commit..."
DEPLOY_PARENT="$(cd "${TMPDIR:-/tmp}" && pwd -P)"
DEPLOY_SOURCE="$(mktemp -d "$DEPLOY_PARENT/hansel-deploy.XXXXXX")"
case "$DEPLOY_SOURCE" in
    "$DEPLOY_PARENT"/hansel-deploy.*)
        ;;
    *)
        echo "[ERROR] mktemp returned an unexpected path: $DEPLOY_SOURCE"
        exit 1
        ;;
esac
git archive --format=tar "$DEPLOY_REVISION" | tar -xf - -C "$DEPLOY_SOURCE"

RSYNC_FLAGS="-rz --delete --no-times --no-perms --no-owner --no-group --omit-dir-times"
if [ "$DRY_RUN" = "yes" ]; then
    RSYNC_FLAGS="$RSYNC_FLAGS --dry-run"
fi

EXCLUDES=(
    "--exclude=.git/"
    "--exclude=.env"
    "--exclude=.venv/"
    "--exclude=venv/"
    "--exclude=env/"
    "--exclude=build/"
    "--exclude=dist/"
    "--exclude=.idea/"
    "--exclude=.vscode/"
    "--exclude=__pycache__/"
    "--exclude=*.pyc"
    "--exclude=.pytest_cache/"
    "--exclude=.mypy_cache/"
    "--exclude=.ruff_cache/"
    "--exclude=/logs/"
    "--exclude=/missions/"
    "--exclude=/captures/"
    "--exclude=/report/"
    "--exclude=/.lgd-*"
    "--exclude=/monitor_session.jsonl"
    "--exclude=/video_quality.jsonl"
)

SSH_OPTIONS=(-o BatchMode=yes -o ConnectTimeout=5)

echo "[5/8] Preflighting every mesh target..."
UNREADY_TARGETS=""
for target in $TARGETS; do
    if ! ping -c 1 -W 2 "$target" >/dev/null 2>&1; then
        echo "[ERROR] $target is not reachable by ping."
        UNREADY_TARGETS="$UNREADY_TARGETS $target"
        continue
    fi
    if ! ssh "${SSH_OPTIONS[@]}" "$REMOTE_USER@$target" \
        "command -v python3 >/dev/null &&
         command -v rsync >/dev/null &&
         command -v systemctl >/dev/null &&
         command -v pgrep >/dev/null"
    then
        echo "[ERROR] $target failed SSH/tool preflight."
        UNREADY_TARGETS="$UNREADY_TARGETS $target"
    fi
done

if [ -n "$UNREADY_TARGETS" ]; then
    echo "[ERROR] No files were deployed. Unready:$UNREADY_TARGETS"
    echo "[ERROR] Deploy the versioned control protocol to every unit together."
    exit 1
fi

echo "[6/8] Verifying every motor-control service is stopped..."
ACTIVE_CONTROL_TARGETS=""
for target in $TARGETS; do
    if ! ssh "${SSH_OPTIONS[@]}" "$REMOTE_USER@$target" \
        "! systemctl is-active --quiet hansel-control@head.service &&
         ! systemctl is-active --quiet hansel-control@node1.service &&
         ! systemctl is-active --quiet hansel-control@node2.service &&
         ! systemctl is-active --quiet hansel-control@node3.service &&
         ! pgrep -f '[m]esh_control_server.py' >/dev/null &&
         ! pgrep -f '[r]obot[.]mesh_control_server' >/dev/null"
    then
        echo "[ERROR] A motor-control service or manual server is active on $target."
        ACTIVE_CONTROL_TARGETS="$ACTIVE_CONTROL_TARGETS $target"
    fi
done

if [ -n "$ACTIVE_CONTROL_TARGETS" ]; then
    echo "[ERROR] No files were deployed. Active control:$ACTIVE_CONTROL_TARGETS"
    echo "[ERROR] Stop every hansel-control@ROLE service, support the robot,"
    echo "        then rerun this coordinated deployment."
    exit 1
fi

echo "[7/8] Deploying the same committed snapshot to every mesh target..."
DEPLOY_ATTEMPTED="yes"
for target in $TARGETS; do
    echo "----------------------------------------"
    echo "[INFO] Target: $target"

    if [ "$DRY_RUN" = "yes" ]; then
        if ! ssh "${SSH_OPTIONS[@]}" "$REMOTE_USER@$target" \
            "test -d '$REPO_DIR'"
        then
            echo "[ERROR] Dry run cannot create missing target directory: $target:$REPO_DIR"
            exit 1
        fi
    else
        ssh "${SSH_OPTIONS[@]}" "$REMOTE_USER@$target" \
            "mkdir -p '$REPO_DIR'"
    fi

    rsync $RSYNC_FLAGS \
        -e "ssh -o BatchMode=yes -o ConnectTimeout=5" \
        "${EXCLUDES[@]}" \
        "$DEPLOY_SOURCE/" \
        "$REMOTE_USER@$target:$REPO_DIR/"

    if [ "$DRY_RUN" = "yes" ]; then
        echo "[OK] $target dry-run complete"
        continue
    fi

    ssh "${SSH_OPTIONS[@]}" "$REMOTE_USER@$target" \
        "cd '$REPO_DIR' &&
         chmod 0755 scripts/start_camera_service.sh scripts/start_control_server.sh scripts/start_metrics_agent.sh scripts/start_role_network.sh &&
         test -x scripts/start_camera_service.sh &&
         test -x scripts/start_control_server.sh &&
         test -x scripts/start_metrics_agent.sh &&
         test -x scripts/start_role_network.sh &&
         python3 -m py_compile common/control_protocol.py common/sensor_contract.py robot/motor_driver.py robot/mesh_control_server.py controller/mesh_control_client.py &&
         python3 -c 'import common.control_protocol, common.sensor_contract, robot.mesh_control_server, controller.mesh_control_client' &&
         printf '%s\n' '$DEPLOY_REVISION' > .hansel-deployed-revision"
    echo "[OK] $target deployed and import-verified"
done
DEPLOY_COMPLETE="yes"

echo "[8/8] Deployment result:"
echo "[INFO] Revision: $DEPLOY_REVISION"
if [ "$DRY_RUN" = "yes" ]; then
    echo "[INFO] Dry run only; no target files were changed."
else
    echo "[INFO] Every target now has the same committed snapshot."
    echo "[INFO] Reinstall changed unit files if needed, then start control only"
    echo "       after all target checks pass."
fi

echo "========================================"
echo " Deploy finished."
echo "========================================"
