#!/bin/bash

set -e

ROLE="${1:-}"
REPO_DIR="${REPO_DIR:-/home/hansel/HANSEL_MESH}"
ENABLE_CAMERA="no"

if [ "${2:-}" = "--with-camera" ]; then
    ENABLE_CAMERA="yes"
elif [ -n "${2:-}" ]; then
    echo "[ERROR] Unknown option: $2"
    exit 1
fi

usage() {
    echo "Usage:"
    echo "  sudo ./scripts/enable_mesh_autostart.sh <base|head|node1|node2|node3> [--with-camera]"
    echo ""
    echo "Run this once on each Pi after the repo is deployed."
    echo "Camera autostart is allowed only for head and requires"
    echo "/etc/hansel-mesh/camera.env with CAMERA_ENABLED=yes."
}

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    usage
    exit 1
fi

if [ -z "$ROLE" ]; then
    ROLE="$(hostname | tr '[:upper:]' '[:lower:]')"
fi

case "$ROLE" in
    base|head|node1|node2|node3)
        ;;
    *)
        echo "[ERROR] Unknown role: $ROLE"
        usage
        exit 1
        ;;
esac

if [ ! -f "$REPO_DIR/configs/$ROLE.env" ]; then
    echo "[ERROR] Missing config: $REPO_DIR/configs/$ROLE.env"
    exit 1
fi

CONTROL_CONFIG="/etc/hansel-mesh/control.env"
if [ "$ROLE" != "base" ]; then
    if [ ! -f "$CONTROL_CONFIG" ]; then
        echo "[ERROR] Missing control allowlist: $CONTROL_CONFIG"
        echo "Run scripts/install_mesh.sh or copy configs/control.env.example."
        exit 1
    fi
    if ! grep -Eq '^[[:space:]]*HANSEL_CONTROL_ALLOW_SOURCES=\"?[^\"[:space:]]+.*$' "$CONTROL_CONFIG"; then
        echo "[ERROR] $CONTROL_CONFIG must define a non-empty HANSEL_CONTROL_ALLOW_SOURCES."
        exit 1
    fi
fi

CAMERA_CONFIG="/etc/hansel-mesh/camera.env"
if [ "$ENABLE_CAMERA" = "yes" ]; then
    if [ "$ROLE" != "head" ]; then
        echo "[ERROR] --with-camera is valid only for the head role."
        exit 1
    fi
    if [ ! -f "$CAMERA_CONFIG" ]; then
        echo "[ERROR] Missing camera config: $CAMERA_CONFIG"
        echo "Copy and edit: $REPO_DIR/configs/camera.env.example"
        exit 1
    fi
    if ! grep -Eq '^[[:space:]]*CAMERA_ENABLED=(\"?yes\"?)[[:space:]]*$' "$CAMERA_CONFIG"; then
        echo "[ERROR] $CAMERA_CONFIG must contain CAMERA_ENABLED=yes."
        exit 1
    fi
    if ! grep -Eq '^[[:space:]]*CAMERA_DEST_IP=\"?[^\"[:space:]]+\"?[[:space:]]*$' "$CAMERA_CONFIG"; then
        echo "[ERROR] $CAMERA_CONFIG must contain a non-empty CAMERA_DEST_IP."
        exit 1
    fi
fi

echo "========================================"
echo " HANSEL_MESH autostart enable"
echo "========================================"
echo "Role : $ROLE"
echo "Repo : $REPO_DIR"
echo "Camera autostart requested: $ENABLE_CAMERA"

chmod +x "$REPO_DIR"/scripts/*.sh
if [ -d "$REPO_DIR/scripts/for_monitor" ]; then
    chmod +x "$REPO_DIR"/scripts/for_monitor/*.sh 2>/dev/null || true
fi

echo "[1/5] Installing systemd service files..."
for service_name in \
    hansel-mesh@.service \
    hansel-control@.service \
    hansel-metrics@.service \
    hansel-camera.service
do
    service_src="$REPO_DIR/services/$service_name"
    if [ ! -f "$service_src" ]; then
        echo "[ERROR] Missing service file: $service_src"
        exit 1
    fi
    install -m 0644 "$service_src" "/etc/systemd/system/$service_name"
done
systemctl daemon-reload

echo "[2/5] Disabling other HANSEL role instances on this Pi..."
for other in base head node1 node2 node3; do
    if [ "$other" != "$ROLE" ]; then
        systemctl disable --now "hansel-control@$other"
        systemctl disable --now "hansel-metrics@$other"
        systemctl disable --now "hansel-mesh@$other"
        if systemctl is-active --quiet "hansel-control@$other" ||
           systemctl is-active --quiet "hansel-metrics@$other" ||
           systemctl is-active --quiet "hansel-mesh@$other"; then
            echo "[ERROR] Old role $other is still active. Aborting role switch."
            exit 1
        fi
    fi
done

if [ "$ROLE" != "head" ]; then
    systemctl disable --now hansel-camera.service
    if systemctl is-active --quiet hansel-camera.service; then
        echo "[ERROR] Camera service is still active on non-head role."
        exit 1
    fi
fi

echo "[3/5] Enabling network and metrics for $ROLE..."
systemctl enable "hansel-mesh@$ROLE"
systemctl enable "hansel-metrics@$ROLE"

echo "[4/5] Enabling motor control where applicable..."
if [ "$ROLE" = "base" ]; then
    systemctl disable --now "hansel-control@base"
    echo "[INFO] Base has no motor control service."
else
    systemctl enable "hansel-control@$ROLE"
fi

echo "[5/5] Applying camera opt-in..."
if [ "$ENABLE_CAMERA" = "yes" ]; then
    systemctl enable hansel-camera.service
else
    echo "[INFO] Camera service not changed. Use --with-camera for explicit opt-in."
fi

echo "Done."
echo "Start now:"
echo "  sudo systemctl start hansel-mesh@$ROLE"
echo "  sudo systemctl start hansel-metrics@$ROLE"
if [ "$ROLE" != "base" ]; then
    echo "  sudo systemctl start hansel-control@$ROLE"
fi
if [ "$ENABLE_CAMERA" = "yes" ]; then
    echo "  sudo systemctl start hansel-camera.service"
fi
echo ""
echo "Check after reboot:"
echo "  systemctl status hansel-mesh@$ROLE --no-pager"
echo "  systemctl status hansel-metrics@$ROLE --no-pager"
echo "  journalctl -u hansel-mesh@$ROLE -n 80 --no-pager"
echo "========================================"
