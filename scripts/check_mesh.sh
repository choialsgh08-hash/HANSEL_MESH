#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_ARG="${1:-}"

if [ -n "$CONFIG_ARG" ]; then
    CONFIG_FILE="$CONFIG_ARG"
    if [ ! -f "$CONFIG_FILE" ] && [ -f "$REPO_ROOT/configs/$CONFIG_ARG.env" ]; then
        CONFIG_FILE="$REPO_ROOT/configs/$CONFIG_ARG.env"
    fi
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "[ERROR] Config file or role not found: $CONFIG_ARG"
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
fi

BAT_IF="${BAT_IF:-bat0}"
MESH_IF="${MESH_IF:-wlan0}"

echo "========================================"
echo " HANSEL_MESH status check"
echo "========================================"
echo "Mesh IF: $MESH_IF"
echo "BAT IF : $BAT_IF"

echo ""
echo "[1] Interface summary"
ip -brief addr

echo ""
echo "[2] BATMAN interface"
if ip link show "$BAT_IF" >/dev/null 2>&1; then
    ip addr show "$BAT_IF"
else
    echo "[WARN] $BAT_IF not found."
fi

echo ""
echo "[3] Wi-Fi interface"
if ip link show "$MESH_IF" >/dev/null 2>&1; then
    iw dev "$MESH_IF" info || true
else
    echo "[WARN] $MESH_IF not found."
fi

echo ""
echo "[4] Supported wireless interface modes"
if command -v iw >/dev/null 2>&1; then
    iw list 2>/dev/null | grep -A 40 "Supported interface modes" || echo "[WARN] Could not read supported interface modes."
else
    echo "[WARN] iw not installed."
fi

echo ""
echo "[5] BATMAN hard interfaces"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl -m "$BAT_IF" if || true
else
    echo "[WARN] batctl not installed."
fi

echo ""
echo "[6] BATMAN neighbors"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl -m "$BAT_IF" n || true
else
    echo "[WARN] batctl not installed."
fi

echo ""
echo "[7] BATMAN originators"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl -m "$BAT_IF" o || true
else
    echo "[WARN] batctl not installed."
fi

echo ""
echo "[8] Kernel module"
lsmod | grep batman || echo "[WARN] batman-adv module not loaded."

echo ""
echo "[9] Route table"
ip route

echo ""
echo "========================================"
echo " Check complete."
echo "========================================"
