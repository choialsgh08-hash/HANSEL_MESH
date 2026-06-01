#!/bin/bash

BAT_IF="bat0"
MESH_IF="wlan0"

echo "========================================"
echo " HANSEL_MESH status check"
echo "========================================"

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
echo "[4] BATMAN neighbors"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl n || true
else
    echo "[WARN] batctl not installed."
fi

echo ""
echo "[5] BATMAN originators"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl o || true
else
    echo "[WARN] batctl not installed."
fi

echo ""
echo "[6] Kernel module"
lsmod | grep batman || echo "[WARN] batman-adv module not loaded."

echo ""
echo "========================================"
echo " Check complete."
echo "========================================"