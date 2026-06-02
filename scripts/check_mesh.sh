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
echo "[4] Supported wireless interface modes"
if command -v iw >/dev/null 2>&1; then
    iw list 2>/dev/null | grep -A 40 "Supported interface modes" || echo "[WARN] Could not read supported interface modes."
else
    echo "[WARN] iw not installed."
fi

echo ""
echo "[5] BATMAN hard interfaces"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl if || true
else
    echo "[WARN] batctl not installed."
fi

echo ""
echo "[6] BATMAN neighbors"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl n || true
else
    echo "[WARN] batctl not installed."
fi

echo ""
echo "[7] BATMAN originators"
if command -v batctl >/dev/null 2>&1; then
    sudo batctl o || true
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