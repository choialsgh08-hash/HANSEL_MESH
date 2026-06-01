#!/bin/bash

set -e

echo "========================================"
echo " HANSEL_MESH start script"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/start_mesh.sh configs/head.env"
    exit 1
fi

CONFIG_FILE="$1"

if [ -z "$CONFIG_FILE" ]; then
    echo "[ERROR] Config file is required."
    echo "Usage:"
    echo "sudo ./scripts/start_mesh.sh configs/base.env"
    echo "sudo ./scripts/start_mesh.sh configs/head.env"
    echo "sudo ./scripts/start_mesh.sh configs/node1.env"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[ERROR] Config file not found: $CONFIG_FILE"
    exit 1
fi

source "$CONFIG_FILE"

if [ -z "$NODE_NAME" ] || [ -z "$MESH_IF" ] || [ -z "$BAT_IF" ] || [ -z "$MESH_ID" ] || [ -z "$MESH_FREQ" ] || [ -z "$IP_ADDR" ] || [ -z "$NETMASK_CIDR" ]; then
    echo "[ERROR] Config file has missing required variables."
    echo "Required: NODE_NAME, MESH_IF, BAT_IF, MESH_ID, MESH_FREQ, IP_ADDR, NETMASK_CIDR"
    exit 1
fi

echo "[INFO] Node name : $NODE_NAME"
echo "[INFO] Mesh IF   : $MESH_IF"
echo "[INFO] BAT IF    : $BAT_IF"
echo "[INFO] Mesh ID   : $MESH_ID"
echo "[INFO] Mesh freq : $MESH_FREQ"
echo "[INFO] IP addr   : $IP_ADDR/$NETMASK_CIDR"

if ! ip link show "$MESH_IF" >/dev/null 2>&1; then
    echo "[ERROR] Interface not found: $MESH_IF"
    echo "Check available interfaces:"
    echo "ip link"
    exit 1
fi

echo "[1/10] Unblocking Wi-Fi..."
rfkill unblock wifi || true

echo "[2/10] Loading batman-adv..."
modprobe batman-adv

echo "[3/10] Stopping NetworkManager/dhcpcd interference if possible..."
systemctl stop hostapd 2>/dev/null || true
systemctl stop dnsmasq 2>/dev/null || true

echo "[4/10] Cleaning old BATMAN interface..."
if ip link show "$BAT_IF" >/dev/null 2>&1; then
    ip link set "$BAT_IF" down || true
    ip link delete "$BAT_IF" type batadv || true
fi

echo "[5/10] Resetting mesh interface..."
ip link set "$MESH_IF" down || true
ip addr flush dev "$MESH_IF" || true
iw dev "$MESH_IF" set type ibss

echo "[6/10] Bringing mesh interface up..."
ip link set "$MESH_IF" up

echo "[7/10] Joining IBSS network..."
iw dev "$MESH_IF" ibss join "$MESH_ID" "$MESH_FREQ"

echo "[8/10] Creating BATMAN interface..."
ip link add name "$BAT_IF" type batadv

echo "[9/10] Adding mesh interface to BATMAN..."
batctl if add "$MESH_IF"

echo "[10/10] Assigning IP to BATMAN interface..."
ip link set up dev "$BAT_IF"
ip addr flush dev "$BAT_IF"
ip addr add "$IP_ADDR/$NETMASK_CIDR" dev "$BAT_IF"

echo "========================================"
echo " Mesh started successfully."
echo "========================================"
ip addr show "$BAT_IF"
echo "========================================"
echo "Check neighbors:"
echo "sudo batctl n"
echo "sudo batctl o"