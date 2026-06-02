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

if [ -z "$NODE_NAME" ] || [ -z "$MESH_IF" ] || [ -z "$BAT_IF" ] || [ -z "$MESH_ID" ] || [ -z "$MESH_FREQ" ] || [ -z "$MESH_MODE" ] || [ -z "$IP_ADDR" ] || [ -z "$NETMASK_CIDR" ]; then
    echo "[ERROR] Config file has missing required variables."
    echo "Required: NODE_NAME, MESH_IF, BAT_IF, MESH_ID, MESH_FREQ, MESH_MODE, IP_ADDR, NETMASK_CIDR"
    exit 1
fi

supports_mode() {
    local mode="$1"
    iw list 2>/dev/null | grep -A 40 "Supported interface modes" | grep -q "\* $mode"
}

choose_mesh_mode() {
    if [ "$MESH_MODE" = "ibss" ]; then
        if supports_mode "IBSS"; then
            echo "ibss"
            return 0
        fi
        echo "[ERROR] MESH_MODE=ibss but this Wi-Fi device does not report IBSS support." >&2
        return 1
    fi

    if [ "$MESH_MODE" = "mesh_point" ]; then
        if supports_mode "mesh point"; then
            echo "mesh_point"
            return 0
        fi
        echo "[ERROR] MESH_MODE=mesh_point but this Wi-Fi device does not report mesh point support." >&2
        return 1
    fi

    if [ "$MESH_MODE" = "auto" ]; then
        if supports_mode "IBSS"; then
            echo "ibss"
            return 0
        fi

        if supports_mode "mesh point"; then
            echo "mesh_point"
            return 0
        fi

        echo "[ERROR] Neither IBSS nor mesh point is supported by this Wi-Fi device." >&2
        return 1
    fi

    echo "[ERROR] Unknown MESH_MODE: $MESH_MODE" >&2
    echo "Allowed: auto, ibss, mesh_point" >&2
    return 1
}

echo "[INFO] Node name : $NODE_NAME"
echo "[INFO] Mesh IF   : $MESH_IF"
echo "[INFO] BAT IF    : $BAT_IF"
echo "[INFO] Mesh ID   : $MESH_ID"
echo "[INFO] Mesh freq : $MESH_FREQ"
echo "[INFO] Mesh mode : $MESH_MODE"
echo "[INFO] IP addr   : $IP_ADDR/$NETMASK_CIDR"

if ! ip link show "$MESH_IF" >/dev/null 2>&1; then
    echo "[ERROR] Interface not found: $MESH_IF"
    echo "Check available interfaces:"
    ip link
    exit 1
fi

SELECTED_MODE="$(choose_mesh_mode)"
echo "[INFO] Selected mesh mode: $SELECTED_MODE"

echo "[1/12] Unblocking Wi-Fi..."
rfkill unblock wifi || true

echo "[2/12] Loading batman-adv..."
modprobe batman-adv

echo "[3/12] Stopping possible conflicting services..."
systemctl stop hostapd 2>/dev/null || true
systemctl stop dnsmasq 2>/dev/null || true

echo "[4/12] Removing old BATMAN interface if exists..."
if ip link show "$BAT_IF" >/dev/null 2>&1; then
    ip link set "$BAT_IF" down || true
    ip link delete "$BAT_IF" type batadv || true
fi

echo "[5/12] Resetting mesh interface..."
ip link set "$MESH_IF" down || true
ip addr flush dev "$MESH_IF" || true

echo "[6/12] Setting wireless interface mode..."
if [ "$SELECTED_MODE" = "ibss" ]; then
    iw dev "$MESH_IF" set type ibss
elif [ "$SELECTED_MODE" = "mesh_point" ]; then
    iw dev "$MESH_IF" set type mesh
else
    echo "[ERROR] Invalid selected mode: $SELECTED_MODE"
    exit 1
fi

echo "[7/12] Bringing mesh interface up..."
ip link set "$MESH_IF" up

echo "[8/12] Joining wireless mesh network..."
if [ "$SELECTED_MODE" = "ibss" ]; then
    iw dev "$MESH_IF" ibss join "$MESH_ID" "$MESH_FREQ"
elif [ "$SELECTED_MODE" = "mesh_point" ]; then
    iw dev "$MESH_IF" mesh join "$MESH_ID" freq "$MESH_FREQ"
fi

echo "[9/12] Creating BATMAN interface..."
ip link add name "$BAT_IF" type batadv

echo "[10/12] Adding mesh interface to BATMAN..."
batctl if add "$MESH_IF"

echo "[11/12] Bringing BATMAN interface up..."
ip link set up dev "$BAT_IF"

echo "[12/12] Assigning IP to BATMAN interface..."
ip addr flush dev "$BAT_IF"
ip addr add "$IP_ADDR/$NETMASK_CIDR" dev "$BAT_IF"

echo "========================================"
echo " Mesh started successfully."
echo "========================================"
echo "Node name     : $NODE_NAME"
echo "Selected mode : $SELECTED_MODE"
echo "BATMAN IP     : $IP_ADDR/$NETMASK_CIDR"
echo "========================================"

ip addr show "$BAT_IF"

echo "========================================"
echo "Check:"
echo "sudo batctl n"
echo "sudo batctl o"
echo "ping 192.168.50.1"
echo "========================================"