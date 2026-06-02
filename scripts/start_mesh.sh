#!/bin/bash

set -e

echo "========================================"
echo " HANSEL_MESH start script"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/start_mesh.sh configs/base.env"
    exit 1
fi

CONFIG_FILE="$1"

if [ -z "$CONFIG_FILE" ]; then
    echo "[ERROR] Config file is required."
    echo "Usage:"
    echo "sudo ./scripts/start_mesh.sh configs/base.env"
    echo "sudo ./scripts/start_mesh.sh configs/head.env"
    echo "sudo ./scripts/start_mesh.sh configs/node1.env"
    echo "sudo ./scripts/start_mesh.sh configs/node2.env"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[ERROR] Config file not found: $CONFIG_FILE"
    exit 1
fi

# shellcheck disable=SC1090
source "$CONFIG_FILE"

MESH_MODE="${MESH_MODE:-auto}"
IBSS_BSSID="${IBSS_BSSID:-02:12:34:56:78:9a}"
REQUIRE_IBSS_BSSID_MATCH="${REQUIRE_IBSS_BSSID_MATCH:-no}"

if [ -z "$NODE_NAME" ] || [ -z "$MESH_IF" ] || [ -z "$BAT_IF" ] || [ -z "$MESH_ID" ] || [ -z "$MESH_FREQ" ] || [ -z "$IP_ADDR" ] || [ -z "$NETMASK_CIDR" ]; then
    echo "[ERROR] Config file has missing required variables."
    echo "Required: NODE_NAME, MESH_IF, BAT_IF, MESH_ID, MESH_FREQ, IP_ADDR, NETMASK_CIDR"
    exit 1
fi

case "$MESH_MODE" in
    auto|mesh_point|mesh|80211s|ibss)
        ;;
    *)
        echo "[ERROR] Unsupported MESH_MODE: $MESH_MODE"
        echo "Allowed: auto, mesh_point, mesh, 80211s, ibss"
        exit 1
        ;;
esac

supports_mesh_point() {
    iw list 2>/dev/null | grep -qE '^[[:space:]]*\* mesh point$'
}

supports_ibss() {
    iw list 2>/dev/null | grep -qE '^[[:space:]]*\* IBSS$'
}

freq_to_channel() {
    case "$1" in
        2412) echo "1" ;;
        2417) echo "2" ;;
        2422) echo "3" ;;
        2427) echo "4" ;;
        2432) echo "5" ;;
        2437) echo "6" ;;
        2442) echo "7" ;;
        2447) echo "8" ;;
        2452) echo "9" ;;
        2457) echo "10" ;;
        2462) echo "11" ;;
        2467) echo "12" ;;
        2472) echo "13" ;;
        2484) echo "14" ;;
        *)
            return 1
            ;;
    esac
}

show_wireless_state() {
    iw dev "$MESH_IF" info 2>/dev/null || true
    iwconfig "$MESH_IF" 2>/dev/null || true
}

ibss_joined() {
    iwconfig "$MESH_IF" 2>/dev/null | grep -q "ESSID:\"$MESH_ID\"" || return 1
    iwconfig "$MESH_IF" 2>/dev/null | grep -q "ESSID:off/any" && return 1

    if [ "$REQUIRE_IBSS_BSSID_MATCH" = "yes" ]; then
        iwconfig "$MESH_IF" 2>/dev/null | grep -qi "Cell: $IBSS_BSSID" || return 1
    fi
}

reset_mesh_interface() {
    ip link set "$MESH_IF" down 2>/dev/null || true
    iw dev "$MESH_IF" mesh leave 2>/dev/null || true
    iw dev "$MESH_IF" ibss leave 2>/dev/null || true
    ip addr flush dev "$MESH_IF" || true
    iw dev "$MESH_IF" set type managed 2>/dev/null || true
    ip link set "$MESH_IF" down 2>/dev/null || true
}

start_mesh_point() {
    echo "[INFO] Trying 802.11s mesh point mode..."
    reset_mesh_interface
    iw dev "$MESH_IF" set type mesh || return 1
    ip link set "$MESH_IF" up || return 1
    iw dev "$MESH_IF" mesh join "$MESH_ID" freq "$MESH_FREQ" || return 1
}

start_ibss() {
    echo "[INFO] Trying IBSS mode..."
    reset_mesh_interface

    if iw dev "$MESH_IF" set type ibss; then
        ip link set "$MESH_IF" up || return 1
        if iw dev "$MESH_IF" ibss join "$MESH_ID" "$MESH_FREQ" fixed-freq "$IBSS_BSSID"; then
            sleep 1
            ip link set "$MESH_IF" up || return 1
            if ibss_joined; then
                show_wireless_state
                return 0
            fi
            echo "[WARN] iw IBSS command ran, but interface did not join the expected ESSID."
            show_wireless_state
        fi
    fi

    echo "[WARN] iw IBSS setup failed; trying wireless-tools ad-hoc fallback..."
    if ! command -v iwconfig >/dev/null 2>&1; then
        echo "[ERROR] iwconfig is not installed."
        return 1
    fi

    CHANNEL="$(freq_to_channel "$MESH_FREQ")" || {
        echo "[ERROR] Cannot convert MESH_FREQ=$MESH_FREQ to a 2.4 GHz channel."
        return 1
    }

    reset_mesh_interface
    iwconfig "$MESH_IF" mode ad-hoc || return 1
    iwconfig "$MESH_IF" essid "$MESH_ID" ap "$IBSS_BSSID" channel "$CHANNEL" || return 1
    ip link set "$MESH_IF" up || return 1
    sleep 1

    if ! ibss_joined; then
        echo "[ERROR] ad-hoc fallback did not join the expected ESSID."
        show_wireless_state
        return 1
    fi

    show_wireless_state
}

echo "[INFO] Node name : $NODE_NAME"
echo "[INFO] Mesh IF   : $MESH_IF"
echo "[INFO] BAT IF    : $BAT_IF"
echo "[INFO] Mesh ID   : $MESH_ID"
echo "[INFO] Mesh freq : $MESH_FREQ"
echo "[INFO] Mesh mode : $MESH_MODE"
echo "[INFO] IBSS BSSID: $IBSS_BSSID"
echo "[INFO] Require BSSID match: $REQUIRE_IBSS_BSSID_MATCH"
echo "[INFO] IP addr   : $IP_ADDR/$NETMASK_CIDR"

if ! ip link show "$MESH_IF" >/dev/null 2>&1; then
    echo "[ERROR] Interface not found: $MESH_IF"
    echo "Check available interfaces:"
    echo "ip link"
    exit 1
fi

MESH_STARTED="no"

restore_wifi_on_error() {
    EXIT_CODE="$?"
    if [ "$EXIT_CODE" -eq 0 ] || [ "$MESH_STARTED" = "yes" ]; then
        return
    fi

    echo "[ERROR] Mesh start failed. Restoring $MESH_IF to managed Wi-Fi mode..."
    if ip link show "$BAT_IF" >/dev/null 2>&1; then
        ip link set "$BAT_IF" down 2>/dev/null || true
        ip link delete "$BAT_IF" type batadv 2>/dev/null || true
    fi

    if ip link show "$MESH_IF" >/dev/null 2>&1; then
        ip link set "$MESH_IF" down 2>/dev/null || true
        iw dev "$MESH_IF" mesh leave 2>/dev/null || true
        iw dev "$MESH_IF" ibss leave 2>/dev/null || true
        ip addr flush dev "$MESH_IF" 2>/dev/null || true
        iw dev "$MESH_IF" set type managed 2>/dev/null || true
        ip link set "$MESH_IF" up 2>/dev/null || true
    fi

    if command -v nmcli >/dev/null 2>&1; then
        nmcli dev set "$MESH_IF" managed yes 2>/dev/null || true
        nmcli radio wifi on 2>/dev/null || true
    fi

    systemctl restart NetworkManager 2>/dev/null || true
    systemctl restart dhcpcd 2>/dev/null || true
}

trap restore_wifi_on_error EXIT

HAS_MESH_POINT="no"
HAS_IBSS="no"

if supports_mesh_point; then
    HAS_MESH_POINT="yes"
fi

if supports_ibss; then
    HAS_IBSS="yes"
fi

echo "[INFO] Supported mesh point : $HAS_MESH_POINT"
echo "[INFO] Supported IBSS       : $HAS_IBSS"

if [ "$HAS_MESH_POINT" = "no" ] && [ "$HAS_IBSS" = "no" ]; then
    echo "[ERROR] This Wi-Fi adapter supports neither mesh point nor IBSS."
    echo "Check with: iw list | grep -A 40 'Supported interface modes'"
    exit 1
fi

echo "[1/10] Unblocking Wi-Fi..."
rfkill unblock wifi || true

echo "[2/10] Loading batman-adv..."
modprobe batman-adv

echo "[3/10] Releasing Wi-Fi from AP/client managers if possible..."
systemctl stop hostapd 2>/dev/null || true
systemctl stop dnsmasq 2>/dev/null || true
systemctl stop "wpa_supplicant@$MESH_IF" 2>/dev/null || true
systemctl stop wpa_supplicant 2>/dev/null || true
if command -v nmcli >/dev/null 2>&1; then
    nmcli dev set "$MESH_IF" managed no 2>/dev/null || true
fi

echo "[4/10] Cleaning old BATMAN interface..."
if ip link show "$BAT_IF" >/dev/null 2>&1; then
    ip link set "$BAT_IF" down || true
    ip link delete "$BAT_IF" type batadv || true
fi

ACTIVE_MODE=""

echo "[5/10] Selecting wireless mesh mode..."
case "$MESH_MODE" in
    auto)
        if [ "$HAS_MESH_POINT" = "yes" ]; then
            if start_mesh_point; then
                ACTIVE_MODE="mesh_point"
            elif [ "$HAS_IBSS" = "yes" ]; then
                echo "[WARN] mesh point join failed; falling back to IBSS."
                start_ibss
                ACTIVE_MODE="ibss"
            else
                echo "[ERROR] mesh point join failed and IBSS is not supported."
                exit 1
            fi
        elif [ "$HAS_IBSS" = "yes" ]; then
            start_ibss
            ACTIVE_MODE="ibss"
        fi
        ;;
    mesh_point|mesh|80211s)
        if [ "$HAS_MESH_POINT" != "yes" ]; then
            echo "[ERROR] Requested mesh point mode, but this adapter does not support it."
            exit 1
        fi
        start_mesh_point
        ACTIVE_MODE="mesh_point"
        ;;
    ibss)
        if [ "$HAS_IBSS" != "yes" ]; then
            echo "[ERROR] Requested IBSS mode, but this adapter does not support it."
            exit 1
        fi
        start_ibss
        ACTIVE_MODE="ibss"
        ;;
esac

echo "[6/10] Wireless mode active: $ACTIVE_MODE"
ip link set "$MESH_IF" up || true
show_wireless_state

echo "[7/10] Creating BATMAN interface..."
ip link add name "$BAT_IF" type batadv

echo "[8/10] Adding mesh interface to BATMAN..."
batctl if add "$MESH_IF"

echo "[9/10] Assigning IP to BATMAN interface..."
ip link set up dev "$BAT_IF"
ip addr flush dev "$BAT_IF"
ip addr add "$IP_ADDR/$NETMASK_CIDR" dev "$BAT_IF"

echo "[10/10] Verifying BATMAN interface..."
ip addr show "$BAT_IF"
MESH_STARTED="yes"

echo "========================================"
echo " Mesh started successfully."
echo "========================================"
echo "Active wireless mode: $ACTIVE_MODE"
echo "Check neighbors:"
echo "sudo batctl n"
echo "sudo batctl o"
