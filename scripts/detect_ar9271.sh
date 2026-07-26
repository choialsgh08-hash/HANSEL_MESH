#!/bin/bash
#
# detect_ar9271.sh - report whether the AR9271 / ath9k_htc mesh dongle is ready.
#
# Runs five independent checks (USB present, driver bound, interface up,
# firmware loaded, IBSS/mesh-point supported) and prints a clear per-check
# result plus a summary.
#
# Safe to run WITHOUT the dongle: every check simply reports [MISS] and the
# script still exits 0, so it can be used as a hardware-free smoke test today.
# Pass --strict to exit non-zero when any check fails (for CI / gating).
#
# Config (optional, from configs/<role>.env or the environment):
#   EXPECTED_DRIVER  driver the dongle should bind to   (default: ath9k_htc)
#   MESH_IF          interface the mesh should use       (default: wlan1)
#   AR9271_USB_ID    documented USB vendor:product id    (default: 0cf3:9271)
#                    NOTE: 0cf3:9271 is the commonly documented AR9271 id;
#                    confirm against real `lsusb` output once the dongle arrives.
#
# Usage:
#   ./scripts/detect_ar9271.sh                 # report, always exit 0
#   ./scripts/detect_ar9271.sh --strict        # exit 1 if any check fails
#   sudo ./scripts/detect_ar9271.sh            # dmesg check needs root on some Pis
#   ./scripts/detect_ar9271.sh configs/head.env

set -u

STRICT="no"
CONFIG_FILE=""
for arg in "$@"; do
    case "$arg" in
        --strict) STRICT="yes" ;;
        -h|--help)
            sed -n '2,30p' "$0"
            exit 0
            ;;
        *)
            if [ -f "$arg" ]; then
                CONFIG_FILE="$arg"
            else
                echo "[WARN] ignoring unknown argument: $arg" >&2
            fi
            ;;
    esac
done

if [ -n "$CONFIG_FILE" ]; then
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
fi

EXPECTED_DRIVER="${EXPECTED_DRIVER:-ath9k_htc}"
EXPECTED_IF="${MESH_IF:-wlan1}"
EXPECTED_USB="${AR9271_USB_ID:-0cf3:9271}"

PASS=0
FAIL=0
ok()   { echo "[ OK ] $*"; PASS=$((PASS + 1)); }
miss() { echo "[MISS] $*"; FAIL=$((FAIL + 1)); }
note() { echo "       $*"; }

echo "========================================"
echo " AR9271 / ath9k_htc detection"
echo "========================================"
echo "expected: driver=$EXPECTED_DRIVER  iface=$EXPECTED_IF  usb=$EXPECTED_USB"
echo ""

# --------------------------------------------------------------------------- #
# 1) USB device present
# --------------------------------------------------------------------------- #
echo "[1/5] USB device"
if command -v lsusb >/dev/null 2>&1; then
    USB_LINE="$(lsusb 2>/dev/null | grep -iE "${EXPECTED_USB}|Atheros.*AR9271|AR9271" | head -n 1)"
    if [ -n "$USB_LINE" ]; then
        ok "USB device present"
        note "$USB_LINE"
    else
        miss "no AR9271 ($EXPECTED_USB) in lsusb - dongle not plugged in?"
    fi
else
    miss "lsusb not installed (install with: apt install usbutils)"
fi

# --------------------------------------------------------------------------- #
# 2) Driver bound to a wireless interface
# --------------------------------------------------------------------------- #
echo "[2/5] Driver binding"
FOUND_IF=""
if [ -d /sys/class/net ]; then
    for dev in /sys/class/net/*; do
        [ -e "$dev" ] || continue
        iface="$(basename "$dev")"
        [ "$iface" = "lo" ] && continue
        [ -e "$dev/phy80211" ] || continue          # wireless interfaces only
        drv="$(basename "$(readlink -f "$dev/device/driver" 2>/dev/null)" 2>/dev/null)"
        if [ "$drv" = "$EXPECTED_DRIVER" ]; then
            FOUND_IF="$iface"
            break
        fi
    done
fi
if [ -n "$FOUND_IF" ]; then
    ok "driver $EXPECTED_DRIVER is bound to $FOUND_IF"
    if [ "$FOUND_IF" != "$EXPECTED_IF" ]; then
        note "WARNING: driver iface is '$FOUND_IF' but config MESH_IF='$EXPECTED_IF'."
        note "         Update configs/*.env MESH_IF to '$FOUND_IF' or check udev naming."
    fi
else
    miss "no wireless interface is bound to driver '$EXPECTED_DRIVER'"
fi

# --------------------------------------------------------------------------- #
# 3) Expected interface exists and its link state
# --------------------------------------------------------------------------- #
echo "[3/5] Interface $EXPECTED_IF"
if [ -e "/sys/class/net/$EXPECTED_IF" ]; then
    STATE="$(cat "/sys/class/net/$EXPECTED_IF/operstate" 2>/dev/null || echo unknown)"
    MAC="$(cat "/sys/class/net/$EXPECTED_IF/address" 2>/dev/null || echo unknown)"
    ok "$EXPECTED_IF exists (operstate=$STATE, mac=$MAC)"
else
    miss "$EXPECTED_IF not present"
fi

# --------------------------------------------------------------------------- #
# 4) Firmware loaded (dmesg fingerprint)
# --------------------------------------------------------------------------- #
echo "[4/5] Firmware load"
DMESG_OUT="$(dmesg 2>/dev/null || true)"
if [ -z "$DMESG_OUT" ]; then
    miss "cannot read dmesg (try: sudo $0)"
elif echo "$DMESG_OUT" | grep -iqE "htc_9271|ath9k_htc.*[Ff]irmware"; then
    ok "ath9k_htc firmware line found in dmesg"
    note "$(echo "$DMESG_OUT" | grep -iE "htc_9271|ath9k_htc.*[Ff]irmware" | tail -n 1)"
else
    miss "no ath9k_htc firmware line in dmesg"
fi

# --------------------------------------------------------------------------- #
# 5) Supported interface modes (IBSS is required; mesh point is a bonus)
# --------------------------------------------------------------------------- #
echo "[5/5] Supported modes"
if command -v iw >/dev/null 2>&1; then
    MODES="$(iw list 2>/dev/null | sed -n '/Supported interface modes/,/interface combinations\|Band [0-9]/p')"
    if [ -z "$MODES" ]; then
        miss "could not read supported interface modes from iw list"
    else
        if echo "$MODES" | grep -qiE '^[[:space:]]*\*[[:space:]]*IBSS$'; then
            ok "IBSS mode supported"
        else
            miss "IBSS not listed (this project builds batman-adv on IBSS)"
        fi
        if echo "$MODES" | grep -qiE '^[[:space:]]*\*[[:space:]]*mesh point$'; then
            ok "802.11s mesh point supported (optional)"
        else
            note "mesh point not listed - fine, IBSS fallback is used by start_mesh.sh"
        fi
    fi
else
    miss "iw not installed (install with: apt install iw)"
fi

# --------------------------------------------------------------------------- #
# Summary
# --------------------------------------------------------------------------- #
echo ""
echo "========================================"
if [ "$FAIL" -eq 0 ]; then
    echo " RESULT: FOUND - dongle looks ready ($PASS checks passed)"
    echo "========================================"
    exit 0
fi

echo " RESULT: NOT_FOUND / NOT_READY ($PASS ok, $FAIL missing)"
echo "========================================"
echo "With no AR9271 dongle plugged in, all-missing is expected."
echo "After plugging the dongle in, re-run and expect checks 1-5 to pass."
if [ "$STRICT" = "yes" ]; then
    exit 1
fi
exit 0
