#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${1:-}"

if [ -n "$CONFIG_FILE" ]; then
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "[ERROR] Radar config not found: $CONFIG_FILE" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
fi

RADAR_BIND="${RADAR_BIND:-192.168.50.10}"
RADAR_HTTP_PORT="${RADAR_HTTP_PORT:-8081}"
RADAR_SERIAL_PORT="${RADAR_SERIAL_PORT:-}"
RADAR_XDS_SERIAL="${RADAR_XDS_SERIAL:-}"
RADAR_PROFILE="${RADAR_PROFILE:-$ROOT/configs/radar/iwrl6432_2p5d_operator_nearwall_5hz.cfg}"
RADAR_CALIBRATION="${RADAR_CALIBRATION:-$ROOT/configs/radar/calibrations/head-nearwall-16az-adc-5hz.json}"
RADAR_OUTPUT_ROOT="${RADAR_OUTPUT_ROOT:-$ROOT/runtime/radar}"
RADAR_VERIFY_FRAMES="${RADAR_VERIFY_FRAMES:-3}"
RADAR_COMMAND_TIMEOUT="${RADAR_COMMAND_TIMEOUT:-3}"
RADAR_RETRY_INITIAL="${RADAR_RETRY_INITIAL:-0.25}"
RADAR_RETRY_MAX="${RADAR_RETRY_MAX:-1}"

if ! ip -4 -brief addr show bat0 2>/dev/null | grep -Fq "$RADAR_BIND/"; then
    echo "[ERROR] bat0 does not own $RADAR_BIND." >&2
    echo "Start the Head mesh first and verify: ip -brief addr show bat0" >&2
    exit 1
fi
if [ ! -f "$RADAR_PROFILE" ]; then
    echo "[ERROR] Radar profile not found: $RADAR_PROFILE" >&2
    exit 1
fi
if [ -n "$RADAR_CALIBRATION" ] && [ ! -f "$RADAR_CALIBRATION" ]; then
    echo "[ERROR] Radar calibration not found: $RADAR_CALIBRATION" >&2
    exit 1
fi
if ! python3 -c 'import serial' >/dev/null 2>&1; then
    echo "[ERROR] pyserial is missing. Install python3-serial or requirements-sensors.txt." >&2
    exit 1
fi

mkdir -p "$RADAR_OUTPUT_ROOT"

ARGS=(
    --http-bind "$RADAR_BIND"
    --http-port "$RADAR_HTTP_PORT"
    --output-root "$RADAR_OUTPUT_ROOT"
    --cfg "$RADAR_PROFILE"
    --verify-frames "$RADAR_VERIFY_FRAMES"
    --command-timeout "$RADAR_COMMAND_TIMEOUT"
    --retry-initial "$RADAR_RETRY_INITIAL"
    --retry-max "$RADAR_RETRY_MAX"
    --resume-capture
)
if [ -n "$RADAR_CALIBRATION" ]; then
    ARGS+=(--clutter-calibration "$RADAR_CALIBRATION")
fi
if [ -n "$RADAR_SERIAL_PORT" ]; then
    ARGS+=(--port "$RADAR_SERIAL_PORT")
fi
if [ -n "$RADAR_XDS_SERIAL" ]; then
    ARGS+=(--xds-serial "$RADAR_XDS_SERIAL")
fi

echo "========================================"
echo " HANSEL Head radar over BATMAN"
echo "========================================"
echo "Bind        : $RADAR_BIND:$RADAR_HTTP_PORT"
echo "Profile     : $RADAR_PROFILE"
echo "Calibration : ${RADAR_CALIBRATION:-disabled}"
echo "Serial port : ${RADAR_SERIAL_PORT:-auto-detect}"
echo "XDS serial  : ${RADAR_XDS_SERIAL:-auto-detect}"
echo "PC URL      : http://$RADAR_BIND:$RADAR_HTTP_PORT/"
echo "========================================"

exec python3 "$ROOT/scripts/run_radar_stack.py" "${ARGS[@]}"
