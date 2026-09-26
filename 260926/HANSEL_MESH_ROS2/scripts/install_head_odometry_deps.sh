#!/bin/bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_DIR="${HANSEL_PYTHON_VENDOR:-$ROOT/vendor/python}"

if ! python3 -m pip --version >/dev/null 2>&1; then
    echo "[ERROR] pip is missing. Install it first:" >&2
    echo "  sudo apt install -y python3-pip" >&2
    exit 1
fi

mkdir -p "$VENDOR_DIR"
python3 -m pip install --upgrade --target "$VENDOR_DIR" \
    Adafruit-Blinka \
    adafruit-circuitpython-bno08x

echo "========================================"
echo " Head odometry Python dependencies ready"
echo "========================================"
echo "Vendor path: $VENDOR_DIR"
echo "Next checks:"
echo "  sudo raspi-config nonint do_i2c 0"
echo "  sudo i2cdetect -y 1"
echo "Expected BNO085 address: usually 4a or 4b"
