#!/bin/bash

set -e

ROLE="$1"

echo "========================================"
echo " HANSEL_MESH first boot setup"
echo "========================================"

if [ "$EUID" -ne 0 ]; then
    echo "[ERROR] Please run as root:"
    echo "sudo ./scripts/pi_first_boot_setup.sh [base|head|node1|node2|node3]"
    exit 1
fi

if [ -n "$ROLE" ]; then
    case "$ROLE" in
        base|head|node1|node2|node3)
            ;;
        *)
            echo "[ERROR] Unsupported role: $ROLE"
            echo "Allowed roles: base, head, node1, node2, node3"
            exit 1
            ;;
    esac
fi

echo "[1/9] Checking OS information..."
cat /etc/os-release || true

if [ -n "$ROLE" ]; then
    echo "[2/9] Setting hostname to $ROLE..."
    hostnamectl set-hostname "$ROLE"
else
    echo "[2/9] Keeping existing hostname..."
fi

echo "[3/9] Updating package index..."
apt update

echo "[4/9] Installing basic tools..."
apt install -y \
    git \
    curl \
    wget \
    vim \
    nano \
    net-tools \
    iproute2 \
    wireless-tools \
    iw \
    rfkill \
    batctl \
    tmux \
    htop \
    tree \
    avahi-daemon \
    traceroute

echo "[5/9] Enabling SSH service..."
systemctl enable ssh
systemctl start ssh

echo "[6/9] Enabling avahi-daemon for .local hostname access..."
systemctl enable avahi-daemon
systemctl start avahi-daemon

echo "[7/9] Enabling batman-adv kernel module at boot..."
echo "batman-adv" > /etc/modules-load.d/batman-adv.conf
modprobe batman-adv

echo "[8/9] Unblocking Wi-Fi..."
rfkill unblock wifi || true

echo "[9/9] Showing system summary..."
echo ""
echo "Hostname:"
hostname

echo ""
echo "Network interfaces:"
ip -brief addr

echo ""
echo "Wi-Fi devices:"
iw dev || true

echo ""
echo "Supported wireless mesh modes:"
iw list 2>/dev/null | grep -A 40 "Supported interface modes" || true

echo ""
echo "BATMAN module:"
lsmod | grep batman || echo "[WARN] batman-adv not loaded."

echo ""
echo "========================================"
echo " First boot setup complete."
echo "========================================"
echo "Next:"
echo "cd ~/HANSEL_MESH"
echo "sudo ./scripts/install_mesh.sh"
if [ -n "$ROLE" ]; then
    echo "sudo ./scripts/start_mesh.sh configs/$ROLE.env"
fi
