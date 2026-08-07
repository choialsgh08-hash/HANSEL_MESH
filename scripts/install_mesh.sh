#!/bin/bash
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "[ERROR] Run as root"; exit 1; }
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y batctl iw iproute2 wireless-tools rfkill network-manager rsync openssh-client python3
modprobe batman-adv
cat >/etc/modules-load.d/hansel-batman.conf <<'MOD'
batman-adv
MOD
mkdir -p /etc/hansel-mesh
printf '[OK] Installed batman-adv tools.\n'
