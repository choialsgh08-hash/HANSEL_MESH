#!/bin/bash
set -e
printf '=== USB Wi-Fi devices ===\n'
lsusb | grep -Ei 'Atheros|AR9271|0cf3:9271' || true
printf '\n=== wireless interfaces ===\n'
iw dev || true
printf '\n=== supported modes ===\n'
iw list 2>/dev/null | grep -A 20 'Supported interface modes' || true
printf '\n=== candidate external adapters ===\n'
for i in /sys/class/net/wlan*; do
  [ -e "$i" ] || continue
  iface="$(basename "$i")"
  path="$(readlink -f "$i/device")"
  printf '%-8s %s\n' "$iface" "$path"
done
