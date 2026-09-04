#!/bin/bash
set -euo pipefail

HEAD_IP="${1:-192.168.50.10}"
HTTP_PORT="${2:-8081}"
URL="http://$HEAD_IP:$HTTP_PORT/"
API_URL="${URL}api/radar"

if ! ip route get "$HEAD_IP" >/dev/null 2>&1; then
    echo "[ERROR] No route to Head $HEAD_IP." >&2
    echo "Run scripts/setup_laptop_mesh_routes.sh on the operator PC." >&2
    exit 1
fi
if ! curl --fail --silent --show-error --max-time 3 "$API_URL" >/dev/null; then
    echo "[ERROR] Head radar API is not reachable: $API_URL" >&2
    echo "Check PC -> base -> bat0 routing and the Head radar service." >&2
    exit 1
fi

echo "[OK] Head radar API: $API_URL"
echo "Open: $URL"
if command -v xdg-open >/dev/null 2>&1 && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    xdg-open "$URL" >/dev/null 2>&1 &
fi
