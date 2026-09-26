#!/bin/bash
# Compatibility wrapper. Despite the historical name, normal operation is routing only.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[INFO] setup_base_gateway.sh now configures normal routed operation only (no NAT/default Internet route)."
exec "$SCRIPT_DIR/setup_base_routing.sh" "$@"
