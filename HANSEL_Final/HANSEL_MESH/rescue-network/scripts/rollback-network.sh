#!/usr/bin/env bash
# Roll back changes made by configure-ap.sh, using the backup manifest.
#
#   ./scripts/rollback-network.sh                 # dry-run: show what would revert
#   sudo ./scripts/rollback-network.sh --apply    # restore files / remove new ones
#
# Restores every file configure-ap.sh backed up, and removes files it created.
# It does NOT flush iptables, delete routes, or restart NetworkManager. To also
# stop the AP, run: sudo ./scripts/configure-ap.sh --env <file> --down
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

[[ "${1:-}" == "--apply" ]] && APPLY=1
export APPLY

[[ -r "$RESCUE_ROLLBACK_MANIFEST" ]] || die "no rollback manifest at ${RESCUE_ROLLBACK_MANIFEST} (nothing to undo)"
require_root

info "manifest: ${RESCUE_ROLLBACK_MANIFEST}"
# Process in reverse order so the earliest-backed-up state wins.
mapfile -t lines < <(tac "$RESCUE_ROLLBACK_MANIFEST")
for line in "${lines[@]}"; do
  case "$line" in
    BACKUP::*)
      backup="${line#BACKUP::}"; backup="${backup%%::*}"
      original="${line##*::}"
      info "restore ${original} <- ${backup}"
      run cp -a "$backup" "$original"
      ;;
    CREATED::*)
      path="${line#CREATED::}"
      info "remove created file ${path}"
      run rm -f "$path"
      ;;
    *) warn "skipping unrecognized manifest line: $line";;
  esac
done

if [[ "$APPLY" -eq 1 ]]; then
  # Clear the consumed manifest so a second run is a no-op.
  run mv "$RESCUE_ROLLBACK_MANIFEST" "${RESCUE_ROLLBACK_MANIFEST}.done"
  have nmcli && warn "If NetworkManager config changed, reload it: nmcli general reload"
  ok "Rollback complete."
else
  warn "Dry-run only. Re-run with --apply (as root) to actually revert."
fi
