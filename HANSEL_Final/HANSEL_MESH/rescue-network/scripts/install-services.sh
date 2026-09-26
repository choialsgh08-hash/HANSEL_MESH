#!/usr/bin/env bash
#
# Install rescue-network systemd services for one node role.
#
# This installs the APPLICATION services only (Phase 3). It does NOT touch any
# network configuration, does NOT enable or start anything, and never restarts
# NetworkManager/hostapd/etc. It prints the exact enable/start commands for you
# to run after reviewing.
#
# Usage:
#   sudo ./scripts/install-services.sh <field|receiver|relay> [--dry-run]
#
# Override paths via env vars (defaults shown):
#   INSTALL_DIR=/opt/rescue-network     # where the code + venv live
#   SERVICE_USER=rescue                 # dedicated system user
#   ENV_DIR=/etc/rescue-network         # EnvironmentFile location (secrets)
#   STATE_DIR=/var/lib/rescue-network   # writable SQLite data dir
#
set -euo pipefail

ROLE="${1:-}"
DRY_RUN=0
[[ "${2:-}" == "--dry-run" ]] && DRY_RUN=1

INSTALL_DIR="${INSTALL_DIR:-/opt/rescue-network}"
SERVICE_USER="${SERVICE_USER:-rescue}"
ENV_DIR="${ENV_DIR:-/etc/rescue-network}"
STATE_DIR="${STATE_DIR:-/var/lib/rescue-network}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

die() { echo "ERROR: $*" >&2; exit 1; }

case "${ROLE}" in
  field|receiver|relay) ;;
  *) die "role must be one of: field | receiver | relay (got '${ROLE}')";;
esac

# run <cmd...> : echo, and execute unless --dry-run.
run() {
  echo "  + $*"
  [[ "${DRY_RUN}" -eq 1 ]] || "$@"
}

[[ "${DRY_RUN}" -eq 1 || "$(id -u)" -eq 0 ]] || die "must run as root (use sudo) unless --dry-run"

echo "==> rescue-network install (role=${ROLE}, dry_run=${DRY_RUN})"
echo "    INSTALL_DIR=${INSTALL_DIR}  ENV_DIR=${ENV_DIR}  STATE_DIR=${STATE_DIR}  USER=${SERVICE_USER}"

# 1) Dedicated system user (no login, no home shell).
if id "${SERVICE_USER}" >/dev/null 2>&1; then
  echo "==> user ${SERVICE_USER} already exists"
else
  echo "==> creating system user ${SERVICE_USER}"
  run useradd --system --no-create-home --shell /usr/sbin/nologin "${SERVICE_USER}"
fi

# 2) State dir (systemd also manages this via StateDirectory=, but create it now
#    so a manual run works too).
echo "==> ensuring writable data dir ${STATE_DIR}"
run install -d -o "${SERVICE_USER}" -g "${SERVICE_USER}" -m 0750 "${STATE_DIR}"

# 3) EnvironmentFile: copy the example if not already present (never overwrite a
#    real secret file).
echo "==> ensuring EnvironmentFile ${ENV_DIR}/${ROLE}.env"
run install -d -m 0755 "${ENV_DIR}"
if [[ -f "${ENV_DIR}/${ROLE}.env" ]]; then
  echo "    ${ENV_DIR}/${ROLE}.env exists — leaving it untouched"
else
  run install -o root -g "${SERVICE_USER}" -m 0640 \
    "${REPO_DIR}/config/examples/${ROLE}.env.example" "${ENV_DIR}/${ROLE}.env"
  echo "    >>> EDIT ${ENV_DIR}/${ROLE}.env and set RESCUE_SHARED_TOKEN <<<"
fi

# 4) Unit files for this role.
units=()
case "${ROLE}" in
  field)    units=(rescue-field-web.service rescue-forwarder.service);;
  receiver) units=(rescue-receiver.service);;
  relay)    units=();;  # relay app services arrive in Phase 5
esac

if [[ ${#units[@]} -eq 0 ]]; then
  echo "==> no application services for role '${ROLE}' yet (network-only, Phase 5)"
else
  echo "==> installing unit files to /etc/systemd/system"
  for u in "${units[@]}"; do
    run install -m 0644 "${REPO_DIR}/systemd/${u}" "/etc/systemd/system/${u}"
  done
  run systemctl daemon-reload
fi

echo
echo "==> DONE. Review, then enable/start manually:"
echo "    sudoedit ${ENV_DIR}/${ROLE}.env      # set the shared token"
for u in "${units[@]:-}"; do
  [[ -n "${u}" ]] && echo "    sudo systemctl enable --now ${u}"
done
[[ ${#units[@]} -gt 0 ]] && echo "    journalctl -u ${units[0]} -f      # follow logs"
