#!/bin/bash
# Shared ROS 2 Lyrical environment loader for HANSEL.
# Intended to be sourced by all build/run wrappers.

_HANSEL_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HANSEL_ROOT="${HANSEL_ROOT:-$(cd "$_HANSEL_ENV_DIR/.." && pwd)}"

_hansel_fail() {
  echo "[ERROR] $*" >&2
  return 1 2>/dev/null || exit 1
}

# Refuse to silently mix this source-built Lyrical environment with another ROS distro.
if [ "${HANSEL_ALLOW_MIXED_ROS:-0}" != "1" ]; then
  if [ -n "${ROS_DISTRO:-}" ] && [ "$ROS_DISTRO" != "lyrical" ]; then
    _hansel_fail "Another ROS distro is already sourced: ROS_DISTRO=$ROS_DISTRO. Open a fresh shell or unset the old ROS environment."
  fi
  _old_prefixes="${AMENT_PREFIX_PATH:-}:${COLCON_PREFIX_PATH:-}:${CMAKE_PREFIX_PATH:-}:${PYTHONPATH:-}"
  case "$_old_prefixes" in
    *'/opt/ros/'*|*'/ros2-linux/'*)
      _hansel_fail "A stale binary/system ROS path is present in the shell environment. Open a fresh shell before running HANSEL."
      ;;
  esac
fi

_ros_owner_home="${HOME}"
if [ ! -f "${_ros_owner_home}/ros2_lyrical/install/local_setup.bash" ] && [ -n "${SUDO_USER:-}" ] && command -v getent >/dev/null 2>&1; then
  _sudo_home="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
  if [ -n "$_sudo_home" ]; then _ros_owner_home="$_sudo_home"; fi
fi

export ROS2_LYRICAL_ROOT="${ROS2_LYRICAL_ROOT:-${_ros_owner_home}/ros2_lyrical}"
ROS2_LYRICAL_SETUP="${ROS2_LYRICAL_SETUP:-${ROS2_LYRICAL_ROOT}/install/local_setup.bash}"
if [ ! -f "$ROS2_LYRICAL_SETUP" ] && [ -f "${ROS2_LYRICAL_ROOT}/install/setup.bash" ]; then
  ROS2_LYRICAL_SETUP="${ROS2_LYRICAL_ROOT}/install/setup.bash"
fi
[ -f "$ROS2_LYRICAL_SETUP" ] || _hansel_fail "ROS 2 Lyrical setup not found: ${ROS2_LYRICAL_ROOT}/install/local_setup.bash"

_hansel_source_setup() {
  local _setup_file="$1"
  local _nounset_was_on=0
  local _rc=0
  case "$-" in *u*) _nounset_was_on=1; set +u ;; esac
  # shellcheck disable=SC1090
  source "$_setup_file" || _rc=$?
  if [ "$_nounset_was_on" -eq 1 ]; then set -u; fi
  return "$_rc"
}

_hansel_source_setup "$ROS2_LYRICAL_SETUP" || _hansel_fail "Failed to source ROS 2 Lyrical: $ROS2_LYRICAL_SETUP"
if [ "${ROS_DISTRO:-}" != "lyrical" ]; then
  _hansel_fail "ROS setup loaded, but ROS_DISTRO='${ROS_DISTRO:-unset}' instead of lyrical."
fi
command -v ros2 >/dev/null 2>&1 || _hansel_fail "ros2 command not found after sourcing $ROS2_LYRICAL_SETUP"

# HANSEL deployment uses CycloneDDS; do not inherit an accidental stale RMW.
export RMW_IMPLEMENTATION="${HANSEL_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

_DDS_CONFIG="${HANSEL_ROOT}/ros2_ws/src/hansel_bringup/config/dds/cyclonedds.xml"
if [ -n "${HANSEL_CYCLONEDDS_URI:-}" ]; then
  export CYCLONEDDS_URI="$HANSEL_CYCLONEDDS_URI"
elif [ -f "$_DDS_CONFIG" ] && command -v ip >/dev/null 2>&1 && ip -4 addr show dev bat0 2>/dev/null | grep -qE '^[[:space:]]*inet[[:space:]]'; then
  export CYCLONEDDS_URI="file://${_DDS_CONFIG}"
else
  unset CYCLONEDDS_URI 2>/dev/null || true
fi

if [ "${HANSEL_SOURCE_WORKSPACE:-1}" != "0" ]; then
  _HANSEL_WS_LOCAL="${HANSEL_ROOT}/ros2_ws/install/local_setup.bash"
  _HANSEL_WS_SETUP="${HANSEL_ROOT}/ros2_ws/install/setup.bash"
  if [ -f "$_HANSEL_WS_LOCAL" ]; then
    _hansel_source_setup "$_HANSEL_WS_LOCAL" || _hansel_fail "Failed to source HANSEL workspace: $_HANSEL_WS_LOCAL"
  elif [ -f "$_HANSEL_WS_SETUP" ]; then
    _hansel_source_setup "$_HANSEL_WS_SETUP" || _hansel_fail "Failed to source HANSEL workspace: $_HANSEL_WS_SETUP"
  else
    _hansel_fail "HANSEL ROS workspace is not built. Run: ${HANSEL_ROOT}/scripts/build_ros2.sh"
  fi
fi

unset _HANSEL_ENV_DIR _ros_owner_home _sudo_home _old_prefixes _DDS_CONFIG _HANSEL_WS_LOCAL _HANSEL_WS_SETUP
unset -f _hansel_source_setup _hansel_fail 2>/dev/null || true
