#!/usr/bin/env bash
# Source this file from Bash before building or launching.
_ros2_fp_setup() {
  local workspace distro
  workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  source /etc/os-release
  case "$VERSION_ID" in
    22.04) distro=humble ;;
    24.04) distro=jazzy ;;
    *) printf 'Target: Ubuntu 22.04/Humble atau Ubuntu 24.04/Jazzy.\n' >&2; return 1 ;;
  esac
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$distro" ]]; then
    printf 'Environment ROS %s aktif; gunakan terminal baru untuk ROS %s.\n' "$ROS_DISTRO" "$distro" >&2
    return 1
  fi
  if [[ ! -f "/opt/ros/$distro/setup.bash" ]]; then
    printf 'ROS %s belum terpasang.\n' "$distro" >&2
    return 1
  fi
  source "/opt/ros/$distro/setup.bash" || return
  if [[ -f "$workspace/install/local_setup.bash" ]]; then
    source "$workspace/install/local_setup.bash" || return
  fi
}
_ros2_fp_setup
_ros2_fp_result=$?
unset -f _ros2_fp_setup
return "$_ros2_fp_result" 2>/dev/null || exit "$_ros2_fp_result"
