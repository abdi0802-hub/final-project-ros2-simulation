#!/usr/bin/env bash
# --check is read-only; --install explicitly permits apt changes.
set -eo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:---check}"
if [[ "$mode" != --check && "$mode" != --install ]] || (( $# > 1 )); then
  printf 'Pemakaian: bash scripts/install_dependencies.sh [--check|--install]\n'
  exit 2
fi
source /etc/os-release
case "$VERSION_ID" in
  22.04) distro=humble; integration=ros-humble-ros-gzharmonic; simulator=(gz-harmonic) ;;
  24.04) distro=jazzy; integration=ros-jazzy-ros-gz; simulator=() ;;
  *) printf 'Target: Ubuntu 22.04 atau 24.04, ditemukan %s.\n' "$VERSION_ID"; exit 1 ;;
esac
if [[ ! -f "/opt/ros/$distro/setup.bash" ]]; then
  printf 'Pasang ROS 2 %s terlebih dahulu mengikuti panduan resmi ROS.\n' "$distro"
  exit 1
fi
packages=("$integration" "${simulator[@]}" "ros-$distro-rviz2" "ros-$distro-xacro"
          "ros-$distro-robot-state-publisher" "ros-$distro-teleop-twist-keyboard"
          "ros-$distro-tf2-ros" "ros-$distro-ament-cmake" "ros-$distro-ament-cmake-pytest"
          build-essential cmake python3-colcon-common-extensions python3-yaml python3-pytest)
if [[ "$mode" == --install ]]; then
  if [[ "$distro" == humble ]] && ! apt-cache show "$integration" >/dev/null 2>&1; then
    command -v curl >/dev/null || { printf 'Pasang curl terlebih dahulu.\n'; exit 1; }
    tmp_key="$(mktemp)"
    trap 'rm -f "$tmp_key"' EXIT
    curl --fail --location --proto '=https' --tlsv1.2 \
      https://packages.osrfoundation.org/gazebo.gpg --output "$tmp_key"
    sudo install -m 0644 "$tmp_key" /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable %s main\n' \
      "$(dpkg --print-architecture)" "$VERSION_CODENAME" | sudo tee /etc/apt/sources.list.d/ros2-fp-gazebo-stable.list >/dev/null
  fi
  sudo apt-get update
  printf '\nRencana perubahan dependency:\n'
  apt-get --simulate install "${packages[@]}"
  sudo apt-get install "${packages[@]}"
fi
source "$root_dir/scripts/setup_env.bash"
missing=0
for package in ros_gz_bridge ros_gz_sim rviz2 xacro robot_state_publisher teleop_twist_keyboard tf2_ros ament_cmake ament_cmake_pytest; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    printf '[OK] %s\n' "$package"
  else
    printf '[BELUM] %s\n' "$package"; missing=1
  fi
done
if command -v gz >/dev/null && gz sim --versions 2>/dev/null | grep -Eq '^[[:space:]]*8\.'; then
  printf '[OK] Gazebo Harmonic (gz-sim 8)\n'
else
  printf '[BELUM] Gazebo Harmonic (gz-sim 8); Fortress/Classic tidak memenuhi target.\n'
  missing=1
fi
if [[ "$distro" == humble ]] && ! dpkg-query -W -f='${db:Status-Abbrev}' ros-humble-ros-gzharmonic-bridge 2>/dev/null | grep -q '^ii'; then
  printf '[BELUM] Bridge Humble-Harmonic; gunakan dependency modul sebelumnya atau --install.\n'
  missing=1
fi
for command in colcon cmake make cc c++; do
  if command -v "$command" >/dev/null; then
    printf '[OK] %s\n' "$command"
  else
    printf '[BELUM] %s\n' "$command"; missing=1
  fi
done
if python3 -c 'import yaml, pytest' >/dev/null 2>&1; then
  printf '[OK] Python YAML dan pytest\n'
else
  printf '[BELUM] Python YAML atau pytest\n'; missing=1
fi
printf 'ROS=%s | Ubuntu=%s | DISPLAY=%s\n' "$distro" "$VERSION_ID" "${DISPLAY:-tidak tersedia}"
exit "$missing"
