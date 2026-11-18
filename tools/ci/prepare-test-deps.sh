#!/usr/bin/env bash
# Fetch and build the official test suites.
#
# These packages are not distributed through apt. Nothing in them depends on
# rmw_someip_cpp, so CI caches the whole tree.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

DEPS="$WS/.test_deps"
mkdir -p "$DEPS/src"

clone() {
  local name="$1" url="$2"
  [ -d "$DEPS/src/$name" ] && return 0
  log "cloning $name"
  git clone --depth 1 --branch "$ROS_DISTRO" "$url" "$DEPS/src/$name"
}
clone rmw_implementation https://github.com/ros2/rmw_implementation.git
clone system_tests        https://github.com/ros2/system_tests.git

cd "$DEPS"
log "building the official test suites"
colcon build --packages-select \
  test_rmw_implementation test_communication test_quality_of_service test_rclcpp
