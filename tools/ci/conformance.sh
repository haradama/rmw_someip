#!/usr/bin/env bash
# The four official conformance suites plus the QoS and loaned-message tests.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

log "building rmw_someip_cpp"
colcon build --packages-select rmw_someip_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

log "building the self-made tests"
colcon build --packages-select loaned_messages --base-paths test
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_someip_cpp RMW_SOMEIP_LOG_LEVEL=error

"$(dirname "${BASH_SOURCE[0]}")/prepare-test-deps.sh"
cd "$WS/.test_deps"
# shellcheck disable=SC1091
source install/setup.bash

failed=0
run() {
  local label="$1"; shift
  log "$label"
  if ! "$@"; then
    echo "FAILED: $label" >&2
    failed=1
  fi
}

# test_communication and test_rclcpp fail rarely, on a shutdown race inside
# rclpy. --repeat until-pass:2 retries once; a real
# regression fails both attempts.
run "test_rmw_implementation" \
  ctest --test-dir build/test_rmw_implementation -R someip_cpp \
  --timeout 200 --output-on-failure
run "test_quality_of_service" \
  ctest --test-dir build/test_quality_of_service -R someip \
  --timeout 200 --output-on-failure
run "test_communication" \
  ctest --test-dir build/test_communication -R rmw_someip_cpp -E rmw_fastrtps_cpp \
  --timeout 150 --output-on-failure --repeat until-pass:2
run "test_rclcpp" \
  ctest --test-dir build/test_rclcpp -R rmw_someip_cpp \
  --timeout 150 --output-on-failure --repeat until-pass:2

cd "$WS"
run "qos_events (local transient)" python3 test/qos_events/local_transient.py
log "qos_events (publisher / subscriber)"
if ! (python3 test/qos_events/publisher_side.py &
      sleep 1.5
      python3 test/qos_events/subscriber_side.py
      wait); then
  echo "FAILED: qos_events" >&2
  failed=1
fi
run "loaned_messages" ros2 run loaned_messages loaned_messages

exit "$failed"
