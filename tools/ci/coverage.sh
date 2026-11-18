#!/usr/bin/env bash
# Line coverage for rmw_someip_cpp.
#
# The official suites dlopen librmw_someip_cpp.so, so the library itself has
# to be rebuilt instrumented; --coverage on the test binaries alone counts
# none of its lines.
#
# Test outcomes do not matter here — a failing test still covers the lines it
# reached. Passing or failing is lint.sh, unit.sh and conformance.sh's job.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

log "building rmw_someip_cpp with coverage"
find build/rmw_someip_cpp -name "*.gcda" -delete 2>/dev/null || true
colcon build --packages-select rmw_someip_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DRMW_SOMEIP_COVERAGE=ON -DBUILD_TESTING=OFF \
  > /dev/null
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_someip_cpp RMW_SOMEIP_LOG_LEVEL=error

# The unit tests compile the sources into their own binary, so instrument that too.
log "building and running the unit tests"
colcon build --packages-select rmw_someip_unit_tests --base-paths test \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DRMW_SOMEIP_COVERAGE=ON > /dev/null
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
"$WS/build/rmw_someip_unit_tests/unit_tests" > /dev/null 2>&1 || true
"$WS/build/rmw_someip_unit_tests/rmw_api_tests" > /dev/null 2>&1 || true

log "running the official suites"
"$(dirname "${BASH_SOURCE[0]}")/prepare-test-deps.sh" > /dev/null
cd "$WS/.test_deps"
# shellcheck disable=SC1091
source install/setup.bash
ctest --test-dir build/test_rmw_implementation -R someip_cpp --timeout 200 >/dev/null 2>&1 || true
ctest --test-dir build/test_quality_of_service -R someip --timeout 200 >/dev/null 2>&1 || true
ctest --test-dir build/test_communication -R rmw_someip_cpp -E rmw_fastrtps_cpp \
  --timeout 150 >/dev/null 2>&1 || true
ctest --test-dir build/test_rclcpp -R rmw_someip_cpp --timeout 150 >/dev/null 2>&1 || true

log "running the self-made tests"
cd "$WS"
colcon build --packages-select loaned_messages --base-paths test > /dev/null 2>&1 || true
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
python3 test/qos_events/local_transient.py >/dev/null 2>&1 || true
(python3 test/qos_events/publisher_side.py >/dev/null 2>&1 &
 sleep 1.5; python3 test/qos_events/subscriber_side.py >/dev/null 2>&1; wait) || true
ros2 run loaned_messages loaned_messages >/dev/null 2>&1 || true

log "gcovr"
mkdir -p "$WS/build/coverage"
gcovr --root "$WS" --filter "rmw_someip_cpp/src/" \
  --gcov-ignore-parse-errors negative_hits.warn_once_per_file \
  --txt "$WS/build/coverage.txt" --csv "$WS/build/coverage.csv" \
  --html-details "$WS/build/coverage/index.html" --print-summary 2>/dev/null | tail -4
