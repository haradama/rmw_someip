#!/usr/bin/env bash
# Unit tests and the CDR byte-compatibility test. No SOME/IP traffic needed.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

log "building rmw_someip_cpp"
colcon build --packages-select rmw_someip_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

log "building the unit tests"
colcon build --packages-select rmw_someip_unit_tests --base-paths test \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_someip_cpp RMW_SOMEIP_LOG_LEVEL=error

log "unit_tests (internals of detail/, and malformed input)"
"$WS/build/rmw_someip_unit_tests/unit_tests"

log "rmw_api_tests (public rmw API and allocation failure)"
"$WS/build/rmw_someip_unit_tests/rmw_api_tests"

# CDR has to match Fast DDS byte for byte; without it neither rosbag2
# interoperability nor a DDS bridge works.
log "cdr_compat_test (byte compatibility with Fast DDS)"
cmake -S "$WS/test/cdr_compat_test" -B "$WS/test/cdr_compat_test/build" \
  -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "$WS/test/cdr_compat_test/build" -j"$(nproc)" > /dev/null
LD_LIBRARY_PATH="$WS/install/rmw_someip_cpp/lib:${LD_LIBRARY_PATH:-}" \
  "$WS/test/cdr_compat_test/build/cdr_compat_test"
