#!/usr/bin/env bash
# ament_lint: copyright, cppcheck, cpplint, lint_cmake, uncrustify, xmllint on
# rmw_someip_cpp, plus the Python linters on the Python packages.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

log "building rmw_someip_cpp with BUILD_TESTING=ON"
colcon build --packages-select rmw_someip_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

log "running the linters"
# colcon test does not always exit non-zero on an individual failure, so the
# verdict comes from test-result.
colcon test --packages-select rmw_someip_cpp --return-code-on-test-failure || true
colcon test-result --all --verbose

# The Python packages are not ament_cmake, so their linters run directly.
log "linting the Python packages"
ament_flake8 demo/py_app
ament_pep257 demo/py_app
ament_copyright demo/py_app
