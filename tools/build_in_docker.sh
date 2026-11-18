#!/usr/bin/env bash
# CMake build inside the container. --user passes the host UID so the outputs
# are not owned by root.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${RMW_SOMEIP_IMAGE:-rmw_someip/base:jazzy}"

[ $# -ge 1 ] || { echo "usage: $0 <source-dir-relative-to-repo-root> [...]" >&2; exit 1; }

for src in "$@"; do
  echo "=== building $src ==="
  docker run --rm \
    --user "$(id -u):$(id -g)" \
    -v "$REPO_ROOT":/ws \
    "$IMAGE" \
    bash -c "cmake -S /ws/$src -B /ws/$src/build -DCMAKE_BUILD_TYPE=Release && cmake --build /ws/$src/build -j\$(nproc)"
done
