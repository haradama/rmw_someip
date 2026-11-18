#!/usr/bin/env bash
# Run a CI script inside the container, locally.
#
#   ./tools/ci/in-docker.sh lint
#   ./tools/ci/in-docker.sh unit
#   ./tools/ci/in-docker.sh conformance
#   ./tools/ci/in-docker.sh coverage
#
# GitHub Actions runs the same scripts in the same image.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${RMW_SOMEIP_IMAGE:-rmw_someip/test:jazzy}"

[ $# -ge 1 ] || { echo "usage: $0 <lint|unit|conformance|coverage>" >&2; exit 1; }
SCRIPT="$1"; shift

# BENCH_* configures the benchmark, so pass it through.
env_args=()
while IFS='=' read -r name _; do
  env_args+=(-e "$name")
done < <(env | grep '^BENCH_' || true)

docker run --rm --cap-add NET_ADMIN \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp/rosuser \
  "${env_args[@]}" \
  -v "$REPO_ROOT":/ws \
  "$IMAGE" \
  bash -lc "/ws/tools/ci/${SCRIPT}.sh $*"
