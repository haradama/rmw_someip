#!/usr/bin/env bash
# Line coverage; runs tools/ci/coverage.sh inside the container.
# Reports: build/coverage.txt, build/coverage.csv, build/coverage/index.html
set -euo pipefail
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ci/in-docker.sh" coverage "$@"
