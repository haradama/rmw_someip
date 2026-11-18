#!/usr/bin/env bash
# Compare one-way latency against other RMW implementations.
#
# Publisher and subscriber run as separate processes and share the host's
# steady_clock, so the difference is the latency. No means are reported: the
# differences between implementations show in the tail, so this reports
# p50/p90/p99, the maximum and the loss rate.
#
#   BENCH_RMWS="rmw_someip_cpp rmw_fastrtps_cpp"  what to compare
#   BENCH_SIZES="64 1024 16384"                   payload sizes in bytes
#   BENCH_COUNT=2000 BENCH_RATE_HZ=500            samples and rate per case
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

log "building the benchmark"
colcon build --packages-select rmw_someip_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF > /dev/null
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
colcon build --packages-select rmw_someip_benchmark --base-paths test > /dev/null
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

RMWS="${BENCH_RMWS:-rmw_someip_cpp rmw_fastrtps_cpp}"
SIZES="${BENCH_SIZES:-64 1024 16384}"
export BENCH_COUNT="${BENCH_COUNT:-2000}"
export BENCH_RATE_HZ="${BENCH_RATE_HZ:-500}"
export BENCH_WARMUP="${BENCH_WARMUP:-200}"
export RMW_SOMEIP_LOG_LEVEL=error

results="$(mktemp)"
trap 'rm -f "$results"' EXIT

run_one() {
  local rmw="$1" size="$2" reliability="$3"
  export RMW_IMPLEMENTATION="$rmw"
  export BENCH_PAYLOAD_BYTES="$size"
  export BENCH_LABEL="${rmw}|${size}|${reliability}"
  if [ "$reliability" = "best_effort" ]; then
    export BENCH_BEST_EFFORT=1
  else
    unset BENCH_BEST_EFFORT
  fi

  echo "--- $BENCH_LABEL" >&2
  # Start the subscriber first; the publisher waits for one to appear.
  ros2 run rmw_someip_benchmark bench_sub 2>/dev/null | grep '^RESULT' >> "$results" &
  local sub_pid=$!
  sleep 1
  ros2 run rmw_someip_benchmark bench_pub > /dev/null 2>&1 || true
  wait "$sub_pid" || true
  # Let the participants disappear before the next case's discovery.
  sleep 2
}

for rmw in $RMWS; do
  for size in $SIZES; do
    for reliability in reliable best_effort; do
      run_one "$rmw" "$size" "$reliability"
    done
  done
done

log "results (one-way latency, microseconds)"
printf '%-18s %7s %-12s %6s %8s %8s %8s %9s %7s %5s\n' \
  RMW bytes QoS n p50 p90 p99 max 'lost%' ooo
while IFS=$'\t' read -r _ label n p50 p90 p99 max lost ooo; do
  IFS='|' read -r rmw size reliability <<< "$label"
  printf '%-18s %7s %-12s %6s %8s %8s %8s %9s %7s %5s\n' \
    "$rmw" "$size" "$reliability" "$n" "$p50" "$p90" "$p99" "$max" "$lost" "$ooo"
done < "$results"

# No results means failure; never report zeros for a run that never connected.
grep -q '^RESULT' "$results" || { echo "no results collected" >&2; exit 1; }
