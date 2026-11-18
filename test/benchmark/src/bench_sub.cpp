// Copyright 2026 rmw_someip contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Measures one-way latency and loss.
//
// A separate process on the same host as the publisher, so steady_clock is
// shared and the difference from the send time in the message is the latency.
// Do not look only at the mean; the differences show in the tail.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw_someip_benchmark/msg/payload.hpp"

namespace
{

uint64_t now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

size_t env_size(const char * name, size_t fallback)
{
  const char * v = std::getenv(name);
  return (v != nullptr && *v != '\0') ? std::strtoul(v, nullptr, 10) : fallback;
}

double percentile(std::vector<double> & sorted, double p)
{
  if (sorted.empty()) {
    return 0.0;
  }
  const size_t index = static_cast<size_t>(p * (sorted.size() - 1));
  return sorted[index];
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const size_t count = env_size("BENCH_COUNT", 2000);
  // The path is still cold at first — TCP congestion window, page faults,
  // branch prediction — and keeping those raises even the p50.
  const size_t warmup = env_size("BENCH_WARMUP", 200);
  const bool best_effort = std::getenv("BENCH_BEST_EFFORT") != nullptr;
  const std::string label = std::getenv("BENCH_LABEL") ? std::getenv("BENCH_LABEL") : "bench";

  auto node = rclcpp::Node::make_shared("bench_sub");

  rclcpp::QoS qos(rclcpp::KeepLast(100));
  if (best_effort) {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  std::vector<double> latencies_us;
  latencies_us.reserve(count);
  uint64_t received = 0;
  uint64_t last_sequence = 0;
  bool have_last = false;
  uint64_t out_of_order = 0;

  auto subscription = node->create_subscription<rmw_someip_benchmark::msg::Payload>(
    "/bench", qos,
    [&](const rmw_someip_benchmark::msg::Payload & message) {
      ++received;
      if (have_last && message.sequence <= last_sequence) {
        ++out_of_order;
      }
      last_sequence = message.sequence;
      have_last = true;
      if (message.sequence >= warmup) {
        latencies_us.push_back(
          static_cast<double>(now_ns() - message.stamp_ns) / 1000.0);
      }
      if (message.sequence + 1 >= count) {
        rclcpp::shutdown();
      }
    });

  // In case the publisher ends without shutdown arriving, having lost everything.
  auto guard = node->create_wall_timer(
    std::chrono::seconds(60), []() {rclcpp::shutdown();});

  rclcpp::spin(node);

  std::sort(latencies_us.begin(), latencies_us.end());
  const double lost_pct = (count == 0) ? 0.0 :
    100.0 * static_cast<double>(count - std::min<uint64_t>(received, count)) /
    static_cast<double>(count);

  // One TSV record per line; the driver script assembles the table.
  std::printf(
    "RESULT\t%s\t%zu\t%.1f\t%.1f\t%.1f\t%.1f\t%.2f\t%llu\n",
    label.c_str(), latencies_us.size(),
    percentile(latencies_us, 0.50), percentile(latencies_us, 0.90),
    percentile(latencies_us, 0.99),
    latencies_us.empty() ? 0.0 : latencies_us.back(),
    lost_pct, static_cast<unsigned long long>(out_of_order));
  std::fflush(stdout);
  return 0;
}
