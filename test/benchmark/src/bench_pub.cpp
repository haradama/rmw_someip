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
// Publishes at a fixed rate. The subscriber does the measuring.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

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

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const size_t payload_bytes = env_size("BENCH_PAYLOAD_BYTES", 64);
  const size_t rate_hz = env_size("BENCH_RATE_HZ", 500);
  const size_t count = env_size("BENCH_COUNT", 2000);
  const bool best_effort = std::getenv("BENCH_BEST_EFFORT") != nullptr;

  auto node = rclcpp::Node::make_shared("bench_pub");

  // Not depth 1: with KEEP_LAST(1) a momentary stall on the subscriber side
  // drops samples, and it stops being clear whether latency or congestion is
  // being measured.
  rclcpp::QoS qos(rclcpp::KeepLast(100));
  if (best_effort) {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  auto publisher = node->create_publisher<rmw_someip_benchmark::msg::Payload>("/bench", qos);

  // Wait for a subscriber, so discovery time does not enter the measurement.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (publisher->get_subscription_count() == 0) {
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(node->get_logger(), "no subscriber appeared within 20s");
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  // Give the subscriber's receive path time to come up.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  rmw_someip_benchmark::msg::Payload message;
  message.data.assign(payload_bytes, 0xAB);

  const auto interval = std::chrono::nanoseconds(1'000'000'000ULL / rate_hz);
  auto next = std::chrono::steady_clock::now();
  for (size_t i = 0; i < count && rclcpp::ok(); ++i) {
    next += interval;
    std::this_thread::sleep_until(next);
    message.sequence = i;
    message.stamp_ns = now_ns();
    publisher->publish(message);
  }

  // Let the last samples arrive before exiting.
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
  return 0;
}
