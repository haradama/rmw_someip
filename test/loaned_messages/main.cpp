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
// End-to-end check of the loaned-message API.
//
// The official loaned tests only cover argument checking and the empty-queue
// case; this actually sends data both ways.
//
// Using a type with strings and variable-length sequences is the point. Loans
// are reused from a pool, so failing to clear one on return leaks the previous
// value into the next — which a fixed-size-only type would never reveal.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

using namespace std::chrono_literals;
using test_msgs::msg::UnboundedSequences;

namespace
{

/// Contents of the ith message; both length and value differ each time.
UnboundedSequences make_message(size_t i)
{
  UnboundedSequences message;
  message.string_values.push_back("loan-" + std::to_string(i));
  for (size_t k = 0; k <= i; ++k) {
    message.int32_values.push_back(static_cast<int32_t>(i * 100 + k));
  }
  return message;
}

bool matches(const UnboundedSequences & message, size_t i)
{
  const auto expected = make_message(i);
  return message.string_values == expected.string_values &&
         message.int32_values == expected.int32_values;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  constexpr size_t kCount = 5;

  auto node = rclcpp::Node::make_shared("loaned_messages");
  auto publisher = node->create_publisher<UnboundedSequences>("/loaned", 10);

  printf("can_loan_messages: publisher=%s\n", publisher->can_loan_messages() ? "true" : "false");

  std::vector<UnboundedSequences> received;
  auto subscription = node->create_subscription<UnboundedSequences>(
    "/loaned", 10,
    [&received](UnboundedSequences::SharedPtr message) {received.push_back(*message);});

  // Wait until matched.
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (publisher->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  // Borrow, fill, publish. rclcpp's LoanedMessage borrows from the middleware
  // through rmw_borrow_loaned_message() when can_loan_messages() is true.
  for (size_t i = 0; i < kCount; ++i) {
    auto loaned = publisher->borrow_loaned_message();
    if (!loaned.is_valid()) {
      fprintf(stderr, "borrow_loaned_message() failed\n");
      rclcpp::shutdown();
      return 1;
    }
    loaned.get() = make_message(i);
    publisher->publish(std::move(loaned));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  const auto until = std::chrono::steady_clock::now() + 5s;
  while (received.size() < kCount && std::chrono::steady_clock::now() < until) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(20ms);
  }

  bool ok = received.size() == kCount;
  for (size_t i = 0; i < received.size(); ++i) {
    if (!matches(received[i], i)) {
      fprintf(stderr, "message %zu does not match what was sent\n", i);
      ok = false;
    }
  }
  printf(
    "received %zu of %zu, contents match=%s\n",
    received.size(), kCount, ok ? "yes" : "no");
  printf("=== LOANED RESULT: %s ===\n", ok ? "SUCCESS" : "FAILURE");

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
