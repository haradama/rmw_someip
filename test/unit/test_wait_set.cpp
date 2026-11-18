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
// Wait sets and guard conditions.
//
// vsomeip callback threads run alongside the executor thread, so these are
// checked under concurrency.

#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "detail/guard_condition.hpp"
#include "detail/wait_set.hpp"

using namespace std::chrono_literals;

TEST(WaitSet, wait_returns_immediately_when_already_triggered)
{
  rmw_someip_cpp::WaitSetImpl wait_set;
  wait_set.trigger();
  const auto start = std::chrono::steady_clock::now();
  wait_set.wait(std::chrono::nanoseconds(-1));  // returns at once even so
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

TEST(WaitSet, wait_times_out_without_a_trigger)
{
  rmw_someip_cpp::WaitSetImpl wait_set;
  const auto start = std::chrono::steady_clock::now();
  wait_set.wait(50ms);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 40ms);
}

TEST(WaitSet, zero_timeout_polls)
{
  rmw_someip_cpp::WaitSetImpl wait_set;
  const auto start = std::chrono::steady_clock::now();
  wait_set.wait(std::chrono::nanoseconds::zero());
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

TEST(WaitSet, another_thread_can_wake_the_waiter)
{
  rmw_someip_cpp::WaitSetImpl wait_set;
  std::thread waker(
    [&wait_set]() {
      std::this_thread::sleep_for(20ms);
      wait_set.trigger();
    });
  wait_set.wait(std::chrono::nanoseconds(-1));
  waker.join();
  SUCCEED();
}

/// One wait consumes the trigger; it is an edge, not a level.
TEST(WaitSet, the_trigger_is_consumed_by_one_wait)
{
  rmw_someip_cpp::WaitSetImpl wait_set;
  wait_set.trigger();
  wait_set.wait(std::chrono::nanoseconds(-1));

  const auto start = std::chrono::steady_clock::now();
  wait_set.wait(30ms);  // the second one should time out
  EXPECT_GE(std::chrono::steady_clock::now() - start, 20ms);
}

TEST(GuardCondition, take_reports_and_clears_the_trigger)
{
  rmw_someip_cpp::GuardCondition guard;
  EXPECT_FALSE(guard.take());
  EXPECT_FALSE(guard.has_data());

  guard.trigger();
  EXPECT_TRUE(guard.has_data());
  EXPECT_TRUE(guard.take());
  EXPECT_FALSE(guard.take());
  EXPECT_FALSE(guard.has_data());
}

/// Attaching something already triggered wakes the wait set immediately.
/// Without that, anything that arrived just before the attach is lost.
TEST(GuardCondition, attaching_an_already_triggered_condition_wakes_the_wait_set)
{
  rmw_someip_cpp::WaitSetImpl wait_set;
  rmw_someip_cpp::GuardCondition guard;
  guard.trigger();
  guard.attach(&wait_set);

  const auto start = std::chrono::steady_clock::now();
  wait_set.wait(std::chrono::nanoseconds(-1));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  guard.detach();
}

TEST(GuardCondition, trigger_after_attach_wakes_the_wait_set)
{
  rmw_someip_cpp::WaitSetImpl wait_set;
  rmw_someip_cpp::GuardCondition guard;
  guard.attach(&wait_set);

  std::thread waker(
    [&guard]() {
      std::this_thread::sleep_for(20ms);
      guard.trigger();
    });
  wait_set.wait(std::chrono::nanoseconds(-1));
  waker.join();
  EXPECT_TRUE(guard.take());
  guard.detach();
}

/// A trigger after detach must not reach the wait set, which may be gone.
TEST(GuardCondition, trigger_after_detach_does_not_touch_the_wait_set)
{
  rmw_someip_cpp::GuardCondition guard;
  {
    rmw_someip_cpp::WaitSetImpl wait_set;
    guard.attach(&wait_set);
    guard.detach();
  }
  guard.trigger();  // wait_set is gone; this must not crash
  EXPECT_TRUE(guard.take());
}

TEST(GuardCondition, attaching_a_null_wait_set_is_allowed)
{
  rmw_someip_cpp::GuardCondition guard;
  guard.trigger();
  guard.attach(nullptr);
  EXPECT_TRUE(guard.take());
}
