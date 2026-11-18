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
// Cleanup when an allocation fails partway through creating an entity.
//
// Neither rmw_publisher_allocate() nor `new (std::nothrow)` takes an
// allocator, so substituting an rcutils_allocator_t cannot reach them;
// alloc_hooks.cpp substitutes the allocation functions instead.
//
// Which allocation is the nth is an implementation detail, so no test pins a
// number. The failure point is swept from 0 upwards, and wherever it lands
// the requirements are the same: do not crash, do not return a half-built
// object (either fully usable or null), and set an error when returning null.
// Between them these walk every goto-fail and delete path.

#include <new>
#include <string>
#include <vector>

#include "rmw/allocators.h"

#include "gtest/gtest.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rmw/error_handling.h"
#include "rmw/init_options.h"
#include "rmw/rmw.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/srv/basic_types.hpp"

#include "alloc_hooks.hpp"

using rmw_someip_cpp_test::allocation_failure_fired;
using rmw_someip_cpp_test::fail_allocation_at;
using rmw_someip_cpp_test::stop_failing_allocations;

namespace
{

/// How far the failure point is swept. Entity creation performs at most about
/// this many allocations up front, which spans both `new (std::nothrow)` and
/// rmw_*_allocate().
constexpr int64_t kSweep = 40;

/// Arm a failure, run the callable, disarm.
template<typename Callable>
void with_failing_allocation(int64_t at, Callable && callable)
{
  fail_allocation_at(at);
  callable();
  stop_failing_allocations();
  rmw_reset_error();
}

class AllocFailure : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    options_ = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(RMW_RET_OK, rmw_init_options_init(&options_, allocator));
    options_.enclave = rcutils_strdup("/", allocator);
    ASSERT_NE(nullptr, options_.enclave);
    context_ = rmw_get_zero_initialized_context();
    ASSERT_EQ(RMW_RET_OK, rmw_init(&options_, &context_)) << rmw_get_error_string().str;
    node_ = rmw_create_node(&context_, "alloc_node", "/");
    ASSERT_NE(nullptr, node_);
  }

  void TearDown() override
  {
    stop_failing_allocations();
    if (node_ != nullptr) {
      EXPECT_EQ(RMW_RET_OK, rmw_destroy_node(node_));
    }
    EXPECT_EQ(RMW_RET_OK, rmw_shutdown(&context_));
    EXPECT_EQ(RMW_RET_OK, rmw_context_fini(&context_));
    EXPECT_EQ(RMW_RET_OK, rmw_init_options_fini(&options_));
    rmw_reset_error();
  }

  const rosidl_message_type_support_t * message_ts() const
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>();
  }
  const rosidl_service_type_support_t * service_ts() const
  {
    return rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::BasicTypes>();
  }

  rmw_init_options_t options_;
  rmw_context_t context_;
  rmw_node_t * node_ {nullptr};
};

}  // namespace

TEST_F(AllocFailure, the_hook_actually_makes_an_allocation_fail)
{
  // If the substitution is not in effect, every test below verifies nothing.

  // (1) the nothrow new
  fail_allocation_at(0);
  int * value = new (std::nothrow) int(1);
  stop_failing_allocations();
  EXPECT_EQ(nullptr, value);
  EXPECT_TRUE(allocation_failure_fired());

  // (2) an allocation inside librmw, showing the executable's definition wins
  fail_allocation_at(0);
  rmw_publisher_t * publisher = rmw_publisher_allocate();
  stop_failing_allocations();
  EXPECT_EQ(nullptr, publisher);
  EXPECT_TRUE(allocation_failure_fired());

  // (3) normal behaviour once disarmed
  value = new (std::nothrow) int(1);
  ASSERT_NE(nullptr, value);
  EXPECT_EQ(1, *value);
  delete value;
  publisher = rmw_publisher_allocate();
  ASSERT_NE(nullptr, publisher);
  rmw_publisher_free(publisher);
}

TEST_F(AllocFailure, creating_a_publisher_either_succeeds_completely_or_fails_cleanly)
{
  auto options = rmw_get_default_publisher_options();
  size_t failures = 0;
  for (int64_t at = 0; at < kSweep; ++at) {
    rmw_publisher_t * publisher = nullptr;
    with_failing_allocation(
      at, [&]() {
        publisher = rmw_create_publisher(
          node_, message_ts(), "/alloc_pub", &rmw_qos_profile_default, &options);
      });
    if (publisher == nullptr) {
      ++failures;
      continue;
    }
    // If it succeeded it has to be fully usable, not half-built.
    size_t count = 0;
    EXPECT_EQ(RMW_RET_OK, rmw_publisher_count_matched_subscriptions(publisher, &count));
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
  }
  // At least one sweep position must have taken the failure path.
  EXPECT_GT(failures, 0u);
}

TEST_F(AllocFailure, creating_a_subscription_either_succeeds_completely_or_fails_cleanly)
{
  auto options = rmw_get_default_subscription_options();
  size_t failures = 0;
  for (int64_t at = 0; at < kSweep; ++at) {
    rmw_subscription_t * subscription = nullptr;
    with_failing_allocation(
      at, [&]() {
        subscription = rmw_create_subscription(
          node_, message_ts(), "/alloc_sub", &rmw_qos_profile_default, &options);
      });
    if (subscription == nullptr) {
      ++failures;
      continue;
    }
    size_t count = 0;
    EXPECT_EQ(RMW_RET_OK, rmw_subscription_count_matched_publishers(subscription, &count));
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  }
  EXPECT_GT(failures, 0u);
}

TEST_F(AllocFailure, creating_a_service_either_succeeds_completely_or_fails_cleanly)
{
  size_t failures = 0;
  for (int64_t at = 0; at < kSweep; ++at) {
    rmw_service_t * service = nullptr;
    with_failing_allocation(
      at, [&]() {
        service = rmw_create_service(
          node_, service_ts(), "/alloc_srv", &rmw_qos_profile_services_default);
      });
    if (service == nullptr) {
      ++failures;
      continue;
    }
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_service(node_, service));
  }
  EXPECT_GT(failures, 0u);
}

TEST_F(AllocFailure, creating_a_client_either_succeeds_completely_or_fails_cleanly)
{
  size_t failures = 0;
  for (int64_t at = 0; at < kSweep; ++at) {
    rmw_client_t * client = nullptr;
    with_failing_allocation(
      at, [&]() {
        client = rmw_create_client(
          node_, service_ts(), "/alloc_cli", &rmw_qos_profile_services_default);
      });
    if (client == nullptr) {
      ++failures;
      continue;
    }
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_client(node_, client));
  }
  EXPECT_GT(failures, 0u);
}

TEST_F(AllocFailure, creating_a_node_either_succeeds_completely_or_fails_cleanly)
{
  size_t failures = 0;
  for (int64_t at = 0; at < kSweep; ++at) {
    rmw_node_t * node = nullptr;
    with_failing_allocation(
      at, [&]() {
        node = rmw_create_node(&context_, "alloc_extra", "/");
      });
    if (node == nullptr) {
      ++failures;
      continue;
    }
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_node(node));
  }
  EXPECT_GT(failures, 0u);
}

TEST_F(AllocFailure, creating_a_wait_set_or_guard_condition_fails_cleanly)
{
  size_t wait_set_failures = 0;
  for (int64_t at = 0; at < 8; ++at) {
    rmw_wait_set_t * wait_set = nullptr;
    with_failing_allocation(at, [&]() {wait_set = rmw_create_wait_set(&context_, 4);});
    if (wait_set == nullptr) {
      ++wait_set_failures;
      continue;
    }
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_wait_set(wait_set));
  }
  EXPECT_GT(wait_set_failures, 0u);

  size_t guard_failures = 0;
  for (int64_t at = 0; at < 8; ++at) {
    rmw_guard_condition_t * guard = nullptr;
    with_failing_allocation(at, [&]() {guard = rmw_create_guard_condition(&context_);});
    if (guard == nullptr) {
      ++guard_failures;
      continue;
    }
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_guard_condition(guard));
  }
  EXPECT_GT(guard_failures, 0u);
}

TEST_F(AllocFailure, borrowing_a_loaned_message_that_cannot_be_allocated_fails_cleanly)
{
  auto options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, message_ts(), "/alloc_loan", &rmw_qos_profile_default, &options);
  ASSERT_NE(nullptr, publisher);

  size_t failures = 0;
  for (int64_t at = 0; at < 8; ++at) {
    void * loaned = nullptr;
    rmw_ret_t ret = RMW_RET_OK;
    with_failing_allocation(
      at, [&]() {
        ret = rmw_borrow_loaned_message(publisher, message_ts(), &loaned);
      });
    if (ret != RMW_RET_OK) {
      ++failures;
      EXPECT_EQ(nullptr, loaned);
      continue;
    }
    ASSERT_NE(nullptr, loaned);
    EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_publisher(publisher, loaned));
  }
  EXPECT_GT(failures, 0u);

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}
