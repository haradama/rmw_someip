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
// The loaned-message pool.
//
// Failing to clear a message on return leaks the previous value into the next
// loan. That only shows with a type holding strings or sequences, so those are
// what these tests use.

#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

#include "detail/message_pool.hpp"
#include "detail/type_support.hpp"

namespace
{
template<typename T>
rmw_someip_cpp::TypeSupportInfo resolve()
{
  rmw_someip_cpp::TypeSupportInfo info;
  std::string error;
  EXPECT_TRUE(
    rmw_someip_cpp::resolve_type_support(
      rosidl_typesupport_cpp::get_message_type_support_handle<T>(), info, error)) << error;
  return info;
}
}  // namespace

TEST(MessagePool, is_unavailable_until_configured)
{
  rmw_someip_cpp::MessagePool pool;
  EXPECT_FALSE(pool.available());
  EXPECT_EQ(nullptr, pool.borrow());
}

TEST(MessagePool, borrow_returns_an_initialised_message)
{
  rmw_someip_cpp::MessagePool pool;
  pool.configure(resolve<test_msgs::msg::BasicTypes>());
  ASSERT_TRUE(pool.available());

  auto * message = static_cast<test_msgs::msg::BasicTypes *>(pool.borrow());
  ASSERT_NE(nullptr, message);
  // init_function has run, so the fields hold their defaults.
  EXPECT_FALSE(message->bool_value);
  EXPECT_EQ(0, message->int32_value);
  EXPECT_TRUE(pool.give_back(message));
}

TEST(MessagePool, give_back_rejects_a_pointer_it_did_not_lend)
{
  rmw_someip_cpp::MessagePool pool;
  pool.configure(resolve<test_msgs::msg::BasicTypes>());

  test_msgs::msg::BasicTypes on_the_stack;
  // Letting this through would try to free the caller's stack variable.
  EXPECT_FALSE(pool.give_back(&on_the_stack));
  EXPECT_FALSE(pool.give_back(nullptr));

  auto * borrowed = pool.borrow();
  EXPECT_TRUE(pool.give_back(borrowed));
  // Returning twice is invalid.
  EXPECT_FALSE(pool.give_back(borrowed));
}

/// Reuse must not leak the previous contents.
/// give_back has to fini and re-init.
TEST(MessagePool, reused_messages_do_not_carry_old_contents)
{
  rmw_someip_cpp::MessagePool pool;
  pool.configure(resolve<test_msgs::msg::UnboundedSequences>());

  auto * first = static_cast<test_msgs::msg::UnboundedSequences *>(pool.borrow());
  ASSERT_NE(nullptr, first);
  first->string_values = {"stale", "data"};
  first->int32_values = {1, 2, 3};
  ASSERT_TRUE(pool.give_back(first));

  auto * second = static_cast<test_msgs::msg::UnboundedSequences *>(pool.borrow());
  ASSERT_NE(nullptr, second);
  EXPECT_TRUE(second->string_values.empty()) << "the previous value survived";
  EXPECT_TRUE(second->int32_values.empty()) << "the previous value survived";
  EXPECT_TRUE(pool.give_back(second));
}

TEST(MessagePool, lends_several_messages_at_once)
{
  rmw_someip_cpp::MessagePool pool;
  pool.configure(resolve<test_msgs::msg::BasicTypes>());

  std::set<void *> borrowed;
  for (int i = 0; i < 32; ++i) {
    void * message = pool.borrow();
    ASSERT_NE(nullptr, message);
    EXPECT_TRUE(borrowed.insert(message).second) << "lent the same message twice";
  }
  for (void * message : borrowed) {
    EXPECT_TRUE(pool.give_back(message));
  }
}

/// Idle messages are capped: anything past the cap is freed, so a burst of
/// loans is not held onto forever.
TEST(MessagePool, does_not_hoard_more_than_the_idle_cap)
{
  rmw_someip_cpp::MessagePool pool;
  pool.configure(resolve<test_msgs::msg::BasicTypes>());

  std::vector<void *> borrowed;
  for (int i = 0; i < 32; ++i) {
    borrowed.push_back(pool.borrow());
  }
  for (void * message : borrowed) {
    EXPECT_TRUE(pool.give_back(message));
  }
  // The surplus was destroyed; borrowing again still works.
  void * again = pool.borrow();
  EXPECT_NE(nullptr, again);
  EXPECT_TRUE(pool.give_back(again));
}

/// Destroying the pool with loans outstanding must not leak: a caller that
/// forgot to return one is cleaned up when the publisher or subscription goes.
TEST(MessagePool, destructor_frees_outstanding_loans)
{
  {
    rmw_someip_cpp::MessagePool pool;
    pool.configure(resolve<test_msgs::msg::UnboundedSequences>());
    auto * message = static_cast<test_msgs::msg::UnboundedSequences *>(pool.borrow());
    ASSERT_NE(nullptr, message);
    message->string_values = {"not returned"};
    // Deliberately not returned.
  }
  SUCCEED();  // the point is that ASan and valgrind report no leak
}
