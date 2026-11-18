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
// The public rmw API, exercised directly, mostly on its failure paths.
//
// test_rmw_implementation covers the happy path and argument checking well but
// never reaches an allocation failure. rmw takes an rcutils_allocator_t by
// argument, so substituting a failing one provokes those from outside; without
// that, the goto-fail branches never run.

#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"

#include "rmw/error_handling.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/discovery_options.h"
#include "rmw/init_options.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/network_flow_endpoint_array.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/srv/basic_types.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace
{

/// An allocator that fails from its nth allocation onward.
///
/// 0 fails immediately. Varying n reaches the cleanup paths that only run when
/// the failure happens partway through, such as rmw_names_and_types_fini().
struct FailingAllocator
{
  static size_t remaining;

  static void * allocate(size_t size, void * state)
  {
    if (remaining == 0) {
      return nullptr;
    }
    --remaining;
    return rcutils_get_default_allocator().allocate(size, state);
  }
  static void deallocate(void * pointer, void * state)
  {
    rcutils_get_default_allocator().deallocate(pointer, state);
  }
  static void * reallocate(void * pointer, size_t size, void * state)
  {
    if (remaining == 0) {
      return nullptr;
    }
    --remaining;
    return rcutils_get_default_allocator().reallocate(pointer, size, state);
  }
  static void * zero_allocate(size_t count, size_t size, void * state)
  {
    if (remaining == 0) {
      return nullptr;
    }
    --remaining;
    return rcutils_get_default_allocator().zero_allocate(count, size, state);
  }

  static rcutils_allocator_t make(size_t successes)
  {
    remaining = successes;
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    allocator.allocate = allocate;
    allocator.deallocate = deallocate;
    allocator.reallocate = reallocate;
    allocator.zero_allocate = zero_allocate;
    return allocator;
  }
};
size_t FailingAllocator::remaining = 0;

/// A fixture holding one context and one node.
class RmwApi : public ::testing::Test
{
protected:
  void SetUp() override
  {
    options_ = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(RMW_RET_OK, rmw_init_options_init(&options_, rcutils_get_default_allocator()));
    options_.domain_id = 42;
    // rcl normally fills in the enclave, and rmw_init() rejects a null one, so
    // a test calling rmw directly has to supply it.
    options_.enclave = rcutils_strdup("/", rcutils_get_default_allocator());
    ASSERT_NE(nullptr, options_.enclave);
    context_ = rmw_get_zero_initialized_context();
    ASSERT_EQ(RMW_RET_OK, rmw_init(&options_, &context_)) << rmw_get_error_string().str;
    node_ = rmw_create_node(&context_, "unit_node", "/");
    ASSERT_NE(nullptr, node_) << rmw_get_error_string().str;
  }

  void TearDown() override
  {
    if (node_ != nullptr) {
      EXPECT_EQ(RMW_RET_OK, rmw_destroy_node(node_));
    }
    EXPECT_EQ(RMW_RET_OK, rmw_shutdown(&context_));
    EXPECT_EQ(RMW_RET_OK, rmw_context_fini(&context_));
    EXPECT_EQ(RMW_RET_OK, rmw_init_options_fini(&options_));
    rmw_reset_error();
  }

  const rosidl_message_type_support_t * strings_ts() const
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>();
  }

  const rosidl_service_type_support_t * srv_ts() const
  {
    return rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::BasicTypes>();
  }

  rmw_init_options_t options_;
  rmw_context_t context_;
  rmw_node_t * node_ {nullptr};
};

/// A typesupport that provides no introspection.
///
/// An all-null handle trips an assert in get_message_typesupport_handle(), so
/// it carries a func that always fails to resolve instead.
const rosidl_message_type_support_t * always_unresolvable(
  const rosidl_message_type_support_t *, const char *)
{
  return nullptr;
}

const rosidl_message_type_support_t * unresolvable_type_support()
{
  static rosidl_message_type_support_t handle = {
    "not_a_real_typesupport", nullptr, always_unresolvable, nullptr};
  return &handle;
}

}  // namespace

// ---------------------------------------------------------------------------
// The API declared unsupported
//
// Never called, never run. Continuing to return UNSUPPORTED is part of the
// contract — rcl accepts it from these — so it is pinned down here.
// ---------------------------------------------------------------------------

TEST(RmwUnsupported, stubs_report_unsupported)
{
  EXPECT_EQ(RMW_RET_UNSUPPORTED, rmw_set_log_severity(RMW_LOG_SEVERITY_DEBUG));
  rmw_reset_error();

  EXPECT_EQ(
    RMW_RET_UNSUPPORTED,
    rmw_serialization_support_init("whatever", nullptr, nullptr));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_UNSUPPORTED, rmw_fini_publisher_allocation(nullptr));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_UNSUPPORTED, rmw_fini_subscription_allocation(nullptr));
  rmw_reset_error();

  EXPECT_FALSE(rmw_feature_supported(RMW_MIDDLEWARE_SUPPORTS_TYPE_DISCOVERY));
}

TEST(RmwUnsupported, identifiers_are_stable)
{
  EXPECT_STREQ("rmw_someip_cpp", rmw_get_implementation_identifier());
  // Returning "cdr" commits to byte compatibility with DDS implementations.
  EXPECT_STREQ("cdr", rmw_get_serialization_format());
}

// ---------------------------------------------------------------------------
// Allocation failure
// ---------------------------------------------------------------------------

/// Graph introspection allocates names and type names. Failing partway must
/// return BAD_ALLOC and free what was allocated so far.
TEST_F(RmwApi, graph_queries_survive_allocation_failure)
{
  // This node's own parameter services guarantee at least one entry.
  for (size_t budget = 0; budget < 6; ++budget) {
    auto allocator = FailingAllocator::make(budget);
    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    const rmw_ret_t ret =
      rmw_get_topic_names_and_types(node_, &allocator, false, &names_and_types);
    if (ret == RMW_RET_OK) {
      EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&names_and_types));
    } else {
      EXPECT_TRUE(ret == RMW_RET_BAD_ALLOC || ret == RMW_RET_ERROR) << "budget=" << budget;
    }
    rmw_reset_error();
  }
}

TEST_F(RmwApi, node_scoped_graph_queries_survive_allocation_failure)
{
  for (size_t budget = 0; budget < 6; ++budget) {
    auto allocator = FailingAllocator::make(budget);
    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    const rmw_ret_t ret = rmw_get_subscriber_names_and_types_by_node(
      node_, &allocator, "unit_node", "/", false, &names_and_types);
    if (ret == RMW_RET_OK) {
      EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&names_and_types));
    }
    rmw_reset_error();
  }
}

TEST_F(RmwApi, node_names_survives_allocation_failure)
{
  for (size_t budget = 0; budget < 6; ++budget) {
    auto allocator = FailingAllocator::make(budget);
    auto names = rcutils_get_zero_initialized_string_array();
    auto namespaces = rcutils_get_zero_initialized_string_array();
    names.allocator = allocator;
    namespaces.allocator = allocator;
    const rmw_ret_t ret = rmw_get_node_names(node_, &names, &namespaces);
    if (ret == RMW_RET_OK) {
      EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&names));
      EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&namespaces));
    }
    rmw_reset_error();
  }
}

TEST_F(RmwApi, endpoint_info_survives_allocation_failure)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/alloc_topic", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher) << rmw_get_error_string().str;

  for (size_t budget = 0; budget < 8; ++budget) {
    auto allocator = FailingAllocator::make(budget);
    auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
    const rmw_ret_t ret =
      rmw_get_publishers_info_by_topic(node_, &allocator, "/alloc_topic", false, &info);
    if (ret == RMW_RET_OK) {
      auto default_allocator = rcutils_get_default_allocator();
      (void)rmw_topic_endpoint_info_array_fini(&info, &default_allocator);
    }
    rmw_reset_error();
  }
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

// ---------------------------------------------------------------------------
// Serialization failure paths
// ---------------------------------------------------------------------------

TEST_F(RmwApi, serialize_reports_a_bad_type_support)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto serialized = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 0, &allocator));

  test_msgs::msg::Strings message;
  EXPECT_EQ(RMW_RET_ERROR, rmw_serialize(&message, unresolvable_type_support(), &serialized));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_ERROR, rmw_deserialize(&serialized, unresolvable_type_support(), &message));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
}

/// A buffer too small is grown before writing.
TEST_F(RmwApi, serialize_grows_a_small_buffer)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto serialized = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 1, &allocator));

  test_msgs::msg::Strings message;
  message.string_value = std::string(500, 'x');
  EXPECT_EQ(RMW_RET_OK, rmw_serialize(&message, strings_ts(), &serialized));
  EXPECT_GT(serialized.buffer_length, 500u);

  test_msgs::msg::Strings restored;
  EXPECT_EQ(RMW_RET_OK, rmw_deserialize(&serialized, strings_ts(), &restored));
  EXPECT_EQ(message.string_value, restored.string_value);

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
}

TEST_F(RmwApi, deserialize_reports_a_corrupt_payload)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto serialized = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 8, &allocator));
  // Make the encapsulation header say big endian.
  serialized.buffer[0] = 0x00;
  serialized.buffer[1] = 0x00;
  serialized.buffer[2] = 0x00;
  serialized.buffer[3] = 0x00;
  serialized.buffer_length = 8;

  test_msgs::msg::Strings message;
  EXPECT_EQ(RMW_RET_ERROR, rmw_deserialize(&serialized, strings_ts(), &message));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
}

/// An unbounded member makes the size undecidable; unsupported elsewhere too.
TEST(RmwSerialize, size_is_not_available)
{
  size_t size = 0;
  EXPECT_EQ(
    RMW_RET_UNSUPPORTED,
    rmw_get_serialized_message_size(nullptr, nullptr, &size));
  rmw_reset_error();
}

// ---------------------------------------------------------------------------
// Entity lifecycle
// ---------------------------------------------------------------------------

TEST_F(RmwApi, publisher_rejects_unknown_qos_policies)
{
  auto options = rmw_get_default_publisher_options();
  rmw_qos_profile_t qos = rmw_qos_profile_default;

  qos.history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
  EXPECT_EQ(nullptr, rmw_create_publisher(node_, strings_ts(), "/t", &qos, &options));
  rmw_reset_error();

  qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
  EXPECT_EQ(nullptr, rmw_create_publisher(node_, strings_ts(), "/t", &qos, &options));
  rmw_reset_error();

  qos = rmw_qos_profile_default;
  qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
  EXPECT_EQ(nullptr, rmw_create_publisher(node_, strings_ts(), "/t", &qos, &options));
  rmw_reset_error();
}

TEST_F(RmwApi, subscription_rejects_unknown_qos_policies)
{
  auto options = rmw_get_default_subscription_options();
  rmw_qos_profile_t qos = rmw_qos_profile_default;

  qos.history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
  EXPECT_EQ(nullptr, rmw_create_subscription(node_, strings_ts(), "/t", &qos, &options));
  rmw_reset_error();

  qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
  EXPECT_EQ(nullptr, rmw_create_subscription(node_, strings_ts(), "/t", &qos, &options));
  rmw_reset_error();
}

/// An unresolvable typesupport cannot create an entity.
TEST_F(RmwApi, entities_reject_an_unresolvable_type_support)
{
  auto publisher_options = rmw_get_default_publisher_options();
  EXPECT_EQ(
    nullptr,
    rmw_create_publisher(
      node_, unresolvable_type_support(), "/t", &rmw_qos_profile_default, &publisher_options));
  rmw_reset_error();

  auto subscription_options = rmw_get_default_subscription_options();
  EXPECT_EQ(
    nullptr,
    rmw_create_subscription(
      node_, unresolvable_type_support(), "/t", &rmw_qos_profile_default,
      &subscription_options));
  rmw_reset_error();
}

/// SOME/IP has no counterpart to content filters.
TEST_F(RmwApi, content_filters_are_unsupported)
{
  auto options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/cft", &rmw_qos_profile_default, &options);
  ASSERT_NE(nullptr, subscription) << rmw_get_error_string().str;

  EXPECT_FALSE(rmw_subscription_get_content_filter(subscription, nullptr, nullptr) == RMW_RET_OK);
  rmw_reset_error();
  EXPECT_NE(RMW_RET_OK, rmw_subscription_set_content_filter(subscription, nullptr));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
}

/// Network flow endpoints are unsupported; vsomeip does not expose them.
TEST_F(RmwApi, network_flow_endpoints_are_unsupported)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/nfe", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);

  auto allocator = rcutils_get_default_allocator();
  auto array = rmw_get_zero_initialized_network_flow_endpoint_array();
  EXPECT_EQ(
    RMW_RET_UNSUPPORTED,
    rmw_publisher_get_network_flow_endpoints(publisher, &allocator, &array));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

/// A BEST_EFFORT publisher uses UDP, a different path from RELIABLE over TCP,
/// so both are exercised.
TEST_F(RmwApi, best_effort_publisher_round_trips_locally)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher =
    rmw_create_publisher(node_, strings_ts(), "/be", &qos, &publisher_options);
  ASSERT_NE(nullptr, publisher) << rmw_get_error_string().str;

  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription =
    rmw_create_subscription(node_, strings_ts(), "/be", &qos, &subscription_options);
  ASSERT_NE(nullptr, subscription) << rmw_get_error_string().str;

  test_msgs::msg::Strings message;
  message.string_value = "best effort";
  EXPECT_EQ(RMW_RET_OK, rmw_publish(publisher, &message, nullptr));

  test_msgs::msg::Strings received;
  bool taken = false;
  rmw_message_info_t info = rmw_get_zero_initialized_message_info();
  EXPECT_EQ(RMW_RET_OK, rmw_take_with_info(subscription, &received, &taken, &info, nullptr));
  EXPECT_TRUE(taken);
  EXPECT_EQ("best effort", received.string_value);

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

/// Publishing and taking a pre-serialized message.
TEST_F(RmwApi, serialized_publish_and_take)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/ser", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/ser", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto serialized = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 0, &allocator));
  test_msgs::msg::Strings message;
  message.string_value = "serialized";
  ASSERT_EQ(RMW_RET_OK, rmw_serialize(&message, strings_ts(), &serialized));

  EXPECT_EQ(RMW_RET_OK, rmw_publish_serialized_message(publisher, &serialized, nullptr));

  auto received = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&received, 0, &allocator));
  bool taken = false;
  EXPECT_EQ(
    RMW_RET_OK, rmw_take_serialized_message(subscription, &received, &taken, nullptr));
  EXPECT_TRUE(taken);

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&received));
  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

/// A loaned message round trip, and rejection of a pointer never lent.
TEST_F(RmwApi, loaned_messages_round_trip)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/loan", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  ASSERT_TRUE(publisher->can_loan_messages);

  void * loaned = nullptr;
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_message(publisher, strings_ts(), &loaned));
  ASSERT_NE(nullptr, loaned);
  // Cannot lend into a non-empty pointer.
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_borrow_loaned_message(publisher, strings_ts(), &loaned));
  rmw_reset_error();

  static_cast<test_msgs::msg::Strings *>(loaned)->string_value = "loaned";
  EXPECT_EQ(RMW_RET_OK, rmw_publish_loaned_message(publisher, loaned, nullptr));

  // Returning a pointer that was never lent is an error.
  test_msgs::msg::Strings on_the_stack;
  EXPECT_EQ(RMW_RET_ERROR, rmw_return_loaned_message_from_publisher(publisher, &on_the_stack));
  rmw_reset_error();

  // Borrow and return without publishing.
  void * unused = nullptr;
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_message(publisher, strings_ts(), &unused));
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_publisher(publisher, unused));

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

TEST_F(RmwApi, dynamic_messages_are_unsupported)
{
  auto options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/dyn", &rmw_qos_profile_default, &options);
  ASSERT_NE(nullptr, subscription);

  bool taken = false;
  EXPECT_EQ(
    RMW_RET_UNSUPPORTED,
    rmw_take_dynamic_message(subscription, nullptr, &taken, nullptr));
  rmw_reset_error();
  EXPECT_EQ(
    RMW_RET_UNSUPPORTED,
    rmw_take_dynamic_message_with_info(subscription, nullptr, &taken, nullptr, nullptr));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
}

// ---------------------------------------------------------------------------
// Failing the name copy
//
// Every entity creation duplicates names with `context->options.allocator`,
// which the caller can substitute, so the cleanup after a failed copy is
// reachable from outside. The allocations rmw_*_allocate() performs internally
// are not.
// ---------------------------------------------------------------------------

TEST_F(RmwApi, entity_creation_cleans_up_when_the_name_copy_fails)
{
  auto default_allocator = context_.options.allocator;
  // Allow one allocation per creation, so the name copy is what fails.
  context_.options.allocator = FailingAllocator::make(0);

  auto publisher_options = rmw_get_default_publisher_options();
  EXPECT_EQ(
    nullptr,
    rmw_create_publisher(node_, strings_ts(), "/alloc", &rmw_qos_profile_default,
    &publisher_options));
  rmw_reset_error();

  FailingAllocator::remaining = 0;
  auto subscription_options = rmw_get_default_subscription_options();
  EXPECT_EQ(
    nullptr,
    rmw_create_subscription(node_, strings_ts(), "/alloc", &rmw_qos_profile_default,
    &subscription_options));
  rmw_reset_error();

  FailingAllocator::remaining = 0;
  EXPECT_EQ(
    nullptr,
    rmw_create_client(node_, srv_ts(), "/alloc_srv", &rmw_qos_profile_services_default));
  rmw_reset_error();

  FailingAllocator::remaining = 0;
  EXPECT_EQ(
    nullptr,
    rmw_create_service(node_, srv_ts(), "/alloc_srv", &rmw_qos_profile_services_default));
  rmw_reset_error();

  FailingAllocator::remaining = 0;
  EXPECT_EQ(nullptr, rmw_create_node(&context_, "alloc_node", "/"));
  rmw_reset_error();

  context_.options.allocator = default_allocator;
}

// ---------------------------------------------------------------------------
// The rest of graph introspection
// ---------------------------------------------------------------------------

TEST_F(RmwApi, node_names_with_enclaves)
{
  auto names = rcutils_get_zero_initialized_string_array();
  auto namespaces = rcutils_get_zero_initialized_string_array();
  auto enclaves = rcutils_get_zero_initialized_string_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_node_names_with_enclaves(node_, &names, &namespaces, &enclaves))
    << rmw_get_error_string().str;
  EXPECT_EQ(names.size, enclaves.size);
  EXPECT_GT(names.size, 0u);
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&names));
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&namespaces));
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&enclaves));

  // The output array is contractually zero-initialized on entry.
  auto dirty = rcutils_get_zero_initialized_string_array();
  auto dirty_allocator = rcutils_get_default_allocator();
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_string_array_init(&dirty, 1, &dirty_allocator));
  auto ok_names = rcutils_get_zero_initialized_string_array();
  auto ok_namespaces = rcutils_get_zero_initialized_string_array();
  EXPECT_EQ(
    RMW_RET_INVALID_ARGUMENT,
    rmw_get_node_names_with_enclaves(node_, &ok_names, &ok_namespaces, &dirty));
  rmw_reset_error();
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&dirty));
}

TEST_F(RmwApi, counts_and_service_names)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/counted", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);

  size_t count = 0;
  EXPECT_EQ(RMW_RET_OK, rmw_count_publishers(node_, "/counted", &count));
  EXPECT_EQ(1u, count);
  EXPECT_EQ(RMW_RET_OK, rmw_count_subscribers(node_, "/counted", &count));
  EXPECT_EQ(0u, count);
  EXPECT_EQ(RMW_RET_OK, rmw_count_services(node_, "/nope", &count));
  EXPECT_EQ(0u, count);
  EXPECT_EQ(RMW_RET_OK, rmw_count_clients(node_, "/nope", &count));
  EXPECT_EQ(0u, count);
  // An invalid name is rejected before counting.
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_count_publishers(node_, "bad name", &count));
  rmw_reset_error();

  auto allocator = rcutils_get_default_allocator();
  auto names_and_types = rmw_get_zero_initialized_names_and_types();
  EXPECT_EQ(
    RMW_RET_OK, rmw_get_service_names_and_types(node_, &allocator, &names_and_types));
  EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&names_and_types));

  // A query about a node that does not exist has its own error code.
  auto by_node = rmw_get_zero_initialized_names_and_types();
  EXPECT_EQ(
    RMW_RET_NODE_NAME_NON_EXISTENT,
    rmw_get_publisher_names_and_types_by_node(
      node_, &allocator, "no_such_node", "/", false, &by_node));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

TEST_F(RmwApi, gids_and_comparison)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * first = rmw_create_publisher(
    node_, strings_ts(), "/gid_a", &rmw_qos_profile_default, &publisher_options);
  auto * second = rmw_create_publisher(
    node_, strings_ts(), "/gid_b", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, first);
  ASSERT_NE(nullptr, second);

  rmw_gid_t gid_a = {};
  rmw_gid_t gid_b = {};
  ASSERT_EQ(RMW_RET_OK, rmw_get_gid_for_publisher(first, &gid_a));
  ASSERT_EQ(RMW_RET_OK, rmw_get_gid_for_publisher(second, &gid_b));

  bool equal = false;
  EXPECT_EQ(RMW_RET_OK, rmw_compare_gids_equal(&gid_a, &gid_a, &equal));
  EXPECT_TRUE(equal);
  EXPECT_EQ(RMW_RET_OK, rmw_compare_gids_equal(&gid_a, &gid_b, &equal));
  EXPECT_FALSE(equal) << "each publisher must have a distinct gid";

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, second));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, first));
}

// ---------------------------------------------------------------------------
// take failure paths
// ---------------------------------------------------------------------------

/// A corrupt payload comes back as a deserialization failure.
/// publish_serialized_message puts arbitrary bytes on the wire.
TEST_F(RmwApi, take_reports_a_payload_it_cannot_deserialize)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/corrupt", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/corrupt", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  // Declares a string length with no body.
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto serialized = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 8, &allocator));
  const uint8_t corrupt[] = {0x00, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x7F};
  std::memcpy(serialized.buffer, corrupt, sizeof(corrupt));
  serialized.buffer_length = sizeof(corrupt);
  ASSERT_EQ(RMW_RET_OK, rmw_publish_serialized_message(publisher, &serialized, nullptr));

  test_msgs::msg::Strings message;
  bool taken = false;
  EXPECT_EQ(RMW_RET_ERROR, rmw_take(subscription, &message, &taken, nullptr));
  rmw_reset_error();

  // Same on the loaned path.
  ASSERT_EQ(RMW_RET_OK, rmw_publish_serialized_message(publisher, &serialized, nullptr));
  void * loaned = nullptr;
  EXPECT_EQ(RMW_RET_ERROR, rmw_take_loaned_message(subscription, &loaned, &taken, nullptr));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

/// Taking as a loan and returning it.
TEST_F(RmwApi, take_loaned_message_round_trips)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/loan_take", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/loan_take", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);
  ASSERT_TRUE(subscription->can_loan_messages);

  test_msgs::msg::Strings message;
  message.string_value = "loaned take";
  ASSERT_EQ(RMW_RET_OK, rmw_publish(publisher, &message, nullptr));

  void * loaned = nullptr;
  bool taken = false;
  rmw_message_info_t info = rmw_get_zero_initialized_message_info();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_take_loaned_message_with_info(subscription, &loaned, &taken, &info, nullptr));
  ASSERT_TRUE(taken);
  ASSERT_NE(nullptr, loaned);
  EXPECT_EQ("loaned take", static_cast<test_msgs::msg::Strings *>(loaned)->string_value);
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_subscription(subscription, loaned));

  // A pointer never lent is rejected.
  EXPECT_EQ(
    RMW_RET_ERROR, rmw_return_loaned_message_from_subscription(subscription, &message));
  rmw_reset_error();

  // Nothing comes out of an empty queue.
  loaned = nullptr;
  EXPECT_EQ(RMW_RET_OK, rmw_take_loaned_message(subscription, &loaned, &taken, nullptr));
  EXPECT_FALSE(taken);

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

// ---------------------------------------------------------------------------
// QoS events
// ---------------------------------------------------------------------------

/// rmw_take_event() writes a different struct per kind. Every kind is taken
/// once to check none of them is confused with another.
TEST_F(RmwApi, take_event_handles_every_supported_type)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/events", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/events", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  struct Case
  {
    rmw_event_type_t type;
    bool from_publisher;
    size_t status_size;
  };
  const Case cases[] = {
    {RMW_EVENT_PUBLICATION_MATCHED, true, sizeof(rmw_matched_status_t)},
    {RMW_EVENT_OFFERED_DEADLINE_MISSED, true, sizeof(rmw_offered_deadline_missed_status_t)},
    {RMW_EVENT_LIVELINESS_LOST, true, sizeof(rmw_liveliness_lost_status_t)},
    {RMW_EVENT_OFFERED_QOS_INCOMPATIBLE, true, sizeof(rmw_qos_incompatible_event_status_t)},
    {RMW_EVENT_SUBSCRIPTION_MATCHED, false, sizeof(rmw_matched_status_t)},
    {RMW_EVENT_REQUESTED_DEADLINE_MISSED, false, sizeof(rmw_requested_deadline_missed_status_t)},
    {RMW_EVENT_LIVELINESS_CHANGED, false, sizeof(rmw_liveliness_changed_status_t)},
    {RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, false, sizeof(rmw_qos_incompatible_event_status_t)},
  };

  for (const auto & test_case : cases) {
    rmw_event_t event = rmw_get_zero_initialized_event();
    const rmw_ret_t init_ret = test_case.from_publisher ?
      rmw_publisher_event_init(&event, publisher, test_case.type) :
      rmw_subscription_event_init(&event, subscription, test_case.type);
    ASSERT_EQ(RMW_RET_OK, init_ret) << "type=" << test_case.type;

    // The status differs in size per kind, so pass ample room.
    alignas(8) uint8_t status[64] = {};
    ASSERT_LE(test_case.status_size, sizeof(status));
    bool taken = false;
    EXPECT_EQ(RMW_RET_OK, rmw_take_event(&event, status, &taken)) << "type=" << test_case.type;
    // A cumulative status, so it can be taken with nothing changed.
    EXPECT_TRUE(taken) << "type=" << test_case.type;

    // Setting and clearing the callback.
    EXPECT_EQ(RMW_RET_OK, rmw_event_set_callback(&event, nullptr, nullptr));
  }

  // Unsupported kinds report false.
  EXPECT_FALSE(rmw_event_type_is_supported(RMW_EVENT_MESSAGE_LOST));

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

// ---------------------------------------------------------------------------
// service / client
// ---------------------------------------------------------------------------

TEST_F(RmwApi, client_without_a_server)
{
  auto * client = rmw_create_client(
    node_, srv_ts(), "/no_server", &rmw_qos_profile_services_default);
  ASSERT_NE(nullptr, client) << rmw_get_error_string().str;

  bool available = true;
  EXPECT_EQ(RMW_RET_OK, rmw_service_server_is_available(node_, client, &available));
  EXPECT_FALSE(available);

  // No server, so nothing can be sent.
  test_msgs::srv::BasicTypes_Request request;
  int64_t sequence = 0;
  EXPECT_EQ(RMW_RET_ERROR, rmw_send_request(client, &request, &sequence));
  rmw_reset_error();

  // And no response arrives.
  test_msgs::srv::BasicTypes_Response response;
  rmw_service_info_t info = {};
  bool taken = true;
  EXPECT_EQ(RMW_RET_OK, rmw_take_response(client, &info, &response, &taken));
  EXPECT_FALSE(taken);

  EXPECT_EQ(RMW_RET_OK, rmw_client_set_on_new_response_callback(client, nullptr, nullptr));

  rmw_qos_profile_t qos = rmw_qos_profile_unknown;
  EXPECT_EQ(RMW_RET_OK, rmw_client_request_publisher_get_actual_qos(client, &qos));
  EXPECT_NE(RMW_QOS_POLICY_RELIABILITY_UNKNOWN, qos.reliability);
  EXPECT_EQ(RMW_RET_OK, rmw_client_response_subscription_get_actual_qos(client, &qos));

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_client(node_, client));
}

TEST_F(RmwApi, service_without_a_request)
{
  auto * service = rmw_create_service(
    node_, srv_ts(), "/idle_service", &rmw_qos_profile_services_default);
  ASSERT_NE(nullptr, service) << rmw_get_error_string().str;

  test_msgs::srv::BasicTypes_Request request;
  rmw_service_info_t info = {};
  bool taken = true;
  EXPECT_EQ(RMW_RET_OK, rmw_take_request(service, &info, &request, &taken));
  EXPECT_FALSE(taken);

  // Responding to a request that was never recorded is an error.
  test_msgs::srv::BasicTypes_Response response;
  rmw_request_id_t id = {};
  id.sequence_number = 12345;
  EXPECT_EQ(RMW_RET_ERROR, rmw_send_response(service, &id, &response));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_service_set_on_new_request_callback(service, nullptr, nullptr));

  rmw_qos_profile_t qos = rmw_qos_profile_unknown;
  EXPECT_EQ(RMW_RET_OK, rmw_service_request_subscription_get_actual_qos(service, &qos));
  EXPECT_EQ(RMW_RET_OK, rmw_service_response_publisher_get_actual_qos(service, &qos));

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_service(node_, service));
}

// ---------------------------------------------------------------------------
// wait set
// ---------------------------------------------------------------------------

/// Null entries in the arrays are skipped: rcl passes null for unused slots.
TEST_F(RmwApi, wait_skips_null_entries)
{
  auto * wait_set = rmw_create_wait_set(&context_, 4);
  ASSERT_NE(nullptr, wait_set);

  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/waited", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  void * subscription_entries[2] = {nullptr, subscription->data};
  rmw_subscriptions_t subscriptions = {2, subscription_entries};
  void * event_entries[2] = {nullptr, nullptr};
  rmw_events_t events = {2, event_entries};

  rmw_time_t timeout = {0, 20000000};  // 20ms
  const rmw_ret_t ret =
    rmw_wait(&subscriptions, nullptr, nullptr, nullptr, &events, wait_set, &timeout);
  EXPECT_EQ(RMW_RET_TIMEOUT, ret);
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_wait_set(wait_set));
}

TEST_F(RmwApi, guard_condition_wakes_the_wait_set)
{
  auto * wait_set = rmw_create_wait_set(&context_, 1);
  ASSERT_NE(nullptr, wait_set);
  auto * guard = rmw_create_guard_condition(&context_);
  ASSERT_NE(nullptr, guard);

  ASSERT_EQ(RMW_RET_OK, rmw_trigger_guard_condition(guard));

  void * entries[1] = {guard->data};
  rmw_guard_conditions_t guards = {1, entries};
  rmw_time_t timeout = {5, 0};
  EXPECT_EQ(RMW_RET_OK, rmw_wait(nullptr, &guards, nullptr, nullptr, nullptr, wait_set,
    &timeout));
  EXPECT_NE(nullptr, entries[0]) << "the guard condition that woke it was cleared";

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_guard_condition(guard));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_wait_set(wait_set));
}

TEST_F(RmwApi, publisher_and_subscription_queries)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/queried", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/queried", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  size_t count = 0;
  EXPECT_EQ(RMW_RET_OK, rmw_publisher_count_matched_subscriptions(publisher, &count));
  EXPECT_EQ(1u, count);
  EXPECT_EQ(RMW_RET_OK, rmw_subscription_count_matched_publishers(subscription, &count));
  EXPECT_EQ(1u, count);

  rmw_qos_profile_t qos = rmw_qos_profile_unknown;
  EXPECT_EQ(RMW_RET_OK, rmw_publisher_get_actual_qos(publisher, &qos));
  EXPECT_EQ(RMW_RET_OK, rmw_subscription_get_actual_qos(subscription, &qos));

  // Nothing to acknowledge, so this returns at once.
  EXPECT_EQ(RMW_RET_OK, rmw_publisher_wait_for_all_acked(publisher, {0, 0}));

  // Under AUTOMATIC liveliness, asserting simply succeeds.
  EXPECT_EQ(RMW_RET_OK, rmw_publisher_assert_liveliness(publisher));

  EXPECT_EQ(
    RMW_RET_OK, rmw_subscription_set_on_new_message_callback(subscription, nullptr, nullptr));

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

// ---------------------------------------------------------------------------
// Copying and initializing init_options
// ---------------------------------------------------------------------------

namespace
{

/// init_options carrying an enclave, a security_root_path and a static peer.
/// That is three allocations, so the failure count selects the path.
class InitOptions : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ASSERT_EQ(RMW_RET_OK, rmw_init_options_init(&source_, allocator));
    source_.enclave = rcutils_strdup("/", allocator);
    ASSERT_NE(nullptr, source_.enclave);
    source_.security_options.security_root_path = rcutils_strdup("/tmp", allocator);
    ASSERT_NE(nullptr, source_.security_options.security_root_path);
    ASSERT_EQ(
      RMW_RET_OK,
      rmw_discovery_options_init(&source_.discovery_options, 1, &allocator));
    snprintf(
      source_.discovery_options.static_peers[0].peer_address,
      RMW_DISCOVERY_OPTIONS_STATIC_PEERS_MAX_LENGTH, "127.0.0.1");
  }

  void TearDown() override
  {
    EXPECT_EQ(RMW_RET_OK, rmw_init_options_fini(&source_));
  }

  rmw_init_options_t source_ = rmw_get_zero_initialized_init_options();
};

}  // namespace

TEST_F(InitOptions, a_copy_carries_every_field)
{
  rmw_init_options_t copy = rmw_get_zero_initialized_init_options();
  ASSERT_EQ(RMW_RET_OK, rmw_init_options_copy(&source_, &copy));
  EXPECT_STREQ("/", copy.enclave);
  EXPECT_STREQ("/tmp", copy.security_options.security_root_path);
  ASSERT_EQ(1u, copy.discovery_options.static_peers_count);
  EXPECT_STREQ("127.0.0.1", copy.discovery_options.static_peers[0].peer_address);
  EXPECT_EQ(RMW_RET_OK, rmw_init_options_fini(&copy));
}

TEST_F(InitOptions, copying_onto_an_initialized_destination_is_refused)
{
  rmw_init_options_t copy = rmw_get_zero_initialized_init_options();
  ASSERT_EQ(RMW_RET_OK, rmw_init_options_copy(&source_, &copy));
  rmw_init_options_t again = rmw_get_zero_initialized_init_options();
  ASSERT_EQ(RMW_RET_OK, rmw_init_options_copy(&source_, &again));

  // A non-empty destination is rejected rather than overwritten, which would
  // leak what it held.
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_init_options_copy(&copy, &again));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_init_options_fini(&copy));
  EXPECT_EQ(RMW_RET_OK, rmw_init_options_fini(&again));
}

/// The enclave, security_root_path and static peers allocate in that order,
/// so the failure count picks which cleanup path runs.
TEST_F(InitOptions, a_copy_that_runs_out_of_memory_leaves_nothing_behind)
{
  for (size_t successes = 0; successes < 3; ++successes) {
    source_.allocator = FailingAllocator::make(successes);
    rmw_init_options_t copy = rmw_get_zero_initialized_init_options();
    EXPECT_NE(RMW_RET_OK, rmw_init_options_copy(&source_, &copy)) << "successes=" << successes;
    rmw_reset_error();
    // Having failed, the destination has to still be zero-initialized.
    EXPECT_EQ(nullptr, copy.implementation_identifier) << "successes=" << successes;
  }
  source_.allocator = rcutils_get_default_allocator();
}

TEST_F(InitOptions, init_refuses_a_context_it_cannot_populate)
{
  source_.allocator = FailingAllocator::make(0);
  rmw_context_t context = rmw_get_zero_initialized_context();
  EXPECT_NE(RMW_RET_OK, rmw_init(&source_, &context));
  rmw_reset_error();
  EXPECT_EQ(nullptr, context.impl);
  source_.allocator = rcutils_get_default_allocator();
}

TEST_F(InitOptions, init_reports_a_mapping_file_it_cannot_read)
{
  // The extern mapping is read before the vsomeip configuration is generated.
  ASSERT_EQ(0, setenv("RMW_SOMEIP_MAPPING_FILE", "/nonexistent/mapping.yaml", 1));
  rmw_context_t context = rmw_get_zero_initialized_context();
  EXPECT_EQ(RMW_RET_ERROR, rmw_init(&source_, &context));
  rmw_reset_error();
  EXPECT_EQ(nullptr, context.impl);
  ASSERT_EQ(0, unsetenv("RMW_SOMEIP_MAPPING_FILE"));
}

TEST(RmwContext, a_context_with_live_nodes_cannot_be_finalized)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_init_options_t options = rmw_get_zero_initialized_init_options();
  ASSERT_EQ(RMW_RET_OK, rmw_init_options_init(&options, allocator));
  options.enclave = rcutils_strdup("/", allocator);
  ASSERT_NE(nullptr, options.enclave);

  rmw_context_t context = rmw_get_zero_initialized_context();
  ASSERT_EQ(RMW_RET_OK, rmw_init(&options, &context));
  rmw_node_t * node = rmw_create_node(&context, "lingering", "/");
  ASSERT_NE(nullptr, node);

  // Cannot finalize before shutdown.
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_context_fini(&context));
  rmw_reset_error();

  ASSERT_EQ(RMW_RET_OK, rmw_shutdown(&context));
  // Nor while a node remains.
  EXPECT_EQ(RMW_RET_ERROR, rmw_context_fini(&context));
  rmw_reset_error();

  ASSERT_EQ(RMW_RET_OK, rmw_destroy_node(node));
  EXPECT_EQ(RMW_RET_OK, rmw_context_fini(&context));
  EXPECT_EQ(RMW_RET_OK, rmw_init_options_fini(&options));
}

// ---------------------------------------------------------------------------
// Graph queries against a populated graph
// ---------------------------------------------------------------------------

/// Entities this process creates appear in its own GraphCache. The code that
/// actually fills the name and type arrays only runs with at least one entry.
class RmwGraph : public RmwApi
{
protected:
  void SetUp() override
  {
    RmwApi::SetUp();
    auto publisher_options = rmw_get_default_publisher_options();
    publisher_ = rmw_create_publisher(
      node_, strings_ts(), "/graph_topic", &rmw_qos_profile_default, &publisher_options);
    ASSERT_NE(nullptr, publisher_);

    auto subscription_options = rmw_get_default_subscription_options();
    subscription_ = rmw_create_subscription(
      node_, strings_ts(), "/graph_topic", &rmw_qos_profile_default, &subscription_options);
    ASSERT_NE(nullptr, subscription_);

    service_ = rmw_create_service(node_, srv_ts(), "/graph_service", &rmw_qos_profile_services_default);
    ASSERT_NE(nullptr, service_);
    client_ = rmw_create_client(node_, srv_ts(), "/graph_service", &rmw_qos_profile_services_default);
    ASSERT_NE(nullptr, client_);
  }

  void TearDown() override
  {
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_client(node_, client_));
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_service(node_, service_));
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription_));
    EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher_));
    RmwApi::TearDown();
  }

  /// Does names_and_types contain this name?
  static bool contains(const rmw_names_and_types_t & names_and_types, const char * name)
  {
    for (size_t i = 0; i < names_and_types.names.size; ++i) {
      if (std::string(names_and_types.names.data[i]) == name) {
        return true;
      }
    }
    return false;
  }

  rmw_publisher_t * publisher_ {nullptr};
  rmw_subscription_t * subscription_ {nullptr};
  rmw_service_t * service_ {nullptr};
  rmw_client_t * client_ {nullptr};
};

TEST_F(RmwGraph, topic_and_service_names_are_listed)
{
  auto allocator = rcutils_get_default_allocator();

  auto topics = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(RMW_RET_OK, rmw_get_topic_names_and_types(node_, &allocator, false, &topics));
  EXPECT_TRUE(contains(topics, "/graph_topic"));
  for (size_t i = 0; i < topics.names.size; ++i) {
    if (std::string(topics.names.data[i]) == "/graph_topic") {
      ASSERT_EQ(1u, topics.types[i].size);
      EXPECT_STREQ("test_msgs/msg/Strings", topics.types[i].data[0]);
    }
  }
  EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&topics));

  auto services = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(RMW_RET_OK, rmw_get_service_names_and_types(node_, &allocator, &services));
  EXPECT_TRUE(contains(services, "/graph_service"));
  EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&services));
}

TEST_F(RmwGraph, names_and_types_are_listed_per_node)
{
  auto allocator = rcutils_get_default_allocator();
  struct Query
  {
    const char * label;
    rmw_ret_t (* function)(
      const rmw_node_t *, rcutils_allocator_t *, const char *, const char *, bool,
      rmw_names_and_types_t *);
    const char * expected;
  };
  const Query queries[] = {
    {"publishers", rmw_get_publisher_names_and_types_by_node, "/graph_topic"},
    {"subscribers", rmw_get_subscriber_names_and_types_by_node, "/graph_topic"},
  };
  for (const auto & query : queries) {
    auto result = rmw_get_zero_initialized_names_and_types();
    ASSERT_EQ(
      RMW_RET_OK,
      query.function(node_, &allocator, "unit_node", "/", false, &result)) << query.label;
    EXPECT_TRUE(contains(result, query.expected)) << query.label;
    EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&result));
  }

  auto services = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_service_names_and_types_by_node(node_, &allocator, "unit_node", "/", &services));
  EXPECT_TRUE(contains(services, "/graph_service"));
  EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&services));

  auto clients = rmw_get_zero_initialized_names_and_types();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_client_names_and_types_by_node(node_, &allocator, "unit_node", "/", &clients));
  EXPECT_TRUE(contains(clients, "/graph_service"));
  EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&clients));
}

TEST_F(RmwGraph, an_invalid_node_namespace_is_refused)
{
  auto allocator = rcutils_get_default_allocator();
  auto result = rmw_get_zero_initialized_names_and_types();
  EXPECT_EQ(
    RMW_RET_INVALID_ARGUMENT,
    rmw_get_publisher_names_and_types_by_node(node_, &allocator, "unit_node", "bad ns", false,
    &result));
  rmw_reset_error();
}

TEST_F(RmwGraph, listing_names_and_types_that_cannot_be_allocated_fails_cleanly)
{
  // Each name costs a strdup, a string_array_init and one strdup per type.
  // Wherever it runs out, no partially filled array may be returned.
  for (size_t successes = 0; successes < 6; ++successes) {
    auto allocator = FailingAllocator::make(successes);
    auto topics = rmw_get_zero_initialized_names_and_types();
    const rmw_ret_t ret = rmw_get_topic_names_and_types(node_, &allocator, false, &topics);
    rmw_reset_error();
    if (ret == RMW_RET_OK) {
      EXPECT_EQ(RMW_RET_OK, rmw_names_and_types_fini(&topics));
      continue;
    }
    // The exact code comes from rmw's own array API, so this only checks that
    // it failed and that no half-filled array came back.
    EXPECT_NE(RMW_RET_OK, ret) << "successes=" << successes;
    EXPECT_EQ(0u, topics.names.size) << "successes=" << successes;
  }
}

TEST_F(RmwGraph, endpoint_info_describes_each_publisher)
{
  auto allocator = rcutils_get_default_allocator();
  auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_publishers_info_by_topic(node_, &allocator, "/graph_topic", false, &info));
  ASSERT_EQ(1u, info.size);
  EXPECT_STREQ("unit_node", info.info_array[0].node_name);
  EXPECT_STREQ("/", info.info_array[0].node_namespace);
  EXPECT_STREQ("test_msgs/msg/Strings", info.info_array[0].topic_type);
  EXPECT_EQ(RMW_ENDPOINT_PUBLISHER, info.info_array[0].endpoint_type);
  EXPECT_EQ(RMW_RET_OK, rmw_topic_endpoint_info_array_fini(&info, &allocator));

  auto subscribers = rmw_get_zero_initialized_topic_endpoint_info_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_subscriptions_info_by_topic(node_, &allocator, "/graph_topic", false, &subscribers));
  ASSERT_EQ(1u, subscribers.size);
  EXPECT_EQ(RMW_ENDPOINT_SUBSCRIPTION, subscribers.info_array[0].endpoint_type);
  EXPECT_EQ(RMW_RET_OK, rmw_topic_endpoint_info_array_fini(&subscribers, &allocator));
}

TEST_F(RmwGraph, endpoint_info_that_cannot_be_allocated_fails_cleanly)
{
  for (size_t successes = 0; successes < 5; ++successes) {
    auto allocator = FailingAllocator::make(successes);
    auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
    const rmw_ret_t ret =
      rmw_get_publishers_info_by_topic(node_, &allocator, "/graph_topic", false, &info);
    rmw_reset_error();
    if (ret == RMW_RET_OK) {
      auto fini_allocator = rcutils_get_default_allocator();
      EXPECT_EQ(RMW_RET_OK, rmw_topic_endpoint_info_array_fini(&info, &fini_allocator));
      continue;
    }
    EXPECT_NE(RMW_RET_OK, ret) << "successes=" << successes;
    EXPECT_EQ(0u, info.size) << "successes=" << successes;
  }
}

// ---------------------------------------------------------------------------
// Around take
// ---------------------------------------------------------------------------

TEST_F(RmwApi, take_sequence_stops_at_the_first_empty_slot)
{
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/sequence", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  auto sequence_allocator = rcutils_get_default_allocator();
  rmw_message_sequence_t messages;
  ASSERT_EQ(RMW_RET_OK, rmw_message_sequence_init(&messages, 2, &sequence_allocator));
  rmw_message_info_sequence_t infos;
  ASSERT_EQ(RMW_RET_OK, rmw_message_info_sequence_init(&infos, 2, &sequence_allocator));

  test_msgs::msg::Strings first;
  test_msgs::msg::Strings second;
  messages.data[0] = &first;
  messages.data[1] = &second;

  // Nothing arrived, so it returns zero; an empty first slot stops the loop.
  size_t taken = 1;
  EXPECT_EQ(RMW_RET_OK, rmw_take_sequence(subscription, 2, &messages, &infos, &taken, nullptr));
  EXPECT_EQ(0u, taken);
  EXPECT_EQ(0u, messages.size);

  EXPECT_EQ(RMW_RET_OK, rmw_message_sequence_fini(&messages));
  EXPECT_EQ(RMW_RET_OK, rmw_message_info_sequence_fini(&infos));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
}

TEST_F(RmwApi, taking_a_serialized_message_into_a_buffer_that_cannot_grow_fails)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/serialized_take", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/serialized_take", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  test_msgs::msg::Strings message;
  message.string_value = "payload that does not fit in nothing";
  ASSERT_EQ(RMW_RET_OK, rmw_publish(publisher, &message, nullptr));

  // Zero capacity, with an allocator that fails when it tries to grow.
  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  auto failing = FailingAllocator::make(1);
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 0, &failing));
  FailingAllocator::remaining = 0;

  bool taken = false;
  EXPECT_EQ(
    RMW_RET_BAD_ALLOC,
    rmw_take_serialized_message(subscription, &serialized, &taken, nullptr));
  rmw_reset_error();
  EXPECT_FALSE(taken);

  serialized.allocator = rcutils_get_default_allocator();
  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

TEST_F(RmwApi, returning_a_message_the_publisher_never_loaned_is_refused)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/foreign_loan", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);

  test_msgs::msg::Strings not_loaned;
  EXPECT_EQ(
    RMW_RET_ERROR, rmw_return_loaned_message_from_publisher(publisher, &not_loaned));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

TEST_F(RmwApi, features_without_a_someip_counterpart_report_unsupported)
{
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/unsupported", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  auto allocator = rcutils_get_default_allocator();

  // Content filters: SOME/IP-SD has nowhere to carry a predicate.
  auto filter_options = rmw_get_zero_initialized_content_filter_options();
  EXPECT_EQ(
    RMW_RET_UNSUPPORTED,
    rmw_subscription_get_content_filter(subscription, &allocator, &filter_options));
  rmw_reset_error();
  EXPECT_EQ(
    RMW_RET_UNSUPPORTED, rmw_subscription_set_content_filter(subscription, &filter_options));
  rmw_reset_error();

  // Network flows: vsomeip does not expose the endpoints in use.
  auto flows = rmw_get_zero_initialized_network_flow_endpoint_array();
  EXPECT_EQ(
    RMW_RET_UNSUPPORTED,
    rmw_subscription_get_network_flow_endpoints(subscription, &allocator, &flows));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
}

/// vsomeip silently ignores a configuration file it cannot read and starts
/// with defaults. A mistyped path produces no diagnostic and the process runs
/// with settings nobody chose, so the RMW checks readability itself.
TEST_F(InitOptions, init_reports_a_config_file_it_cannot_read)
{
  ASSERT_EQ(0, setenv("RMW_SOMEIP_CONFIG_FILE", "/nonexistent/dir/vsomeip.json", 1));
  rmw_context_t context = rmw_get_zero_initialized_context();
  EXPECT_EQ(RMW_RET_ERROR, rmw_init(&source_, &context));
  rmw_reset_error();
  EXPECT_EQ(nullptr, context.impl);
  ASSERT_EQ(0, unsetenv("RMW_SOMEIP_CONFIG_FILE"));
}

TEST_F(InitOptions, a_readable_config_file_is_used_as_is)
{
  const char * path = "/tmp/rmw_someip_user_config.json";
  {
    std::ofstream out(path);
    out << "{ \"unicast\": \"127.0.0.1\" }";
  }
  ASSERT_EQ(0, setenv("RMW_SOMEIP_CONFIG_FILE", path, 1));
  rmw_context_t context = rmw_get_zero_initialized_context();
  ASSERT_EQ(RMW_RET_OK, rmw_init(&source_, &context)) << rmw_get_error_string().str;
  EXPECT_EQ(RMW_RET_OK, rmw_shutdown(&context));
  EXPECT_EQ(RMW_RET_OK, rmw_context_fini(&context));

  // The user's file is not deleted.
  EXPECT_TRUE(std::ifstream(path).good());
  std::remove(path);
  ASSERT_EQ(0, unsetenv("RMW_SOMEIP_CONFIG_FILE"));
}

TEST_F(RmwApi, take_sequence_stops_after_the_last_message)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/one_message", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/one_message", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  test_msgs::msg::Strings sent;
  sent.string_value = "only one";
  ASSERT_EQ(RMW_RET_OK, rmw_publish(publisher, &sent, nullptr));

  auto sequence_allocator = rcutils_get_default_allocator();
  rmw_message_sequence_t messages;
  ASSERT_EQ(RMW_RET_OK, rmw_message_sequence_init(&messages, 3, &sequence_allocator));
  rmw_message_info_sequence_t infos;
  ASSERT_EQ(RMW_RET_OK, rmw_message_info_sequence_init(&infos, 3, &sequence_allocator));

  test_msgs::msg::Strings slots[3];
  for (size_t i = 0; i < 3; ++i) {
    messages.data[i] = &slots[i];
  }

  // Three requested, one available; it stops at the empty second slot.
  size_t taken = 0;
  EXPECT_EQ(RMW_RET_OK, rmw_take_sequence(subscription, 3, &messages, &infos, &taken, nullptr));
  EXPECT_EQ(1u, taken);
  EXPECT_EQ(1u, messages.size);
  EXPECT_EQ("only one", slots[0].string_value);

  EXPECT_EQ(RMW_RET_OK, rmw_message_sequence_fini(&messages));
  EXPECT_EQ(RMW_RET_OK, rmw_message_info_sequence_fini(&infos));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}

TEST(RmwQosCompatibility, an_incompatible_pair_explains_itself)
{
  rmw_qos_profile_t publisher_qos = rmw_qos_profile_default;
  publisher_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rmw_qos_profile_t subscription_qos = rmw_qos_profile_default;
  subscription_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  char reason[256] = {};
  rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_OK;
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_qos_profile_check_compatible(
      publisher_qos, subscription_qos, &compatibility, reason, sizeof(reason)));
  EXPECT_EQ(RMW_QOS_COMPATIBILITY_ERROR, compatibility);
  EXPECT_GT(std::strlen(reason), 0u);

  // A reason that does not fit is truncated, always NUL-terminated.
  char narrow[8] = {};
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_qos_profile_check_compatible(
      publisher_qos, subscription_qos, &compatibility, narrow, sizeof(narrow)));
  EXPECT_EQ(RMW_QOS_COMPATIBILITY_ERROR, compatibility);
  EXPECT_LT(std::strlen(narrow), sizeof(narrow));

  // A leftover SYSTEM_DEFAULT gives the "cannot be decided" warning.
  char warning[256] = {};
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_qos_profile_check_compatible(
      publisher_qos, publisher_qos, &compatibility, warning, sizeof(warning)));
  EXPECT_EQ(RMW_QOS_COMPATIBILITY_WARNING, compatibility);
  EXPECT_GT(std::strlen(warning), 0u);

  // With every field concrete, the reason is empty.
  rmw_qos_profile_t resolved = rmw_qos_profile_default;
  resolved.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  resolved.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  resolved.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  resolved.deadline = RMW_DURATION_INFINITE;
  resolved.liveliness_lease_duration = RMW_DURATION_INFINITE;
  char none[64] = {};
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_qos_profile_check_compatible(resolved, resolved, &compatibility, none, sizeof(none)));
  EXPECT_EQ(RMW_QOS_COMPATIBILITY_OK, compatibility);
  EXPECT_STREQ("", none);
}

TEST_F(RmwGraph, endpoint_info_refuses_an_array_that_is_not_zeroed)
{
  auto allocator = rcutils_get_default_allocator();
  auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
  ASSERT_EQ(
    RMW_RET_OK,
    rmw_get_publishers_info_by_topic(node_, &allocator, "/graph_topic", false, &info));
  ASSERT_EQ(1u, info.size);

  // An array that already holds data is rejected, not overwritten.
  EXPECT_EQ(
    RMW_RET_INVALID_ARGUMENT,
    rmw_get_publishers_info_by_topic(node_, &allocator, "/graph_topic", false, &info));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_topic_endpoint_info_array_fini(&info, &allocator));
}

/// rmw_serialize() resolves the typesupport and then writes CDR. A type it
/// cannot write — a malformed descriptor rosidl would never emit — fails partway
/// and reports it.
TEST(RmwSerialize, a_message_that_cannot_be_written_is_reported)
{
  rosidl_typesupport_introspection_c__MessageMember member {};
  member.name_ = "mystery";
  member.type_id_ = 200;   // an unknown type ID

  rosidl_typesupport_introspection_c__MessageMembers members {};
  members.message_namespace_ = "synthetic__msg";
  members.message_name_ = "Broken";
  members.member_count_ = 1;
  members.size_of_ = sizeof(int32_t);
  members.members_ = &member;

  rosidl_message_type_support_t handle {};
  handle.typesupport_identifier = rosidl_typesupport_introspection_c__identifier;
  handle.data = &members;
  handle.func = [](const rosidl_message_type_support_t * self, const char *) {return self;};

  int32_t message = 0;
  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  auto allocator = rcutils_get_default_allocator();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 16, &allocator));

  EXPECT_EQ(RMW_RET_ERROR, rmw_serialize(&message, &handle, &serialized));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
}

TEST_F(RmwGraph, a_topic_with_no_endpoints_yields_an_empty_array)
{
  auto allocator = rcutils_get_default_allocator();
  auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
  EXPECT_EQ(
    RMW_RET_OK,
    rmw_get_publishers_info_by_topic(node_, &allocator, "/nobody_here", false, &info));
  EXPECT_EQ(0u, info.size);
  EXPECT_EQ(RMW_RET_OK, rmw_topic_endpoint_info_array_fini(&info, &allocator));
}

TEST_F(RmwGraph, an_invalid_node_name_is_refused)
{
  auto allocator = rcutils_get_default_allocator();
  auto result = rmw_get_zero_initialized_names_and_types();
  EXPECT_EQ(
    RMW_RET_INVALID_ARGUMENT,
    rmw_get_publisher_names_and_types_by_node(
      node_, &allocator, "bad node name", "/", false, &result));
  rmw_reset_error();
}

TEST_F(RmwApi, take_sequence_reports_a_payload_it_cannot_deserialize)
{
  auto publisher_options = rmw_get_default_publisher_options();
  auto * publisher = rmw_create_publisher(
    node_, strings_ts(), "/corrupt_sequence", &rmw_qos_profile_default, &publisher_options);
  ASSERT_NE(nullptr, publisher);
  auto subscription_options = rmw_get_default_subscription_options();
  auto * subscription = rmw_create_subscription(
    node_, strings_ts(), "/corrupt_sequence", &rmw_qos_profile_default, &subscription_options);
  ASSERT_NE(nullptr, subscription);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto serialized = rmw_get_zero_initialized_serialized_message();
  ASSERT_EQ(RMW_RET_OK, rmw_serialized_message_init(&serialized, 8, &allocator));
  const uint8_t corrupt[] = {0x00, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x7F};
  std::memcpy(serialized.buffer, corrupt, sizeof(corrupt));
  serialized.buffer_length = sizeof(corrupt);
  ASSERT_EQ(RMW_RET_OK, rmw_publish_serialized_message(publisher, &serialized, nullptr));

  rmw_message_sequence_t messages;
  ASSERT_EQ(RMW_RET_OK, rmw_message_sequence_init(&messages, 2, &allocator));
  rmw_message_info_sequence_t infos;
  ASSERT_EQ(RMW_RET_OK, rmw_message_info_sequence_init(&infos, 2, &allocator));
  test_msgs::msg::Strings slots[2];
  messages.data[0] = &slots[0];
  messages.data[1] = &slots[1];

  // A failure on the first take stops the loop and is reported.
  size_t taken = 0;
  EXPECT_EQ(
    RMW_RET_ERROR, rmw_take_sequence(subscription, 2, &messages, &infos, &taken, nullptr));
  rmw_reset_error();

  EXPECT_EQ(RMW_RET_OK, rmw_message_sequence_fini(&messages));
  EXPECT_EQ(RMW_RET_OK, rmw_message_info_sequence_fini(&infos));
  EXPECT_EQ(RMW_RET_OK, rmw_serialized_message_fini(&serialized));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_subscription(node_, subscription));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(node_, publisher));
}
