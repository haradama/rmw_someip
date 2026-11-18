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
// Argument checking and QoS resolution.
//
// Argument checking is part of the rmw contract, down to which error code each
// bad argument produces (docs/design.md §15).

#include "gtest/gtest.h"

#include "rmw/error_handling.h"
#include "rmw/qos_profiles.h"

#include "detail/rmw_checks.hpp"

TEST(ValidateTopicName, accepts_a_fully_qualified_name)
{
  EXPECT_EQ(RMW_RET_OK, rmw_someip_cpp::validate_topic_name("/chatter"));
  EXPECT_EQ(RMW_RET_OK, rmw_someip_cpp::validate_topic_name("/ns/chatter"));
}

TEST(ValidateTopicName, rejects_null)
{
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_someip_cpp::validate_topic_name(nullptr));
  rmw_reset_error();
}

TEST(ValidateTopicName, rejects_relative_empty_and_malformed_names)
{
  for (const char * name : {"", "chatter", "/foo bar", "//foo", "/foo/"}) {
    EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_someip_cpp::validate_topic_name(name))
      << "name='" << name << "'";
    rmw_reset_error();
  }
}

/// With avoid_ros_namespace_conventions set, ROS conventions are not applied:
/// the flag means the name is a DDS topic name chosen directly.
TEST(ValidateTopicName, skips_ros_rules_when_conventions_are_avoided)
{
  EXPECT_EQ(RMW_RET_OK, rmw_someip_cpp::validate_topic_name("chatter", true));
  EXPECT_EQ(RMW_RET_OK, rmw_someip_cpp::validate_topic_name("no/slash/prefix", true));
  // An empty name is still invalid.
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_someip_cpp::validate_topic_name("", true));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_someip_cpp::validate_topic_name(nullptr, true));
  rmw_reset_error();
}

/// rmw_*_get_actual_qos() must report the value actually in use, so
/// SYSTEM_DEFAULT cannot be passed through.
TEST(ResolveQos, replaces_system_default_with_concrete_values)
{
  rmw_qos_profile_t requested = rmw_qos_profile_default;
  requested.history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  requested.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  requested.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  requested.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
  requested.depth = 0;

  const auto resolved = rmw_someip_cpp::resolve_qos(requested);
  EXPECT_EQ(RMW_QOS_POLICY_HISTORY_KEEP_LAST, resolved.history);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_RELIABLE, resolved.reliability);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_VOLATILE, resolved.durability);
  EXPECT_EQ(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, resolved.liveliness);
  EXPECT_EQ(1u, resolved.depth);
}

TEST(ResolveQos, replaces_unknown_with_concrete_values)
{
  rmw_qos_profile_t requested = rmw_qos_profile_default;
  requested.history = RMW_QOS_POLICY_HISTORY_UNKNOWN;
  requested.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
  requested.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
  requested.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;

  const auto resolved = rmw_someip_cpp::resolve_qos(requested);
  EXPECT_EQ(RMW_QOS_POLICY_HISTORY_KEEP_LAST, resolved.history);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_RELIABLE, resolved.reliability);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_VOLATILE, resolved.durability);
  EXPECT_EQ(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, resolved.liveliness);
}

TEST(ResolveQos, leaves_concrete_values_alone)
{
  rmw_qos_profile_t requested = rmw_qos_profile_default;
  requested.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  requested.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  requested.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  requested.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  requested.depth = 42;

  const auto resolved = rmw_someip_cpp::resolve_qos(requested);
  EXPECT_EQ(RMW_QOS_POLICY_HISTORY_KEEP_ALL, resolved.history);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, resolved.reliability);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, resolved.durability);
  EXPECT_EQ(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, resolved.liveliness);
  EXPECT_EQ(42u, resolved.depth);
}
