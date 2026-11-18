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
// QoS compatibility.
//
// Three callers share this rule: the public API, the match counts and the
// incompatibility events. Drift would produce contradictions such as
// rmw_qos_profile_check_compatible() reporting ERROR while
// count_matched_subscriptions() still counts the pair, so every branch is
// pinned down here.

#include "gtest/gtest.h"

#include "rmw/qos_profiles.h"

#include "detail/qos_compat.hpp"

namespace
{
rmw_qos_profile_t concrete()
{
  // A leftover SYSTEM_DEFAULT falls into the warning case, so pin every field.
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  return qos;
}
}  // namespace

TEST(QosCompat, compatible_profiles_report_nothing)
{
  const auto result = rmw_someip_cpp::check_incompatible(concrete(), concrete());
  EXPECT_FALSE(result.incompatible());
  EXPECT_EQ(RMW_QOS_POLICY_INVALID, result.policy_kind);
  EXPECT_EQ(nullptr, result.reason);
  EXPECT_EQ(nullptr, rmw_someip_cpp::check_warning(concrete(), concrete()));
}

TEST(QosCompat, best_effort_publisher_and_reliable_subscription_is_incompatible)
{
  auto publisher = concrete();
  publisher.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  const auto result = rmw_someip_cpp::check_incompatible(publisher, concrete());
  ASSERT_TRUE(result.incompatible());
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY, result.policy_kind);
}

/// The other direction, a RELIABLE publisher and BEST_EFFORT subscription, is fine.
TEST(QosCompat, reliable_publisher_satisfies_a_best_effort_subscription)
{
  auto subscription = concrete();
  subscription.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  EXPECT_FALSE(rmw_someip_cpp::check_incompatible(concrete(), subscription).incompatible());
}

TEST(QosCompat, volatile_publisher_and_transient_local_subscription_is_incompatible)
{
  auto subscription = concrete();
  subscription.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  const auto result = rmw_someip_cpp::check_incompatible(concrete(), subscription);
  ASSERT_TRUE(result.incompatible());
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY, result.policy_kind);
}

TEST(QosCompat, automatic_publisher_and_manual_subscription_is_incompatible)
{
  auto subscription = concrete();
  subscription.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  const auto result = rmw_someip_cpp::check_incompatible(concrete(), subscription);
  ASSERT_TRUE(result.incompatible());
  EXPECT_EQ(RMW_QOS_POLICY_LIVELINESS, result.policy_kind);
}

/// With several incompatibilities, reliability is reported first.
TEST(QosCompat, reliability_is_reported_first)
{
  auto publisher = concrete();
  publisher.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto subscription = concrete();
  subscription.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  EXPECT_EQ(
    RMW_QOS_POLICY_RELIABILITY,
    rmw_someip_cpp::check_incompatible(publisher, subscription).policy_kind);
}

/// SYSTEM_DEFAULT and UNKNOWN leave the value undecided, hence a warning.
/// Liveliness has to be included, or two default profiles compare as OK,
/// which disagrees with implementations that return WARNING.
TEST(QosCompat, undetermined_reliability_warns)
{
  for (auto value :
    {RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT, RMW_QOS_POLICY_RELIABILITY_UNKNOWN})
  {
    auto publisher = concrete();
    publisher.reliability = value;
    EXPECT_NE(nullptr, rmw_someip_cpp::check_warning(publisher, concrete()));
    auto subscription = concrete();
    subscription.reliability = value;
    EXPECT_NE(nullptr, rmw_someip_cpp::check_warning(concrete(), subscription));
  }
}

TEST(QosCompat, undetermined_durability_warns)
{
  for (auto value :
    {RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT, RMW_QOS_POLICY_DURABILITY_UNKNOWN})
  {
    auto publisher = concrete();
    publisher.durability = value;
    EXPECT_NE(nullptr, rmw_someip_cpp::check_warning(publisher, concrete()));
    auto subscription = concrete();
    subscription.durability = value;
    EXPECT_NE(nullptr, rmw_someip_cpp::check_warning(concrete(), subscription));
  }
}

TEST(QosCompat, undetermined_liveliness_warns)
{
  for (auto value :
    {RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT, RMW_QOS_POLICY_LIVELINESS_UNKNOWN})
  {
    auto publisher = concrete();
    publisher.liveliness = value;
    EXPECT_NE(nullptr, rmw_someip_cpp::check_warning(publisher, concrete()));
    auto subscription = concrete();
    subscription.liveliness = value;
    EXPECT_NE(nullptr, rmw_someip_cpp::check_warning(concrete(), subscription));
  }
}
