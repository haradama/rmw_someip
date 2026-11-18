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
// GraphCache: every ROS graph introspection call is a query against it.

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rmw/qos_profiles.h"

#include "detail/graph_cache.hpp"

namespace
{

using rmw_someip_cpp::EndpointInfo;
using rmw_someip_cpp::EndpointKind;
using rmw_someip_cpp::GraphCache;
using rmw_someip_cpp::ParticipantEntitiesInfo;

rmw_qos_profile_t reliable_volatile()
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  return qos;
}

EndpointInfo make(
  EndpointKind kind, const std::string & name, const std::string & type,
  const std::string & ns = "/", const std::string & node = "n")
{
  EndpointInfo endpoint;
  endpoint.kind = kind;
  endpoint.topic_name = name;
  endpoint.type_name = type;
  endpoint.node_namespace = ns;
  endpoint.node_name = node;
  endpoint.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  endpoint.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  endpoint.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  return endpoint;
}

/// Build a cache with participant 1 (node /a) and participant 2 (node /ns/b).
/// GraphCache holds a mutex, so it is neither copyable nor movable and is
/// taken by reference.
void fill_cache(GraphCache & cache)
{
  ParticipantEntitiesInfo first;
  first.instance_id = 1;
  first.nodes.push_back({"/", "a", ""});
  first.endpoints.push_back(
    make(EndpointKind::Publisher, "/chatter", "std_msgs/msg/String", "/", "a"));
  first.endpoints.push_back(make(EndpointKind::Service, "/add", "example/srv/Add", "/", "a"));
  cache.update_participant(first);

  ParticipantEntitiesInfo second;
  second.instance_id = 2;
  second.nodes.push_back({"/ns", "b", ""});
  second.endpoints.push_back(
    make(EndpointKind::Subscription, "/chatter", "std_msgs/msg/String", "/ns", "b"));
  second.endpoints.push_back(
    make(EndpointKind::Client, "/add", "example/srv/Add", "/ns", "b"));
  cache.update_participant(second);
}

/// Shared fixture.
class GraphCacheTest : public ::testing::Test
{
protected:
  void SetUp() override {fill_cache(cache);}
  GraphCache cache;
};

}  // namespace

TEST(GraphCache, update_reports_whether_anything_changed)
{
  GraphCache cache;
  ParticipantEntitiesInfo info;
  info.instance_id = 1;
  info.endpoints.push_back(make(EndpointKind::Publisher, "/t", "T"));

  EXPECT_TRUE(cache.update_participant(info));
  // The graph channel repeats the same contents. Returning false when nothing
  // changed keeps the executor from waking for no reason.
  EXPECT_FALSE(cache.update_participant(info));

  info.endpoints.push_back(make(EndpointKind::Publisher, "/t2", "T"));
  EXPECT_TRUE(cache.update_participant(info));

  // A change in node count alone still counts as a change.
  info.nodes.push_back({"/", "n", ""});
  EXPECT_TRUE(cache.update_participant(info));
}

TEST(GraphCache, detects_changes_in_every_compared_field)
{
  const char * const fields[] =
  {"kind", "topic", "type", "event", "eventgroup", "node", "namespace"};
  for (const char * field : fields) {
    GraphCache cache;
    ParticipantEntitiesInfo info;
    info.instance_id = 1;
    info.endpoints.push_back(make(EndpointKind::Publisher, "/t", "T"));
    ASSERT_TRUE(cache.update_participant(info));

    auto & endpoint = info.endpoints[0];
    const std::string name = field;
    if (name == "kind") {endpoint.kind = EndpointKind::Subscription;} else if (name == "topic") {
      endpoint.topic_name = "/other";
    } else if (name == "type") {endpoint.type_name = "Other";} else if (name == "event") {
      endpoint.event_id = 9;
    } else if (name == "eventgroup") {endpoint.eventgroup_id = 9;} else if (name == "node") {
      endpoint.node_name = "other";
    } else {endpoint.node_namespace = "/other";}
    EXPECT_TRUE(cache.update_participant(info)) << "field=" << field;
  }
}

TEST_F(GraphCacheTest, remove_participant_reports_whether_it_existed)
{
  EXPECT_TRUE(cache.remove_participant(1));
  EXPECT_FALSE(cache.remove_participant(1));
  EXPECT_EQ(0u, cache.count_publishers("/chatter"));
}

TEST_F(GraphCacheTest, node_names_lists_every_participant)
{
  const auto nodes = cache.node_names();
  ASSERT_EQ(2u, nodes.size());
  EXPECT_TRUE(cache.has_node("/", "a"));
  EXPECT_TRUE(cache.has_node("/ns", "b"));
  EXPECT_FALSE(cache.has_node("/", "b"));
  EXPECT_FALSE(cache.has_node("/ns", "a"));
}

TEST_F(GraphCacheTest, counts_by_kind)
{
  EXPECT_EQ(1u, cache.count_publishers("/chatter"));
  EXPECT_EQ(1u, cache.count_subscriptions("/chatter"));
  EXPECT_EQ(0u, cache.count_publishers("/nope"));
  EXPECT_EQ(1u, cache.count_services("/add"));
  EXPECT_EQ(1u, cache.count_clients("/add"));
  EXPECT_EQ(0u, cache.count_services("/nope"));
}

TEST_F(GraphCacheTest, find_publishers_excludes_the_given_instance)
{
  EXPECT_EQ(1u, cache.find_publishers("/chatter", 2).size());
  EXPECT_TRUE(cache.find_publishers("/chatter", 1).empty());

  EXPECT_EQ(1u, cache.all_publishers(2).size());
  EXPECT_TRUE(cache.all_publishers(1).empty());
}

TEST_F(GraphCacheTest, topic_names_and_types_can_select_kinds)
{
  const auto both = cache.topic_names_and_types(true, true);
  ASSERT_EQ(1u, both.size());
  ASSERT_EQ(1u, both.at("/chatter").size());
  EXPECT_EQ("std_msgs/msg/String", both.at("/chatter")[0]);

  EXPECT_EQ(1u, cache.topic_names_and_types(true, false).size());
  EXPECT_EQ(1u, cache.topic_names_and_types(false, true).size());
  EXPECT_TRUE(cache.topic_names_and_types(false, false).empty());
}

/// Two types on one topic both appear; duplicates collapse to one.
TEST(GraphCache, topic_names_and_types_deduplicates)
{
  GraphCache cache;
  ParticipantEntitiesInfo info;
  info.instance_id = 1;
  info.endpoints.push_back(make(EndpointKind::Publisher, "/t", "A"));
  info.endpoints.push_back(make(EndpointKind::Publisher, "/t", "A"));
  info.endpoints.push_back(make(EndpointKind::Publisher, "/t", "B"));
  cache.update_participant(info);
  EXPECT_EQ(2u, cache.topic_names_and_types(true, false).at("/t").size());
}

TEST_F(GraphCacheTest, service_names_and_types_can_select_kinds)
{
  EXPECT_EQ(1u, cache.service_names_and_types(true, true).size());
  EXPECT_EQ(1u, cache.service_names_and_types(true, false).size());
  EXPECT_EQ(1u, cache.service_names_and_types(false, true).size());
  EXPECT_TRUE(cache.service_names_and_types(false, false).empty());
}

TEST_F(GraphCacheTest, names_and_types_of_node_filters_by_node)
{
  EXPECT_EQ(1u, cache.names_and_types_of_node("/", "a", EndpointKind::Publisher).size());
  EXPECT_TRUE(cache.names_and_types_of_node("/", "a", EndpointKind::Subscription).empty());
  EXPECT_EQ(1u, cache.names_and_types_of_node("/ns", "b", EndpointKind::Subscription).size());
  EXPECT_EQ(1u, cache.names_and_types_of_node("/", "a", EndpointKind::Service).size());
  EXPECT_EQ(1u, cache.names_and_types_of_node("/ns", "b", EndpointKind::Client).size());
  // A different namespace is a different node.
  EXPECT_TRUE(cache.names_and_types_of_node("/ns", "a", EndpointKind::Publisher).empty());
}

TEST_F(GraphCacheTest, endpoints_for_topic_filters_by_kind)
{
  EXPECT_EQ(1u, cache.endpoints_for_topic("/chatter", EndpointKind::Publisher).size());
  EXPECT_EQ(1u, cache.endpoints_for_topic("/chatter", EndpointKind::Subscription).size());
  EXPECT_TRUE(cache.endpoints_for_topic("/nope", EndpointKind::Publisher).empty());
}

TEST_F(GraphCacheTest, find_service_returns_the_instance_and_endpoint)
{
  const auto found = cache.find_service("/add");
  EXPECT_EQ(1u, found.first);
  EXPECT_EQ("/add", found.second.topic_name);

  const auto missing = cache.find_service("/nope");
  EXPECT_EQ(0u, missing.first);
}

// -- Queries that consider QoS compatibility ----------------------------------

TEST_F(GraphCacheTest, matched_counts_only_compatible_peers)
{
  EXPECT_EQ(1u, cache.count_matched_subscriptions("/chatter", reliable_volatile()));
  EXPECT_EQ(1u, cache.count_matched_publishers("/chatter", reliable_volatile()));

  // Requesting TRANSIENT_LOCAL does not match a VOLATILE publisher.
  auto demanding = reliable_volatile();
  demanding.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  EXPECT_EQ(0u, cache.count_matched_publishers("/chatter", demanding));
}

TEST_F(GraphCacheTest, incompatible_counts_report_the_offending_policy)
{
  auto demanding = reliable_volatile();
  demanding.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  const auto publishers = cache.count_incompatible_publishers("/chatter", demanding);
  EXPECT_EQ(1u, publishers.count);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY, publishers.last_policy_kind);

  // A BEST_EFFORT publisher is incompatible with a RELIABLE subscription.
  auto best_effort = reliable_volatile();
  best_effort.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  const auto subscriptions = cache.count_incompatible_subscriptions("/chatter", best_effort);
  EXPECT_EQ(1u, subscriptions.count);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY, subscriptions.last_policy_kind);

  // Compatible: no count and no policy.
  const auto none = cache.count_incompatible_publishers("/chatter", reliable_volatile());
  EXPECT_EQ(0u, none.count);
  EXPECT_EQ(RMW_QOS_POLICY_INVALID, none.last_policy_kind);
}

TEST(GraphCache, matched_publisher_liveliness_returns_gid_and_qos)
{
  GraphCache cache;
  ParticipantEntitiesInfo info;
  info.instance_id = 1;
  auto publisher = make(EndpointKind::Publisher, "/t", "T");
  publisher.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  publisher.liveliness_lease_sec = 2;
  publisher.gid[0] = 0x42;
  info.endpoints.push_back(publisher);
  cache.update_participant(info);

  auto subscription_qos = reliable_volatile();
  subscription_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  const auto matched = cache.matched_publisher_liveliness("/t", subscription_qos);
  ASSERT_EQ(1u, matched.size());
  EXPECT_EQ(0x42, matched[0].first[0]);
  EXPECT_EQ(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, matched[0].second.liveliness);
  EXPECT_EQ(2u, matched[0].second.liveliness_lease_duration.sec);

  // Incompatible peers are excluded: AUTOMATIC publisher, MANUAL_BY_TOPIC
  // subscription.
  GraphCache automatic_cache;
  ParticipantEntitiesInfo automatic;
  automatic.instance_id = 1;
  automatic.endpoints.push_back(make(EndpointKind::Publisher, "/t", "T"));
  automatic_cache.update_participant(automatic);
  EXPECT_TRUE(automatic_cache.matched_publisher_liveliness("/t", subscription_qos).empty());
}

TEST(GraphCache, qos_of_restores_every_carried_field)
{
  EndpointInfo endpoint;
  endpoint.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  endpoint.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  endpoint.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  endpoint.depth = 42;
  endpoint.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  endpoint.deadline_sec = 1;
  endpoint.deadline_nsec = 2;
  endpoint.lifespan_sec = 3;
  endpoint.lifespan_nsec = 4;
  endpoint.liveliness_lease_sec = 5;
  endpoint.liveliness_lease_nsec = 6;

  const auto qos = GraphCache::qos_of(endpoint);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, qos.reliability);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, qos.durability);
  EXPECT_EQ(RMW_QOS_POLICY_HISTORY_KEEP_ALL, qos.history);
  EXPECT_EQ(42u, qos.depth);
  EXPECT_EQ(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, qos.liveliness);
  EXPECT_EQ(1u, qos.deadline.sec);
  EXPECT_EQ(2u, qos.deadline.nsec);
  EXPECT_EQ(3u, qos.lifespan.sec);
  EXPECT_EQ(4u, qos.lifespan.nsec);
  EXPECT_EQ(5u, qos.liveliness_lease_duration.sec);
  EXPECT_EQ(6u, qos.liveliness_lease_duration.nsec);
}
