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
// Graph channel encoding.
//
// The format is internal to this RMW, but it still arrives from the
// network, so malformed or hostile input must not crash it or force a huge
// allocation.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "detail/graph_message.hpp"

namespace
{

rmw_someip_cpp::ParticipantEntitiesInfo make_info()
{
  rmw_someip_cpp::ParticipantEntitiesInfo info;
  info.instance_id = 0x1234;
  info.guid = 0x0123456789ABCDEFULL;
  info.nodes.push_back({"/ns", "node_a", "/enclave"});
  info.nodes.push_back({"/", "node_b", ""});

  rmw_someip_cpp::EndpointInfo publisher;
  publisher.kind = rmw_someip_cpp::EndpointKind::Publisher;
  publisher.node_namespace = "/ns";
  publisher.node_name = "node_a";
  publisher.topic_name = "/chatter";
  publisher.type_name = "std_msgs/msg/String";
  publisher.type_hash.version = 1;
  publisher.type_hash.value[0] = 0xAB;
  publisher.type_hash.value[31] = 0xCD;
  publisher.event_id = 0x8001;
  publisher.eventgroup_id = 0x0002;
  publisher.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  publisher.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  publisher.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  publisher.depth = 7;
  publisher.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  publisher.deadline_sec = 1;
  publisher.deadline_nsec = 2;
  publisher.lifespan_sec = 3;
  publisher.lifespan_nsec = 4;
  publisher.liveliness_lease_sec = 5;
  publisher.liveliness_lease_nsec = 6;
  publisher.gid[0] = 0x11;
  publisher.gid[15] = 0x22;
  info.endpoints.push_back(publisher);

  rmw_someip_cpp::EndpointInfo service;
  service.kind = rmw_someip_cpp::EndpointKind::Service;
  service.topic_name = "/add_two_ints";
  service.type_name = "example_interfaces/srv/AddTwoInts";
  service.method_id = 0x0001;
  info.endpoints.push_back(service);

  return info;
}

}  // namespace

TEST(GraphMessage, round_trips_every_field)
{
  const auto original = make_info();
  const auto encoded = rmw_someip_cpp::encode_graph_message(original);

  rmw_someip_cpp::ParticipantEntitiesInfo decoded;
  std::string error;
  ASSERT_TRUE(
    rmw_someip_cpp::decode_graph_message(
      encoded.data(), encoded.size(), decoded, error)) << error;

  EXPECT_EQ(original.instance_id, decoded.instance_id);
  EXPECT_EQ(original.guid, decoded.guid);
  ASSERT_EQ(original.nodes.size(), decoded.nodes.size());
  EXPECT_EQ("node_a", decoded.nodes[0].node_name);
  EXPECT_EQ("/enclave", decoded.nodes[0].enclave);
  EXPECT_EQ("", decoded.nodes[1].enclave);

  ASSERT_EQ(original.endpoints.size(), decoded.endpoints.size());
  const auto & publisher = decoded.endpoints[0];
  EXPECT_EQ(rmw_someip_cpp::EndpointKind::Publisher, publisher.kind);
  EXPECT_EQ("/chatter", publisher.topic_name);
  EXPECT_EQ("std_msgs/msg/String", publisher.type_name);
  EXPECT_EQ(1, publisher.type_hash.version);
  EXPECT_EQ(0xAB, publisher.type_hash.value[0]);
  EXPECT_EQ(0xCD, publisher.type_hash.value[31]);
  EXPECT_EQ(0x8001, publisher.event_id);
  EXPECT_EQ(0x0002, publisher.eventgroup_id);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, publisher.reliability);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, publisher.durability);
  EXPECT_EQ(7u, publisher.depth);
  EXPECT_EQ(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC, publisher.liveliness);
  EXPECT_EQ(1, publisher.deadline_sec);
  EXPECT_EQ(6u, publisher.liveliness_lease_nsec);
  EXPECT_EQ(0x11, publisher.gid[0]);
  EXPECT_EQ(0x22, publisher.gid[15]);

  EXPECT_EQ(rmw_someip_cpp::EndpointKind::Service, decoded.endpoints[1].kind);
  EXPECT_EQ(0x0001, decoded.endpoints[1].method_id);
}

TEST(GraphMessage, round_trips_an_empty_participant)
{
  rmw_someip_cpp::ParticipantEntitiesInfo original;
  original.instance_id = 1;
  const auto encoded = rmw_someip_cpp::encode_graph_message(original);

  rmw_someip_cpp::ParticipantEntitiesInfo decoded;
  std::string error;
  ASSERT_TRUE(
    rmw_someip_cpp::decode_graph_message(encoded.data(), encoded.size(), decoded, error)) << error;
  EXPECT_TRUE(decoded.nodes.empty());
  EXPECT_TRUE(decoded.endpoints.empty());
}

TEST(GraphMessage, rejects_a_bad_magic_number)
{
  auto encoded = rmw_someip_cpp::encode_graph_message(make_info());
  encoded[0] ^= 0xFF;

  rmw_someip_cpp::ParticipantEntitiesInfo decoded;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::decode_graph_message(encoded.data(), encoded.size(), decoded, error));
  EXPECT_NE(std::string::npos, error.find("magic"));
}

/// A changed format is rejected rather than silently misread.
TEST(GraphMessage, rejects_an_unsupported_version)
{
  auto encoded = rmw_someip_cpp::encode_graph_message(make_info());
  encoded[4] = 0x7F;  // the version field

  rmw_someip_cpp::ParticipantEntitiesInfo decoded;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::decode_graph_message(encoded.data(), encoded.size(), decoded, error));
  EXPECT_NE(std::string::npos, error.find("not supported"));
}

/// Truncation at any point must not crash.
TEST(GraphMessage, rejects_any_truncation)
{
  const auto encoded = rmw_someip_cpp::encode_graph_message(make_info());
  for (size_t length = 0; length < encoded.size(); ++length) {
    rmw_someip_cpp::ParticipantEntitiesInfo decoded;
    std::string error;
    EXPECT_FALSE(
      rmw_someip_cpp::decode_graph_message(encoded.data(), length, decoded, error))
      << "unexpectedly succeeded at length=" << length;
    EXPECT_FALSE(error.empty()) << "length=" << length;
  }
}

/// A malformed or hostile count must not force a huge allocation.
TEST(GraphMessage, rejects_an_absurd_node_count)
{
  rmw_someip_cpp::ParticipantEntitiesInfo empty;
  empty.instance_id = 1;
  auto encoded = rmw_someip_cpp::encode_graph_message(empty);
  // The node count starts at byte 14: magic(4) + version(2) + instance(2) + guid(8).
  ASSERT_GE(encoded.size(), 18u);
  const uint32_t huge = 0xFFFFFFFFu;
  std::memcpy(encoded.data() + 16, &huge, sizeof(huge));

  rmw_someip_cpp::ParticipantEntitiesInfo decoded;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::decode_graph_message(encoded.data(), encoded.size(), decoded, error));
  EXPECT_NE(std::string::npos, error.find("invalid node count"));
}

TEST(GraphMessage, rejects_an_unknown_endpoint_kind)
{
  rmw_someip_cpp::ParticipantEntitiesInfo info;
  info.instance_id = 1;
  rmw_someip_cpp::EndpointInfo endpoint;
  endpoint.kind = rmw_someip_cpp::EndpointKind::Publisher;
  info.endpoints.push_back(endpoint);
  auto encoded = rmw_someip_cpp::encode_graph_message(info);

  // Corrupt the endpoint kind byte at offset 24:
  // magic(4) + version(2) + instance(2) + guid(8) + node_count(4) + endpoint_count(4).
  ASSERT_GT(encoded.size(), 24u);
  encoded[24] = 0x7F;

  rmw_someip_cpp::ParticipantEntitiesInfo decoded;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::decode_graph_message(encoded.data(), encoded.size(), decoded, error));
  EXPECT_NE(std::string::npos, error.find("unknown endpoint kind"));
}
