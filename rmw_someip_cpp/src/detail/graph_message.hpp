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

#ifndef DETAIL__GRAPH_MESSAGE_HPP_
#define DETAIL__GRAPH_MESSAGE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rmw/types.h"
#include "rosidl_runtime_c/type_hash.h"

namespace rmw_someip_cpp
{

/// Kind of entity announced on the graph channel.
enum class EndpointKind : uint8_t
{
  Publisher = 0,
  Subscription = 1,
  Service = 2,
  Client = 3,
};

/// One ROS entity: publisher, subscription, service or client.
struct EndpointInfo
{
  EndpointKind kind {EndpointKind::Publisher};
  std::string node_namespace;
  std::string node_name;
  std::string topic_name;   ///< fully qualified ROS name
  std::string type_name;    ///< as "std_msgs/msg/String"

  /// REP-2011 (RIHS) type hash, carried because the name alone cannot tell
  /// apart two types that share it but differ in their fields. Returned
  /// verbatim by rmw_get_{publishers,subscriptions}_info_by_topic(), which is
  /// what `ros2 topic info -v` displays.
  rosidl_type_hash_t type_hash {};

  /// SOME/IP IDs assigned to this endpoint. Carrying these is the whole
  /// reason the graph channel exists; SOME/IP-SD has no room for them.
  uint16_t event_id {0};
  uint16_t eventgroup_id {0};
  uint16_t method_id {0};

  /// QoS. SOME/IP connects regardless of mismatches, so the RMW compares
  /// these itself.
  ///
  /// Every field is carried, not just the ones compatibility depends on:
  /// rmw_get_publishers_info_by_topic() and friends are contractually required
  /// to return the actual QoS, and rcl resolves BEST_AVAILABLE from it.
  uint8_t reliability {0};
  uint8_t durability {0};
  uint8_t history {0};
  uint32_t depth {0};
  uint8_t liveliness {0};
  int64_t deadline_sec {0};
  uint64_t deadline_nsec {0};
  int64_t lifespan_sec {0};
  uint64_t lifespan_nsec {0};
  int64_t liveliness_lease_sec {0};
  uint64_t liveliness_lease_nsec {0};

  uint8_t gid[RMW_GID_STORAGE_SIZE] {};
};

/// The ROS graph published by one participant (one process, one instance).
struct ParticipantEntitiesInfo
{
  uint16_t instance_id {0};

  /// Sender's 64-bit fingerprint (ParticipantIdentity::guid).
  ///
  /// instance_id is only 16 bits and can collide. A receiver treats "same
  /// instance_id, different guid" as a collision; without this it would take
  /// the other participant's graph for its own echo and silently drop it.
  uint64_t guid {0};

  /// Nodes are listed separately so that one without any entity still shows
  /// up in `ros2 node list`.
  struct NodeInfo
  {
    std::string node_namespace;
    std::string node_name;
    std::string enclave;
  };
  std::vector<NodeInfo> nodes;
  std::vector<EndpointInfo> endpoints;
};

/// Encode a graph channel payload.
///
/// The format is internal to this RMW and never reaches a non-ROS SOME/IP peer
/// (those go through extern mode's static mapping), so it is a compact binary
/// format rather than a rosidl message, which would mean another package. A
/// magic number and version at the front make format changes detectable.
std::vector<uint8_t> encode_graph_message(const ParticipantEntitiesInfo & info);

/// Decode a graph channel payload.
/// \return false on failure, with a message in `error`
bool decode_graph_message(
  const uint8_t * data,
  size_t size,
  ParticipantEntitiesInfo & out,
  std::string & error);

}  // namespace rmw_someip_cpp

#endif  // DETAIL__GRAPH_MESSAGE_HPP_
