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

#ifndef DETAIL__GRAPH_CACHE_HPP_
#define DETAIL__GRAPH_CACHE_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rmw/qos_profiles.h"
#include "rmw/types.h"

#include "detail/graph_message.hpp"
#include "detail/qos_compat.hpp"

namespace rmw_someip_cpp
{

/// The ROS graph received from other participants over the graph channel.
///
/// SOME/IP-SD carries only that a participant exists and how to reach it, so
/// node names, topic names, type names, QoS and the assigned event/eventgroup
/// IDs all end up here. The rmw graph introspection API is implemented as
/// queries against this cache.
class GraphCache
{
public:
  /// Replace a participant's entry wholesale. The graph channel always sends
  /// the complete state rather than a delta, so replacing is enough.
  /// \return true if the graph actually changed
  bool update_participant(ParticipantEntitiesInfo info);

  /// A participant went away (SD availability=false).
  /// \return true if the graph actually changed
  bool remove_participant(uint16_t instance_id);

  /// Remote endpoints publishing on this topic.
  /// \param[in] exclude_instance this participant, which must not match itself
  std::vector<EndpointInfo> find_publishers(
    const std::string & topic_name,
    uint16_t exclude_instance) const;

  /// Every publisher, paired with its instance_id.
  std::vector<std::pair<uint16_t, EndpointInfo>> all_publishers(
    uint16_t exclude_instance) const;

  /// Topic name to type names, for rmw_get_topic_names_and_types.
  std::map<std::string, std::vector<std::string>> topic_names_and_types(
    bool include_publishers,
    bool include_subscriptions) const;

  /// Same, restricted to one node's endpoints.
  /// For rmw_get_*_names_and_types_by_node, which backs `ros2 node info`.
  std::map<std::string, std::vector<std::string>> names_and_types_of_node(
    const std::string & node_namespace,
    const std::string & node_name,
    EndpointKind kind) const;

  /// Is this node on the graph? Queries about a node that is not have to
  /// return a distinct error code, so this has to be answerable.
  bool has_node(
    const std::string & node_namespace,
    const std::string & node_name) const;

  /// All nodes, for rmw_get_node_names.
  std::vector<ParticipantEntitiesInfo::NodeInfo> node_names() const;

  /// Publishers and subscriptions on a topic, regardless of QoS compatibility.
  size_t count_publishers(const std::string & topic_name) const;
  size_t count_subscriptions(const std::string & topic_name) const;

  /// Count only QoS-compatible peers.
  ///
  /// SOME/IP connects regardless of mismatches, so answering "how many are
  /// matched" means deciding compatibility here.
  /// Backs rmw_publisher_count_matched_subscriptions() and friends.
  size_t count_matched_subscriptions(
    const std::string & topic_name,
    const rmw_qos_profile_t & publisher_qos) const;
  size_t count_matched_publishers(
    const std::string & topic_name,
    const rmw_qos_profile_t & subscription_qos) const;

  /// Peers rejected as incompatible, and the last offending policy.
  /// This is what the *_QOS_INCOMPATIBLE events report.
  struct IncompatibleCount
  {
    size_t count {0};
    rmw_qos_policy_kind_t last_policy_kind {RMW_QOS_POLICY_INVALID};
  };
  IncompatibleCount count_incompatible_subscriptions(
    const std::string & topic_name,
    const rmw_qos_profile_t & publisher_qos) const;
  IncompatibleCount count_incompatible_publishers(
    const std::string & topic_name,
    const rmw_qos_profile_t & subscription_qos) const;

  /// Matching publishers as (gid, QoS), used to track MANUAL_BY_TOPIC liveliness.
  std::vector<std::pair<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, rmw_qos_profile_t>>
  matched_publisher_liveliness(
    const std::string & topic_name,
    const rmw_qos_profile_t & subscription_qos) const;

  /// Endpoints of one kind on a topic, for
  /// rmw_get_{publishers,subscriptions}_info_by_topic.
  std::vector<EndpointInfo> endpoints_for_topic(
    const std::string & topic_name,
    EndpointKind kind) const;

  /// Turn the QoS carried on the graph channel back into an rmw_qos_profile_t.
  static rmw_qos_profile_t qos_of(const EndpointInfo & endpoint);

  /// Find a service server by name.
  /// \return (instance_id, endpoint), with instance_id 0 when not found
  std::pair<uint16_t, EndpointInfo> find_service(
    const std::string & service_name) const;

  /// Service name to type names, for rmw_get_service_names_and_types.
  std::map<std::string, std::vector<std::string>> service_names_and_types(
    bool include_servers,
    bool include_clients) const;

  /// Servers and clients on a service name.
  size_t count_services(const std::string & service_name) const;
  size_t count_clients(const std::string & service_name) const;

private:
  mutable std::mutex mutex_;
  std::map<uint16_t, ParticipantEntitiesInfo> participants_;
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__GRAPH_CACHE_HPP_
