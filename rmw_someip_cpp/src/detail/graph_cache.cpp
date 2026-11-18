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

#include "detail/graph_cache.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rmw/qos_profiles.h"
#include "rmw/rmw.h"

namespace rmw_someip_cpp
{

namespace
{

bool same_endpoints(
  const std::vector<EndpointInfo> & a,
  const std::vector<EndpointInfo> & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].kind != b[i].kind ||
      a[i].topic_name != b[i].topic_name ||
      a[i].type_name != b[i].type_name ||
      a[i].event_id != b[i].event_id ||
      a[i].eventgroup_id != b[i].eventgroup_id ||
      a[i].node_name != b[i].node_name ||
      a[i].node_namespace != b[i].node_namespace)
    {
      return false;
    }
  }
  return true;
}

/// Rebuild the part of a QoS profile the graph channel carries.
/// Compatibility only depends on reliability, durability and liveliness, so
/// the fields that are not carried keep their defaults.
rmw_qos_profile_t to_qos_profile(const EndpointInfo & endpoint)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = static_cast<rmw_qos_reliability_policy_t>(endpoint.reliability);
  qos.durability = static_cast<rmw_qos_durability_policy_t>(endpoint.durability);
  qos.history = static_cast<rmw_qos_history_policy_t>(endpoint.history);
  qos.depth = endpoint.depth;
  qos.liveliness = static_cast<rmw_qos_liveliness_policy_t>(endpoint.liveliness);
  qos.deadline.sec = static_cast<uint64_t>(endpoint.deadline_sec);
  qos.deadline.nsec = endpoint.deadline_nsec;
  qos.lifespan.sec = static_cast<uint64_t>(endpoint.lifespan_sec);
  qos.lifespan.nsec = endpoint.lifespan_nsec;
  qos.liveliness_lease_duration.sec = static_cast<uint64_t>(endpoint.liveliness_lease_sec);
  qos.liveliness_lease_duration.nsec = endpoint.liveliness_lease_nsec;
  return qos;
}

bool is_compatible(
  const rmw_qos_profile_t & publisher_qos,
  const rmw_qos_profile_t & subscription_qos)
{
  // The rule lives in detail/qos_compat.hpp rather than in
  // rmw_qos_profile_check_compatible(), which collapses WARNING and ERROR into
  // one enum and never names the offending policy, which the
  // *_QOS_INCOMPATIBLE events need.
  return !check_incompatible(publisher_qos, subscription_qos).incompatible();
}

/// Visit every endpoint of every participant.
///
/// Every query on this class has the same shape: walk all participants, walk
/// all their endpoints, count or collect the ones that match. Written out,
/// that is the same nested loop many times over, with the filter — the
/// only interesting part — buried inside. The traversal lives here so each
/// query only says what to do with an endpoint.
///
/// Call with mutex_ held.
template<typename Fn>
void for_each_endpoint(
  const std::map<uint16_t, ParticipantEntitiesInfo> & participants, Fn && fn)
{
  for (const auto & [instance_id, participant] : participants) {
    for (const auto & endpoint : participant.endpoints) {
      fn(instance_id, endpoint);
    }
  }
}

/// Count endpoints matching a kind and a name.
size_t count_matching(
  const std::map<uint16_t, ParticipantEntitiesInfo> & participants,
  const std::string & name,
  EndpointKind kind)
{
  size_t count = 0;
  for_each_endpoint(
    participants, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind == kind && endpoint.topic_name == name) {
        ++count;
      }
    });
  return count;
}

/// Add one entry to a name-to-types map, ignoring duplicates.
void add_name_and_type(
  std::map<std::string, std::vector<std::string>> & result,
  const EndpointInfo & endpoint)
{
  auto & types = result[endpoint.topic_name];
  if (std::find(types.begin(), types.end(), endpoint.type_name) == types.end()) {
    types.push_back(endpoint.type_name);
  }
}

}  // namespace

bool GraphCache::update_participant(ParticipantEntitiesInfo info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const uint16_t instance_id = info.instance_id;

  auto it = participants_.find(instance_id);
  if (it != participants_.end() &&
    it->second.nodes.size() == info.nodes.size() &&
    same_endpoints(it->second.endpoints, info.endpoints))
  {
    // The graph channel repeats the same contents. Leaving the guard
    // condition alone when nothing changed keeps the executor from spinning.
    return false;
  }

  participants_[instance_id] = std::move(info);
  return true;
}

bool GraphCache::remove_participant(uint16_t instance_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return participants_.erase(instance_id) > 0;
}

std::vector<EndpointInfo> GraphCache::find_publishers(
  const std::string & topic_name,
  uint16_t exclude_instance) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<EndpointInfo> result;
  for_each_endpoint(
    participants_, [&](uint16_t instance_id, const EndpointInfo & endpoint) {
      if (instance_id != exclude_instance &&
      endpoint.kind == EndpointKind::Publisher && endpoint.topic_name == topic_name)
      {
        result.push_back(endpoint);
      }
    });
  return result;
}

std::vector<std::pair<uint16_t, EndpointInfo>> GraphCache::all_publishers(
  uint16_t exclude_instance) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::pair<uint16_t, EndpointInfo>> result;
  for_each_endpoint(
    participants_, [&](uint16_t instance_id, const EndpointInfo & endpoint) {
      if (instance_id != exclude_instance && endpoint.kind == EndpointKind::Publisher) {
        result.emplace_back(instance_id, endpoint);
      }
    });
  return result;
}

std::map<std::string, std::vector<std::string>> GraphCache::topic_names_and_types(
  bool include_publishers,
  bool include_subscriptions) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, std::vector<std::string>> result;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      const bool wanted =
      (include_publishers && endpoint.kind == EndpointKind::Publisher) ||
      (include_subscriptions && endpoint.kind == EndpointKind::Subscription);
      if (wanted) {
        add_name_and_type(result, endpoint);
      }
    });
  return result;
}

std::map<std::string, std::vector<std::string>> GraphCache::names_and_types_of_node(
  const std::string & node_namespace,
  const std::string & node_name,
  EndpointKind kind) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, std::vector<std::string>> result;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind == kind &&
      endpoint.node_name == node_name &&
      endpoint.node_namespace == node_namespace)
      {
        add_name_and_type(result, endpoint);
      }
    });
  return result;
}

bool GraphCache::has_node(
  const std::string & node_namespace,
  const std::string & node_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [instance_id, participant] : participants_) {
    (void)instance_id;
    for (const auto & node : participant.nodes) {
      if (node.node_name == node_name && node.node_namespace == node_namespace) {
        return true;
      }
    }
  }
  return false;
}

std::vector<ParticipantEntitiesInfo::NodeInfo> GraphCache::node_names() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ParticipantEntitiesInfo::NodeInfo> result;
  for (const auto & [instance_id, participant] : participants_) {
    (void)instance_id;
    for (const auto & node : participant.nodes) {
      result.push_back(node);
    }
  }
  return result;
}

size_t GraphCache::count_publishers(const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return count_matching(participants_, topic_name, EndpointKind::Publisher);
}

size_t GraphCache::count_subscriptions(const std::string & topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return count_matching(participants_, topic_name, EndpointKind::Subscription);
}

std::vector<EndpointInfo> GraphCache::endpoints_for_topic(
  const std::string & topic_name,
  EndpointKind kind) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<EndpointInfo> result;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind == kind && endpoint.topic_name == topic_name) {
        result.push_back(endpoint);
      }
    });
  return result;
}

rmw_qos_profile_t GraphCache::qos_of(const EndpointInfo & endpoint)
{
  return to_qos_profile(endpoint);
}

size_t GraphCache::count_matched_subscriptions(
  const std::string & topic_name,
  const rmw_qos_profile_t & publisher_qos) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t count = 0;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind == EndpointKind::Subscription &&
      endpoint.topic_name == topic_name &&
      is_compatible(publisher_qos, to_qos_profile(endpoint)))
      {
        ++count;
      }
    });
  return count;
}

size_t GraphCache::count_matched_publishers(
  const std::string & topic_name,
  const rmw_qos_profile_t & subscription_qos) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t count = 0;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind == EndpointKind::Publisher &&
      endpoint.topic_name == topic_name &&
      is_compatible(to_qos_profile(endpoint), subscription_qos))
      {
        ++count;
      }
    });
  return count;
}

GraphCache::IncompatibleCount GraphCache::count_incompatible_subscriptions(
  const std::string & topic_name,
  const rmw_qos_profile_t & publisher_qos) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  IncompatibleCount result;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind != EndpointKind::Subscription || endpoint.topic_name != topic_name) {
        return;
      }
      const auto incompatibility = check_incompatible(publisher_qos, to_qos_profile(endpoint));
      if (incompatibility.incompatible()) {
        ++result.count;
        result.last_policy_kind = incompatibility.policy_kind;
      }
    });
  return result;
}

GraphCache::IncompatibleCount GraphCache::count_incompatible_publishers(
  const std::string & topic_name,
  const rmw_qos_profile_t & subscription_qos) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  IncompatibleCount result;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind != EndpointKind::Publisher || endpoint.topic_name != topic_name) {
        return;
      }
      const auto incompatibility = check_incompatible(to_qos_profile(endpoint), subscription_qos);
      if (incompatibility.incompatible()) {
        ++result.count;
        result.last_policy_kind = incompatibility.policy_kind;
      }
    });
  return result;
}

std::vector<std::pair<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, rmw_qos_profile_t>>
GraphCache::matched_publisher_liveliness(
  const std::string & topic_name,
  const rmw_qos_profile_t & subscription_qos) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::pair<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, rmw_qos_profile_t>> result;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      if (endpoint.kind != EndpointKind::Publisher || endpoint.topic_name != topic_name) {
        return;
      }
      const auto qos = to_qos_profile(endpoint);
      if (!is_compatible(qos, subscription_qos)) {
        return;
      }
      std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid{};
      std::memcpy(gid.data(), endpoint.gid, RMW_GID_STORAGE_SIZE);
      result.emplace_back(gid, qos);
    });
  return result;
}

std::pair<uint16_t, EndpointInfo> GraphCache::find_service(
  const std::string & service_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Returns the first match, so this walks by hand rather than using
  // for_each_endpoint, which cannot break out early.
  for (const auto & [instance_id, participant] : participants_) {
    for (const auto & endpoint : participant.endpoints) {
      if (endpoint.kind == EndpointKind::Service && endpoint.topic_name == service_name) {
        return {instance_id, endpoint};
      }
    }
  }
  return {0, EndpointInfo{}};
}

std::map<std::string, std::vector<std::string>> GraphCache::service_names_and_types(
  bool include_servers,
  bool include_clients) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, std::vector<std::string>> result;
  for_each_endpoint(
    participants_, [&](uint16_t, const EndpointInfo & endpoint) {
      const bool wanted =
      (include_servers && endpoint.kind == EndpointKind::Service) ||
      (include_clients && endpoint.kind == EndpointKind::Client);
      if (wanted) {
        add_name_and_type(result, endpoint);
      }
    });
  return result;
}

size_t GraphCache::count_services(const std::string & service_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return count_matching(participants_, service_name, EndpointKind::Service);
}

size_t GraphCache::count_clients(const std::string & service_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return count_matching(participants_, service_name, EndpointKind::Client);
}

}  // namespace rmw_someip_cpp
