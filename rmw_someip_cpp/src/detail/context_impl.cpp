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
// The graph channel.
//
// SOME/IP-SD can only carry that a participant exists and where to reach it,
// so everything ROS-specific — node names, topic names, type names, QoS, the
// assigned event IDs — travels here instead. See docs/design.md §5.2.

#include "detail/context_impl.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <deque>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"

#include "detail/graph_message.hpp"
#include "detail/node_data.hpp"
#include "rmw_someip_cpp/identifier.hpp"

namespace
{

/// Copy a QoS profile into a graph channel endpoint.
void fill_qos(rmw_someip_cpp::EndpointInfo & endpoint, const rmw_qos_profile_t & qos)
{
  endpoint.reliability = static_cast<uint8_t>(qos.reliability);
  endpoint.durability = static_cast<uint8_t>(qos.durability);
  endpoint.history = static_cast<uint8_t>(qos.history);
  endpoint.depth = static_cast<uint32_t>(qos.depth);
  endpoint.liveliness = static_cast<uint8_t>(qos.liveliness);
  endpoint.deadline_sec = qos.deadline.sec;
  endpoint.deadline_nsec = qos.deadline.nsec;
  endpoint.lifespan_sec = qos.lifespan.sec;
  endpoint.lifespan_nsec = qos.lifespan.nsec;
  endpoint.liveliness_lease_sec = qos.liveliness_lease_duration.sec;
  endpoint.liveliness_lease_nsec = qos.liveliness_lease_duration.nsec;
}

/// Fill in the parts of an endpoint that every kind shares.
///
/// The same assignments repeated at four call sites means a new field gets
/// forgotten at one of them — adding type_hash required touching all four.
/// With the common part here, only the differences remain at the call sites.
rmw_someip_cpp::EndpointInfo make_endpoint(
  rmw_someip_cpp::EndpointKind kind,
  const std::string & node_namespace,
  const std::string & node_name,
  const std::string & name,
  const std::string & type_name,
  const rosidl_type_hash_t & type_hash)
{
  rmw_someip_cpp::EndpointInfo endpoint;
  endpoint.kind = kind;
  endpoint.node_namespace = node_namespace;
  endpoint.node_name = node_name;
  endpoint.topic_name = name;
  endpoint.type_name = type_name;
  endpoint.type_hash = type_hash;
  return endpoint;
}

int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/// Should this be offered as a field, given durability and depth?
///
/// TRANSIENT_LOCAL at depth 1 maps onto a SOME/IP field exactly: a field
/// delivers its latest value on subscribe, natively in vsomeip.
///
/// Depth above 1 cannot be a field. A field holds one value and has no
/// notion of history, so those are offered as ET_EVENT and replayed to late
/// subscribers with notify_one() from the RMW's own buffer (docs/design.md
/// §21). Sender and subscriber have to agree, so the rule lives in this one
/// function.
bool is_field_for(uint8_t durability, uint32_t depth)
{
  const bool transient_local =
    durability == static_cast<uint8_t>(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  return transient_local && depth <= 1;
}

bool is_field_for(const rmw_qos_profile_t & qos)
{
  return is_field_for(
    static_cast<uint8_t>(qos.durability), static_cast<uint32_t>(qos.depth));
}

/// RELIABLE maps to TCP, BEST_EFFORT to UDP.
bool is_reliable(const rmw_qos_profile_t & qos)
{
  return qos.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
}

}  // namespace

rmw_context_impl_s::rmw_context_impl_s(
  size_t domain_id,
  rmw_someip_cpp::ParticipantIdentity identity,
  std::unique_ptr<rmw_someip_cpp::SomeipEndpoint> session)
: domain_id(domain_id),
  identity(std::move(identity)),
  session(std::move(session))
{
}

rmw_context_impl_s::~rmw_context_impl_s()
{
  // The watchdog touches publishers, subscriptions and their EventData, so
  // stop and join it before destroying any of them.
  stop_qos_watchdog();

  // Then stop the vsomeip application.
  //
  // vsomeip callbacks such as on_someip_message touch graph_cache and routes.
  // Members are destroyed in reverse declaration order, so leaving this to the
  // implicit destructor would tear down graph_cache before session and let a
  // dispatch thread reach a destroyed object.
  //
  // SomeipSession's destructor does clear_all_handler(), stop() and join(), so
  // no callback is running once this returns.
  session.reset();
}

bool rmw_context_impl_s::start_graph_channel(std::string & error)
{
  if (graph_channel_started_) {
    return true;
  }
  auto * endpoint = this->endpoint();
  if (endpoint == nullptr) {
    error = "the SOME/IP endpoint is not available";
    return false;
  }

  const auto service_id = identity.service_id;
  const auto own_instance = identity.instance_id;

  // Offer this participant's service. It is declared with a port in the
  // generated JSON, so an SD offer actually goes out (docs/design.md §3).
  endpoint->offer_service(service_id, own_instance);

  // ET_FIELD, so a participant that starts later receives the current graph
  // the moment it subscribes and cannot miss it.
  //
  // Over TCP (RT_RELIABLE): SOME/IP's UDP has a maximum message length (1416
  // bytes by default) and the graph exceeds it with only a few nodes, since
  // each carries six parameter services plus get_type_description with their
  // names and type names. A dropped graph means a peer is never discovered.
  endpoint->offer_event(
    service_id, own_instance, rmw_someip_cpp::kGraphEventId,
    rmw_someip_cpp::kGraphEventgroupId, true, true);

  // One handler for every service, instance and event. Restricting it to this
  // participant's service would miss extern-mode topics, which use another.
  endpoint->on_message(
    [this](const std::shared_ptr<vsomeip::message> & message) {
      on_someip_message(message);
    });

  endpoint->on_availability(
    service_id,
    [this](uint16_t instance, bool available) {on_availability(instance, available);});

  // A single request with a wildcard instance discovers every participant.
  endpoint->request_service(service_id, vsomeip::ANY_INSTANCE);

  graph_channel_started_ = true;
  publish_local_graph();
  return true;
}

void rmw_context_impl_s::on_availability(uint16_t instance_id, bool available)
{
  if (instance_id == identity.instance_id) {
    return;  // this participant
  }

  if (!available) {
    if (graph_cache.remove_participant(instance_id)) {
      notify_graph_changed();
    }
    return;
  }

  // Subscribe to the peer's graph channel. Being a field, it delivers the
  // peer's current graph as soon as the subscription is in place.
  auto * endpoint = this->endpoint();
  if (endpoint == nullptr) {
    return;
  }
  endpoint->request_event(
    identity.service_id, instance_id, rmw_someip_cpp::kGraphEventId,
    rmw_someip_cpp::kGraphEventgroupId, true, true);
  endpoint->subscribe(
    identity.service_id, instance_id, rmw_someip_cpp::kGraphEventgroupId);
}

void rmw_context_impl_s::report_instance_id_collision(uint64_t other_guid)
{
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    if (!reported_collisions_.insert(other_guid).second) {
      return;
    }
  }
  RCUTILS_LOG_ERROR_NAMED(
    "rmw_someip_cpp",
    "SOME/IP instance id 0x%04X collides with another participant "
    "(ours guid=0x%016" PRIx64 ", theirs guid=0x%016" PRIx64 "). "
    "The colliding participant will not be discovered. "
    "Restart one of the processes, or split them across ROS domains "
    "(ROS_DOMAIN_ID) or hosts.",
    identity.instance_id, identity.guid, other_guid);
}

void rmw_context_impl_s::on_someip_message(
  const std::shared_ptr<vsomeip::message> & message)
{
  auto payload = message->get_payload();
  if (!payload) {
    return;
  }

  // SOME/IP distinguishes events (NOTIFICATION) from methods
  // (REQUEST/RESPONSE) by message type, which is exactly the topic/service
  // split on the ROS side.
  const auto message_type = message->get_message_type();
  if (message_type == vsomeip::message_type_e::MT_REQUEST ||
    message_type == vsomeip::message_type_e::MT_REQUEST_NO_RETURN)
  {
    on_someip_request(message);
    return;
  }
  if (message_type == vsomeip::message_type_e::MT_RESPONSE ||
    message_type == vsomeip::message_type_e::MT_ERROR)
  {
    on_someip_response(message);
    return;
  }

  const auto service_id = static_cast<uint16_t>(message->get_service());
  const auto event_id = static_cast<uint16_t>(message->get_method());
  if (service_id == identity.service_id && event_id == rmw_someip_cpp::kGraphEventId) {
    on_graph_message(static_cast<uint16_t>(message->get_instance()), *payload);
    return;
  }
  on_topic_message(
    service_id, static_cast<uint16_t>(message->get_instance()), event_id, *payload);
}

void rmw_context_impl_s::on_graph_message(
  uint16_t instance_id,
  const vsomeip::payload & payload)
{
  rmw_someip_cpp::ParticipantEntitiesInfo info;
  std::string error;
  if (!rmw_someip_cpp::decode_graph_message(
      payload.get_data(), payload.get_length(), info, error))
  {
    // A malformed graph message must not bring the process down.
    return;
  }

  if (instance_id == identity.instance_id) {
    if (info.guid == identity.guid) {
      return;  // this participant's own graph, echoed back
    }
    // instance_id collision. It is 16 bits of hash(hostname, pid), so two
    // participants can share one. Taking the message for this participant's
    // own echo on instance_id alone would silently drop the other's graph.
    //
    // vsomeip normally rejects the colliding offer during discovery, before
    // anything reaches here; this branch reports the ones that get through.
    // There is no automatic renumbering: instance_id has already been offered,
    // and redoing discovery would lose more messages than the collision does
    // (docs/design.md §20).
    report_instance_id_collision(info.guid);
    return;
  }

  if (graph_cache.update_participant(std::move(info))) {
    match_subscriptions();
    notify_graph_changed();
  }
}

void rmw_context_impl_s::on_topic_message(
  uint16_t service_id,
  uint16_t instance_id,
  uint16_t event_id,
  const vsomeip::payload & payload)
{
  // The gid follows from (instance_id, event_id). Rebuilding it by the
  // sender's rule is what makes rmw_compare_gids_equal() match.
  const rmw_gid_t publisher_gid = rmw_someip_cpp::make_entity_gid(instance_id, event_id);

  std::vector<rmw_someip_cpp::SubscriptionData *> targets;
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    auto it = routes.find({service_id, instance_id, event_id});
    if (it == routes.end()) {
      return;
    }
    targets.assign(it->second.begin(), it->second.end());
  }

  // A zero-length payload is a MANUAL_BY_TOPIC liveliness assertion from
  // rmw_publisher_assert_liveliness(). CDR always starts with a 4-byte
  // encapsulation header, so no ordinary native-mode message is empty.
  //
  // In extern mode zero length is a legitimate payload — the AUTOSAR
  // encoding of a message with no fields is zero bytes. A non-ROS peer knows
  // nothing about liveliness assertions, so extern subscriptions skip this.
  if (payload.get_length() == 0) {
    const int64_t seen_at = rmw_someip_cpp::steady_now_ns();
    for (auto * target : targets) {
      if (!target->is_extern) {
        target->note_publisher_alive(publisher_gid, seen_at);
      }
    }
    return;
  }

  rmw_someip_cpp::ReceivedMessage received;
  received.payload.assign(
    payload.get_data(), payload.get_data() + payload.get_length());
  received.received_timestamp_ns = now_ns();
  received.publisher_gid = publisher_gid;

  // push() takes the subscription's mutex and then the wait set's. Releasing
  // entities_mutex first keeps the lock order simple.
  for (size_t i = 0; i < targets.size(); ++i) {
    if (i + 1 == targets.size()) {
      targets[i]->push(std::move(received));
    } else {
      targets[i]->push(received);
    }
  }
}

void rmw_context_impl_s::match_subscriptions()
{
  auto * endpoint = this->endpoint();
  if (endpoint == nullptr) {
    return;
  }

  // Collect the (instance, eventgroup, event) triples to subscribe to.
  struct Pending
  {
    uint16_t instance_id;
    uint16_t event_id;
    uint16_t eventgroup_id;
    rmw_someip_cpp::SubscriptionData * subscription;
    bool is_field;
    bool reliable;
  };
  std::vector<Pending> pending;

  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    const auto remote = graph_cache.all_publishers(identity.instance_id);
    for (auto * subscription : subscriptions) {
      if (subscription->is_extern) {
        continue;  // extern mode is already connected by its pinned IDs
      }
      for (const auto & [instance_id, endpoint] : remote) {
        if (endpoint.topic_name != subscription->topic_name) {
          continue;
        }
        if (endpoint.type_name != subscription->type_name) {
          // Do not connect topics of different types. SOME/IP knows nothing
          // about types, so skipping this would deserialize the wrong payload.
          continue;
        }
        const rmw_someip_cpp::SubscriptionData::SubscribeKey key{
          identity.service_id, instance_id, endpoint.event_id, endpoint.eventgroup_id};
        if (!subscription->subscribed.insert(key).second) {
          continue;  // this subscription already holds it
        }
        routes[{identity.service_id, instance_id, endpoint.event_id}].insert(subscription);
        if (++subscribe_refcount[key] > 1) {
          // Another subscription already holds it; just add a route.
          continue;
        }
        pending.push_back(
          {instance_id, endpoint.event_id, endpoint.eventgroup_id, subscription,
            is_field_for(endpoint.durability, endpoint.depth),
            endpoint.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT});
      }
    }
  }

  // SOME/IP calls happen outside entities_mutex.
  for (const auto & entry : pending) {
    endpoint->request_event(
      identity.service_id, entry.instance_id, entry.event_id, entry.eventgroup_id,
      entry.is_field, entry.reliable);
    endpoint->subscribe(identity.service_id, entry.instance_id, entry.eventgroup_id);
  }
}

void rmw_context_impl_s::publish_local_graph()
{
  auto * endpoint = this->endpoint();
  if (endpoint == nullptr || !graph_channel_started_) {
    return;
  }

  rmw_someip_cpp::ParticipantEntitiesInfo info;
  info.instance_id = identity.instance_id;
  info.guid = identity.guid;
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    for (const auto * node : nodes) {
      info.nodes.push_back({node->name_space, node->name, node->enclave});
    }
    // Only the IDs, the QoS and whether there is a gid differ per kind.
    for (const auto * publisher : publishers) {
      auto endpoint = make_endpoint(
        rmw_someip_cpp::EndpointKind::Publisher,
        publisher->node_namespace, publisher->node_name,
        publisher->topic_name, publisher->type_name, publisher->type_support.type_hash);
      endpoint.event_id = publisher->event_id;
      endpoint.eventgroup_id = publisher->eventgroup_id;
      fill_qos(endpoint, publisher->qos);
      std::memcpy(endpoint.gid, publisher->gid.data, RMW_GID_STORAGE_SIZE);
      info.endpoints.push_back(std::move(endpoint));
    }
    for (const auto * service : services) {
      auto endpoint = make_endpoint(
        rmw_someip_cpp::EndpointKind::Service,
        service->node_namespace, service->node_name,
        service->service_name, service->type_name, service->type_support.request.type_hash);
      endpoint.method_id = service->method_id;
      info.endpoints.push_back(std::move(endpoint));
    }
    for (const auto * client : clients) {
      auto endpoint = make_endpoint(
        rmw_someip_cpp::EndpointKind::Client,
        client->node_namespace, client->node_name,
        client->service_name, client->type_name, client->type_support.request.type_hash);
      std::memcpy(endpoint.gid, client->gid.data, RMW_GID_STORAGE_SIZE);
      info.endpoints.push_back(std::move(endpoint));
    }
    for (const auto * subscription : subscriptions) {
      auto endpoint = make_endpoint(
        rmw_someip_cpp::EndpointKind::Subscription,
        subscription->node_namespace, subscription->node_name,
        subscription->topic_name, subscription->type_name,
        subscription->type_support.type_hash);
      fill_qos(endpoint, subscription->qos);
      std::memcpy(endpoint.gid, subscription->gid.data, RMW_GID_STORAGE_SIZE);
      info.endpoints.push_back(std::move(endpoint));
    }
  }

  // Put this participant into the cache too, or its own nodes disappear from
  // `ros2 node list` and rmw_count_publishers.
  //
  // Matching within the process — a publisher and subscription in the same
  // context — is only detectable here, so recompute matched events on change.
  const bool changed = graph_cache.update_participant(info);

  const auto encoded = rmw_someip_cpp::encode_graph_message(info);
  endpoint->notify(
    identity.service_id, identity.instance_id, rmw_someip_cpp::kGraphEventId,
    encoded.data(), encoded.size());

  if (changed) {
    notify_graph_changed();
  }
}

void rmw_context_impl_s::notify_graph_changed()
{
  // Derive the current matched and liveliness counts from the graph. SOME/IP
  // reports neither matching nor liveliness, so the graph is the only source.
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    for (auto * publisher : publishers) {
      const size_t matched =
        graph_cache.count_matched_subscriptions(publisher->topic_name, publisher->qos);
      const auto incompatible =
        graph_cache.count_incompatible_subscriptions(publisher->topic_name, publisher->qos);
      std::lock_guard<std::mutex> events_lock(publisher->events_mutex);
      auto it = publisher->events.find(RMW_EVENT_PUBLICATION_MATCHED);
      if (it != publisher->events.end()) {
        it->second->update_current_count(matched);
      }
      auto incompatible_it = publisher->events.find(RMW_EVENT_OFFERED_QOS_INCOMPATIBLE);
      if (incompatible_it != publisher->events.end()) {
        incompatible_it->second->update_incompatible(
          incompatible.count, incompatible.last_policy_kind);
      }
      // LIVELINESS_LOST is not raised here. Under AUTOMATIC a publisher is
      // alive as long as its process is, so a running process cannot lose it.
      // MANUAL_BY_TOPIC expires on time, which the watchdog owns
      // (docs/design.md §21).
    }
    for (auto * subscription : subscriptions) {
      const size_t matched =
        graph_cache.count_matched_publishers(subscription->topic_name, subscription->qos);
      std::lock_guard<std::mutex> events_lock(subscription->events_mutex);
      auto matched_it = subscription->events.find(RMW_EVENT_SUBSCRIPTION_MATCHED);
      if (matched_it != subscription->events.end()) {
        matched_it->second->update_current_count(matched);
      }
      // LIVELINESS_CHANGED is not updated here either. Under AUTOMATIC the
      // matched count is the alive count, but MANUAL_BY_TOPIC depends on time,
      // and computing them in two places makes them fight — the graph setting
      // alive back to 1 while the watchdog sets it to 0. Both live in
      // check_subscription_liveliness().
      const auto incompatible =
        graph_cache.count_incompatible_publishers(subscription->topic_name, subscription->qos);
      auto incompatible_it = subscription->events.find(RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE);
      if (incompatible_it != subscription->events.end()) {
        incompatible_it->second->update_incompatible(
          incompatible.count, incompatible.last_policy_kind);
      }
    }
  }

  // Both liveliness kinds are computed together here. Called on every graph
  // change so AUTOMATIC alive counts still track when no watchdog is running,
  // which is the case with neither a deadline nor MANUAL_BY_TOPIC.
  check_subscription_liveliness();

  std::lock_guard<std::mutex> lock(entities_mutex);
  for (auto * node : nodes) {
    node->graph_guard_condition.trigger();
  }
}

// ---------------------------------------------------------------------------
// QoS watchdog
// ---------------------------------------------------------------------------

void rmw_context_impl_s::ensure_qos_watchdog()
{
  std::lock_guard<std::mutex> lock(qos_mutex_);
  if (qos_watchdog_.joinable() || qos_stop_) {
    return;
  }
  qos_watchdog_ = std::thread([this]() {qos_watchdog_loop();});
}

void rmw_context_impl_s::stop_qos_watchdog()
{
  {
    std::lock_guard<std::mutex> lock(qos_mutex_);
    qos_stop_ = true;
  }
  qos_cv_.notify_all();
  if (qos_watchdog_.joinable()) {
    qos_watchdog_.join();
  }
}

void rmw_context_impl_s::qos_watchdog_loop()
{
  std::unique_lock<std::mutex> lock(qos_mutex_);
  while (!qos_stop_) {
    // Leave on a stop request without waiting out the tick, so shutdown is
    // not delayed.
    qos_cv_.wait_for(lock, kQosTick, [this]() {return qos_stop_;});
    if (qos_stop_) {
      return;
    }
    // entities_mutex nests inside qos_mutex_, so release this one first.
    lock.unlock();
    check_deadlines();
    check_subscription_liveliness();
    drain_replays();
    lock.lock();
  }
}

void rmw_context_impl_s::check_deadlines()
{
  const int64_t now = rmw_someip_cpp::steady_now_ns();

  // entities_mutex keeps these entities alive: rmw_destroy_publisher() runs
  // remove_publisher(), which erases under this same lock, before deleting.
  std::lock_guard<std::mutex> lock(entities_mutex);
  for (auto * publisher : publishers) {
    if (publisher->deadline.expired(now)) {
      publisher->fire_deadline_missed();
    }
    // MANUAL_BY_TOPIC only; liveliness.enabled() is false for AUTOMATIC.
    if (publisher->liveliness.expired(now)) {
      publisher->fire_liveliness_lost();
    }
  }
  for (auto * subscription : subscriptions) {
    if (subscription->deadline.expired(now)) {
      subscription->fire_deadline_missed();
    }
  }
}

void rmw_context_impl_s::check_subscription_liveliness()
{
  const int64_t now = rmw_someip_cpp::steady_now_ns();
  std::lock_guard<std::mutex> lock(entities_mutex);
  for (auto * subscription : subscriptions) {
    std::lock_guard<std::mutex> events_lock(subscription->events_mutex);
    auto it = subscription->events.find(RMW_EVENT_LIVELINESS_CHANGED);
    if (it == subscription->events.end()) {
      continue;
    }
    // Count matched publishers, splitting AUTOMATIC from MANUAL_BY_TOPIC.
    //
    // AUTOMATIC is alive while on the graph. MANUAL_BY_TOPIC depends on
    // whether the peer wrote within its lease, judged from arrival times here.
    // A peer nothing has been received from yet counts as alive, since its
    // lease cannot have expired — otherwise a freshly discovered publisher
    // would immediately be reported not alive.
    size_t alive = 0;
    size_t not_alive = 0;
    const auto matched = graph_cache.matched_publisher_liveliness(
      subscription->topic_name, subscription->qos);
    for (const auto & [gid, qos] : matched) {
      const int64_t lease = rmw_someip_cpp::finite_duration_ns(qos.liveliness_lease_duration);
      if (qos.liveliness != RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC || lease == 0) {
        ++alive;
        continue;
      }
      const int64_t seen = subscription->last_seen(gid);
      if (seen == 0 || now - seen <= lease) {
        ++alive;
      } else {
        ++not_alive;
      }
    }
    it->second->update_liveliness(alive, not_alive);
  }
}

void rmw_context_impl_s::drain_replays()
{
  std::vector<ReplayRequest> requests;
  {
    std::lock_guard<std::mutex> lock(qos_mutex_);
    requests.swap(pending_replays_);
  }
  if (requests.empty()) {
    return;
  }
  auto * endpoint = this->endpoint();
  if (endpoint == nullptr) {
    return;
  }

  for (const auto & request : requests) {
    // Check under entities_mutex that the publisher still exists;
    // rmw_destroy_publisher() can run between queueing and handling.
    uint16_t service_id = 0;
    uint16_t instance_id = 0;
    uint16_t event_id = 0;
    std::deque<std::vector<uint8_t>> history;
    {
      std::lock_guard<std::mutex> lock(entities_mutex);
      if (publishers.count(request.publisher) == 0) {
        continue;
      }
      service_id = request.publisher->service_id;
      instance_id = request.publisher->instance_id;
      event_id = request.publisher->event_id;
      history = request.publisher->history_snapshot();
    }
    // SOME/IP calls happen outside entities_mutex, per the lock order.
    for (const auto & payload_bytes : history) {
      endpoint->notify_one(
        service_id, instance_id, event_id,
        payload_bytes.data(), payload_bytes.size(), request.client);
    }
  }
}

void rmw_context_impl_s::deliver_local_history(
  rmw_someip_cpp::SubscriptionData * subscription)
{
  // Replay a same-process publisher's history to a late TRANSIENT_LOCAL
  // subscriber.
  //
  // From a remote publisher the history arrives either through a field
  // (depth 1) or a notify_one() replay (depth above 1). Within one process
  // neither applies, because nothing goes through SOME/IP. Without this, the
  // history would arrive from another process but not from the same one.
  if (subscription->qos.durability != RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ||
    subscription->ignore_local_publications)
  {
    return;
  }

  struct Pending
  {
    rmw_gid_t gid;
    std::deque<std::vector<uint8_t>> history;
  };
  std::vector<Pending> pending;
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    for (auto * publisher : publishers) {
      if (publisher->history_depth == 0 ||
        publisher->topic_name != subscription->topic_name ||
        publisher->type_name != subscription->type_name)
      {
        continue;
      }
      pending.push_back({publisher->gid, publisher->history_snapshot()});
    }
  }

  // push() takes the subscription's and the wait set's mutexes, so release
  // entities_mutex first, per the lock order.
  for (const auto & entry : pending) {
    for (const auto & payload : entry.history) {
      rmw_someip_cpp::ReceivedMessage message;
      message.payload = payload;
      message.publisher_gid = entry.gid;
      message.received_timestamp_ns = now_ns();
      subscription->push(std::move(message));
    }
  }
}

void rmw_context_impl_s::deliver_locally(
  const rmw_someip_cpp::PublisherData * publisher,
  const std::vector<uint8_t> & payload)
{
  std::vector<rmw_someip_cpp::SubscriptionData *> targets;
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    for (auto * subscription : subscriptions) {
      if (subscription->ignore_local_publications) {
        continue;
      }
      if (subscription->topic_name != publisher->topic_name ||
        subscription->type_name != publisher->type_name)
      {
        continue;
      }
      targets.push_back(subscription);
    }
  }

  // push() takes the subscription's and the wait set's mutexes, so release
  // entities_mutex first, per the lock order.
  for (auto * subscription : targets) {
    rmw_someip_cpp::ReceivedMessage message;
    message.payload = payload;
    message.publisher_gid = publisher->gid;
    message.received_timestamp_ns = now_ns();
    subscription->push(std::move(message));
  }
}

std::pair<uint16_t, uint16_t> rmw_context_impl_s::allocate_topic_ids_locked()
{
  const uint16_t index = next_topic_index++;
  return {
    static_cast<uint16_t>(rmw_someip_cpp::kFirstTopicEventId + index),
    static_cast<uint16_t>(rmw_someip_cpp::kFirstTopicEventgroupId + index)};
}

void rmw_context_impl_s::add_node(rmw_someip_cpp::NodeData * node)
{
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    nodes.insert(node);
  }
  publish_local_graph();
}

void rmw_context_impl_s::remove_node(rmw_someip_cpp::NodeData * node)
{
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    nodes.erase(node);
  }
  publish_local_graph();
}

/// Decide from the QoS whether this publisher needs time tracking or history.
void rmw_context_impl_s::configure_publisher_qos(rmw_someip_cpp::PublisherData * publisher)
{
  publisher->deadline.configure(publisher->qos.deadline);
  // Only MANUAL_BY_TOPIC has a lease to watch. AUTOMATIC cannot lose
  // liveliness while its own process runs.
  if (publisher->qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC) {
    publisher->liveliness.configure(publisher->qos.liveliness_lease_duration);
    publisher->liveliness.set_one_shot();
  }
  // TRANSIENT_LOCAL keeps history, at depth 1 too: a field does not reach a
  // late subscriber in the same process, and deliver_locally has no history.
  if (publisher->qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
    publisher->history_depth = std::max<size_t>(1u, publisher->qos.depth);
    publisher->replay_to_remote = publisher->qos.depth > 1;
  }
  if (publisher->deadline.enabled() || publisher->liveliness.enabled() ||
    publisher->history_depth > 0)
  {
    ensure_qos_watchdog();
  }
}

/// Register an extern-mode publisher.
///
/// Every ID is pinned by someip_mapping.yaml, and the service offered is not
/// the participant's; its port is declared in the generated JSON.
void rmw_context_impl_s::add_extern_publisher(
  rmw_someip_cpp::PublisherData * publisher,
  const rmw_someip_cpp::ExternTopic & mapped,
  rmw_someip_cpp::SomeipEndpoint * endpoint)
{
  publisher->is_extern = true;
  publisher->serialization = mapped.serialization;
  publisher->service_id = mapped.service_id;
  publisher->instance_id = mapped.instance_id;
  publisher->event_id = mapped.event_id;
  publisher->eventgroup_id = mapped.eventgroup_id;

  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    publisher->gid = rmw_someip_cpp::make_entity_gid(mapped.instance_id, mapped.event_id);
    publishers.insert(publisher);
  }

  endpoint->offer_service(mapped.service_id, mapped.instance_id);
  endpoint->offer_event(
    mapped.service_id, mapped.instance_id, mapped.event_id, mapped.eventgroup_id,
    false, mapped.reliable);
}

/// Register a native-mode publisher.
///
/// Event and eventgroup are numbered sequentially within the participant's
/// service, and the assigned IDs are announced on the graph channel.
void rmw_context_impl_s::add_native_publisher(
  rmw_someip_cpp::PublisherData * publisher,
  rmw_someip_cpp::SomeipEndpoint * endpoint)
{
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    const auto ids = allocate_topic_ids_locked();
    publisher->service_id = identity.service_id;
    publisher->instance_id = identity.instance_id;
    publisher->event_id = ids.first;
    publisher->eventgroup_id = ids.second;
    // Assign the gid here. Setting it afterwards at the call site would let
    // the publish_local_graph() that follows announce a zeroed gid.
    publisher->gid =
      rmw_someip_cpp::make_entity_gid(identity.instance_id, publisher->event_id);
    publishers.insert(publisher);
  }

  endpoint->offer_event(
    identity.service_id, identity.instance_id, publisher->event_id,
    publisher->eventgroup_id, is_field_for(publisher->qos), is_reliable(publisher->qos));

  if (publisher->replay_to_remote) {
    // Watch for subscriptions so history can be replayed to late subscribers.
    //
    // The handler only queues a request rather than calling notify_one(),
    // which would send while holding vsomeip's own lock (docs/design.md §21).
    endpoint->on_subscription(
      identity.service_id, identity.instance_id, publisher->eventgroup_id,
      [this, publisher](uint16_t client) {
        std::lock_guard<std::mutex> lock(qos_mutex_);
        pending_replays_.push_back({publisher, client});
      });
  }
}

bool rmw_context_impl_s::add_publisher(
  rmw_someip_cpp::PublisherData * publisher, std::string & error)
{
  auto * endpoint = this->endpoint();
  if (endpoint == nullptr) {
    error = "the SOME/IP endpoint is not available";
    return false;
  }

  configure_publisher_qos(publisher);

  const auto * mapped = extern_mapping.find(publisher->topic_name);
  if (mapped != nullptr && mapped->ros_is_publisher) {
    add_extern_publisher(publisher, *mapped, endpoint);
  } else {
    add_native_publisher(publisher, endpoint);
  }

  publish_local_graph();
  return true;
}

void rmw_context_impl_s::remove_publisher(rmw_someip_cpp::PublisherData * publisher)
{
  if (auto * endpoint = this->endpoint()) {
    if (publisher->replay_to_remote) {
      // The handler captures the publisher by raw pointer, so it has to go
      // before the publisher does.
      endpoint->clear_subscription_handler(
        publisher->service_id, publisher->instance_id, publisher->eventgroup_id);
    }
    endpoint->stop_offer_event(
      publisher->service_id, publisher->instance_id, publisher->event_id);
  }
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    publishers.erase(publisher);
  }
  {
    // Drop pending replay requests too, so no destroyed publisher is touched.
    std::lock_guard<std::mutex> lock(qos_mutex_);
    pending_replays_.erase(
      std::remove_if(
        pending_replays_.begin(), pending_replays_.end(),
        [publisher](const ReplayRequest & request) {
          return request.publisher == publisher;
        }),
      pending_replays_.end());
  }
  publish_local_graph();
}

bool rmw_context_impl_s::add_subscription(
  rmw_someip_cpp::SubscriptionData * subscription, std::string & error)
{
  auto * endpoint = this->endpoint();
  if (endpoint == nullptr) {
    error = "the SOME/IP endpoint is not available";
    return false;
  }

  subscription->deadline.configure(subscription->qos.deadline);
  subscription->lifespan_ns = rmw_someip_cpp::finite_duration_ns(subscription->qos.lifespan);
  // MANUAL_BY_TOPIC expires because nothing happened, which no graph change
  // can reveal; it takes a thread watching the clock.
  if (subscription->deadline.enabled() ||
    subscription->qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
  {
    ensure_qos_watchdog();
  }

  const auto * mapped = extern_mapping.find(subscription->topic_name);
  if (mapped != nullptr && !mapped->ros_is_publisher) {
    // Extern mode: the peer does not join the graph channel, so subscribe
    // directly using the IDs pinned in the configuration file.
    subscription->is_extern = true;
    subscription->serialization = mapped->serialization;
    const rmw_someip_cpp::SubscriptionData::SubscribeKey key{
      mapped->service_id, mapped->instance_id, mapped->event_id, mapped->eventgroup_id};
    bool first = false;
    {
      std::lock_guard<std::mutex> lock(entities_mutex);
      subscription->gid =
        rmw_someip_cpp::make_entity_gid(identity.instance_id, next_entity_id++);
      subscriptions.insert(subscription);
      subscription->subscribed.insert(key);
      routes[{mapped->service_id, mapped->instance_id, mapped->event_id}].insert(subscription);
      first = (++subscribe_refcount[key] == 1);
    }
    if (first) {
      endpoint->request_service(mapped->service_id, mapped->instance_id);
      endpoint->request_event(
        mapped->service_id, mapped->instance_id, mapped->event_id, mapped->eventgroup_id,
        false, mapped->reliable);
      endpoint->subscribe(mapped->service_id, mapped->instance_id, mapped->eventgroup_id);
    }
    publish_local_graph();
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    // Assign the gid here. Setting it afterwards at the call site would let
    // the publish_local_graph() that follows announce a zeroed gid.
    //
    // A subscription owns no SOME/IP event, so it takes a serial number like a
    // client does. Leaving it all zeros would make subscriptions in one
    // process indistinguishable by gid, and `ros2 topic info -v` would show
    // them all the same.
    subscription->gid =
      rmw_someip_cpp::make_entity_gid(identity.instance_id, next_entity_id++);
    subscriptions.insert(subscription);
  }
  deliver_local_history(subscription);
  // Connect immediately to publishers already in the cache, which is what
  // happens when the peer started first.
  match_subscriptions();
  publish_local_graph();
  return true;
}

void rmw_context_impl_s::remove_subscription(
  rmw_someip_cpp::SubscriptionData * subscription)
{
  // Unsubscribe only the subscriptions whose refcount reached zero.
  std::vector<rmw_someip_cpp::SubscriptionData::SubscribeKey> to_release;
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    subscriptions.erase(subscription);
    for (auto it = routes.begin(); it != routes.end(); ) {
      it->second.erase(subscription);
      it = it->second.empty() ? routes.erase(it) : std::next(it);
    }
    for (const auto & key : subscription->subscribed) {
      auto it = subscribe_refcount.find(key);
      if (it == subscribe_refcount.end()) {
        continue;
      }
      if (--it->second == 0) {
        subscribe_refcount.erase(it);
        to_release.push_back(key);
      }
    }
    subscription->subscribed.clear();
  }

  // SOME/IP calls happen outside entities_mutex, per the lock order.
  if (auto * endpoint = this->endpoint()) {
    for (const auto & [service_id, instance_id, event_id, eventgroup_id] : to_release) {
      endpoint->unsubscribe(service_id, instance_id, eventgroup_id);
      endpoint->release_event(service_id, instance_id, event_id);
    }
  }

  publish_local_graph();
}

void rmw_context_impl_s::on_someip_request(
  const std::shared_ptr<vsomeip::message> & message)
{
  const auto method_id = static_cast<uint16_t>(message->get_method());
  auto payload = message->get_payload();

  rmw_someip_cpp::PendingRequest request;
  if (!request.header.read_from(payload->get_data(), payload->get_length())) {
    return;  // no correlation header, so not a request from this RMW
  }
  const auto * body = payload->get_data() + rmw_someip_cpp::ServiceHeader::kSize;
  const size_t body_size = payload->get_length() - rmw_someip_cpp::ServiceHeader::kSize;
  request.payload.assign(body, body + body_size);
  request.original = message;
  request.received_timestamp_ns = now_ns();

  rmw_someip_cpp::ServiceData * target = nullptr;
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    auto it = service_routes.find(method_id);
    if (it == service_routes.end()) {
      return;
    }
    target = it->second;
  }
  // push() takes the service's and the wait set's mutexes, so release
  // entities_mutex first, per the lock order.
  target->push(std::move(request));
}

void rmw_context_impl_s::on_someip_response(
  const std::shared_ptr<vsomeip::message> & message)
{
  auto payload = message->get_payload();

  rmw_someip_cpp::ReceivedResponse response;
  if (!response.header.read_from(payload->get_data(), payload->get_length())) {
    return;
  }
  const auto * body = payload->get_data() + rmw_someip_cpp::ServiceHeader::kSize;
  const size_t body_size = payload->get_length() - rmw_someip_cpp::ServiceHeader::kSize;
  response.payload.assign(body, body + body_size);
  response.received_timestamp_ns = now_ns();

  // Route the response by the client gid in the payload. SOME/IP only returns
  // it to the sending process, which is not enough to tell apart several
  // clients within one.
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> key{};
  std::memcpy(key.data(), response.header.client_gid, RMW_GID_STORAGE_SIZE);

  rmw_someip_cpp::ClientData * target = nullptr;
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    auto it = client_routes.find(key);
    if (it == client_routes.end()) {
      return;
    }
    target = it->second;
  }
  target->push(std::move(response));
}

bool rmw_context_impl_s::add_service(
  rmw_someip_cpp::ServiceData * service, std::string & error)
{
  if (endpoint() == nullptr) {
    error = "the SOME/IP endpoint is not available";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    service->method_id = static_cast<uint16_t>(
      rmw_someip_cpp::kFirstServiceMethodId + next_service_index++);
    services.insert(service);
    service_routes[service->method_id] = service;
  }
  // start_graph_channel already registered one handler for ANY_METHOD, so no
  // further registration is needed.
  publish_local_graph();
  return true;
}

void rmw_context_impl_s::remove_service(rmw_someip_cpp::ServiceData * service)
{
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    services.erase(service);
    service_routes.erase(service->method_id);
  }
  publish_local_graph();
}

bool rmw_context_impl_s::add_client(
  rmw_someip_cpp::ClientData * client, std::string & error)
{
  if (endpoint() == nullptr) {
    error = "the SOME/IP endpoint is not available";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    const uint16_t entity_id = next_entity_id++;
    client->gid = rmw_someip_cpp::make_entity_gid(identity.instance_id, entity_id);

    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key{};
    std::memcpy(key.data(), client->gid.data, RMW_GID_STORAGE_SIZE);
    clients.insert(client);
    client_routes[key] = client;
  }
  publish_local_graph();
  return true;
}

void rmw_context_impl_s::remove_client(rmw_someip_cpp::ClientData * client)
{
  {
    std::lock_guard<std::mutex> lock(entities_mutex);
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key{};
    std::memcpy(key.data(), client->gid.data, RMW_GID_STORAGE_SIZE);
    clients.erase(client);
    client_routes.erase(key);
  }
  publish_local_graph();
}

std::pair<uint16_t, uint16_t> rmw_context_impl_s::find_remote_service(
  const std::string & service_name) const
{
  const auto found = graph_cache.find_service(service_name);
  return {found.first, found.second.method_id};
}
