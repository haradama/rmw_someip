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

#ifndef DETAIL__CONTEXT_IMPL_HPP_
#define DETAIL__CONTEXT_IMPL_HPP_

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <utility>

#include <vsomeip/vsomeip.hpp>

#include "detail/entity_data.hpp"
#include "detail/extern_mapping.hpp"
#include "detail/graph_cache.hpp"
#include "detail/guard_condition.hpp"
#include "detail/participant.hpp"
#include "detail/someip_endpoint.hpp"

namespace rmw_someip_cpp
{
struct NodeData;

/// IDs reserved for the graph channel (docs/design.md §5.1).
/// Topics are numbered sequentially from just after these.
constexpr uint16_t kGraphEventId = 0x8000;
constexpr uint16_t kGraphEventgroupId = 0x0001;
constexpr uint16_t kFirstTopicEventId = 0x8001;
constexpr uint16_t kFirstTopicEventgroupId = 0x0002;
/// First SOME/IP method ID for ROS services.
/// Methods occupy 0x0000-0x7FFF; 0x8000 and above are events.
constexpr uint16_t kFirstServiceMethodId = 0x0001;

}  // namespace rmw_someip_cpp

/// Backing object for rmw_context_impl_t, which rmw declares as incomplete.
///
/// One ROS process is one rmw_context, one vsomeip application, one SOME/IP
/// instance.
///
/// Lock order, strictly:
///   entities_mutex -> SubscriptionData::waitable_mutex -> WaitSetImpl mutex
///   entities_mutex -> GraphCache mutex
/// Never the reverse. vsomeip callback threads and the executor thread run
/// concurrently, so an inverted path deadlocks immediately.
struct rmw_context_impl_s
{
  rmw_context_impl_s(
    size_t domain_id,
    rmw_someip_cpp::ParticipantIdentity identity,
    std::unique_ptr<rmw_someip_cpp::SomeipEndpoint> session);

  ~rmw_context_impl_s();

  size_t domain_id;

  /// Has rmw_shutdown() been called?
  ///
  /// rmw requires context_fini() to reject a context that was not shut down,
  /// and node creation to fail afterwards. Whether a session exists cannot
  /// stand in for this: it would not distinguish shutdown from a session that
  /// died on its own.
  bool is_shutdown {false};

  rmw_someip_cpp::ParticipantIdentity identity;

  /// The way out to SOME/IP; tests substitute a fake (detail/someip_endpoint.hpp).
  std::unique_ptr<rmw_someip_cpp::SomeipEndpoint> session;

  /// Safe access to SOME/IP.
  ///
  /// As designed today this never returns null. A session is always passed
  /// to the constructor — rmw_init() does not build a context if
  /// SomeipSession::create() failed — and is destroyed only in the destructor.
  /// rmw_context_fini() refuses while nodes remain, so no entity outlives it.
  ///
  /// The null checks at the call sites stay anyway. Back when rmw_shutdown()
  /// destroyed the session, this really did crash: rcl keeps destroying
  /// entities afterwards, since rcl_logging_rosout_fini() removes the rosout
  /// publisher after rmw_shutdown(). Moving the destruction point reopens that
  /// hole, so do not remove the checks along with it. They are unreachable,
  /// which leaves a gap in coverage (docs/design.md §23).
  rmw_someip_cpp::SomeipEndpoint * endpoint() const {return session.get();}

  /// The ROS graph received from other participants.
  rmw_someip_cpp::GraphCache graph_cache;

  // -- Local entities --------------------------------------------------------
  std::mutex entities_mutex;
  std::set<rmw_someip_cpp::NodeData *> nodes;
  std::set<rmw_someip_cpp::PublisherData *> publishers;
  std::set<rmw_someip_cpp::SubscriptionData *> subscriptions;
  std::set<rmw_someip_cpp::ServiceData *> services;
  std::set<rmw_someip_cpp::ClientData *> clients;

  /// method_id to this participant's service, for routing requests.
  std::map<uint16_t, rmw_someip_cpp::ServiceData *> service_routes;
  /// Client gid to client, for routing responses.
  ///
  /// SOME/IP returns a response to the sender but cannot tell apart several
  /// clients in one process, so the gid in the correlation header is used.
  std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>,
    rmw_someip_cpp::ClientData *> client_routes;

  /// (service_id, instance_id, event_id) to the subscriptions to deliver to.
  /// vsomeip callbacks route through this table.
  ///
  /// service_id has to be part of the key: extern-mode topics use a service_id
  /// other than the participant's, and would otherwise collide.
  using RouteKey = std::tuple<uint16_t, uint16_t, uint16_t>;
  std::map<RouteKey, std::set<rmw_someip_cpp::SubscriptionData *>> routes;

  /// How many local subscriptions hold each
  /// (service_id, instance_id, event_id, eventgroup_id).
  ///
  /// Two subscriptions can share a topic, so destroying one must not
  /// unsubscribe. Never unsubscribing would instead pile up vsomeip
  /// subscriptions in an application that creates and destroys them. Only
  /// the transition to zero unsubscribes.
  std::map<rmw_someip_cpp::SubscriptionData::SubscribeKey, size_t> subscribe_refcount;

  /// Contents of someip_mapping.yaml, for extern mode.
  rmw_someip_cpp::ExternMapping extern_mapping;

  /// Sequential counters for topic event and eventgroup IDs. Being sequential
  /// rather than hashed, they cannot collide. The IDs change across a restart,
  /// which does not matter because the graph channel is authoritative.
  uint16_t next_topic_index {0};

  /// Sequential counter for service method IDs.
  uint16_t next_service_index {0};

  /// Serial number for entity gids, shared by publishers and clients.
  uint16_t next_entity_id {1};

  // -- Graph Channel ---------------------------------------------------------

  /// Offer and subscribe the graph channel. Called once from rmw_init.
  bool start_graph_channel(std::string & error);

  /// Encode the local graph and publish it. Called whenever entities change.
  void publish_local_graph();

  /// Signal every node's graph guard condition and recompute matched events.
  void notify_graph_changed();

  // -- Entity registration ---------------------------------------------------
  // All of these call publish_local_graph() afterwards.

  void add_node(rmw_someip_cpp::NodeData * node);
  void remove_node(rmw_someip_cpp::NodeData * node);
  bool add_publisher(rmw_someip_cpp::PublisherData * publisher, std::string & error);
  void remove_publisher(rmw_someip_cpp::PublisherData * publisher);
  bool add_subscription(rmw_someip_cpp::SubscriptionData * subscription, std::string & error);
  void remove_subscription(rmw_someip_cpp::SubscriptionData * subscription);
  bool add_service(rmw_someip_cpp::ServiceData * service, std::string & error);
  void remove_service(rmw_someip_cpp::ServiceData * service);
  bool add_client(rmw_someip_cpp::ClientData * client, std::string & error);
  void remove_client(rmw_someip_cpp::ClientData * client);

  /// Find a remote service by name.
  /// \return (instance_id, method_id), with instance_id 0 when not found
  std::pair<uint16_t, uint16_t> find_remote_service(const std::string & service_name) const;

  /// Deliver straight to subscriptions in this context.
  ///
  /// A SOME/IP subscription is against *another* participant's service, so an
  /// event this process notifies never comes back to it. DDS delivers within
  /// the process through loopback; matching that behaviour requires an
  /// explicit hand-off here. Without it a talker and listener in one process
  /// cannot communicate.
  void deliver_locally(
    const rmw_someip_cpp::PublisherData * publisher,
    const std::vector<uint8_t> & payload);

  /// Replay a same-process publisher's history to a new TRANSIENT_LOCAL
  /// subscription. Neither fields nor replay reach a path that skips SOME/IP.
  void deliver_local_history(rmw_someip_cpp::SubscriptionData * subscription);

  /// Allocate the next topic IDs. Call with entities_mutex held.
  std::pair<uint16_t, uint16_t> allocate_topic_ids_locked();

private:
  // -- QoS watchdog ----------------------------------------------------------
  //
  // A deadline detects that nothing happened for a while, which no
  // message-driven path can notice; it takes a thread watching the clock. The
  // thread starts only once an entity with a deadline exists, so the common
  // case adds none.

  /// Watchdog tick, which bounds how late a deadline can be reported.
  static constexpr auto kQosTick = std::chrono::milliseconds(20);

  void ensure_qos_watchdog();
  void stop_qos_watchdog();
  void qos_watchdog_loop();

  /// Raise deadline missed and liveliness lost on expired entities.
  void check_deadlines();

  /// Update subscriptions' LIVELINESS_CHANGED from whether MANUAL_BY_TOPIC
  /// publishers wrote within their lease.
  void check_subscription_liveliness();

  /// Replay TRANSIENT_LOCAL history to late subscribers.
  void drain_replays();

  /// A pending history replay, queued from a vsomeip callback thread and
  /// handled by the housekeeping thread.
  ///
  /// Calling notify_one() inside the callback would send while holding
  /// vsomeip's own lock, hence the hand-off.
  struct ReplayRequest
  {
    rmw_someip_cpp::PublisherData * publisher;
    uint16_t client;
  };
  std::vector<ReplayRequest> pending_replays_;

  std::thread qos_watchdog_;
  std::mutex qos_mutex_;
  std::condition_variable qos_cv_;
  bool qos_stop_ {false};

  /// Message arrival, on a vsomeip thread. Dispatches on message type and ID.
  void on_someip_message(const std::shared_ptr<vsomeip::message> & message);

  /// Graph channel arrival, on a vsomeip thread.
  void on_graph_message(uint16_t instance_id, const vsomeip::payload & payload);

  /// Topic data arrival, on a vsomeip thread.
  void on_topic_message(
    uint16_t service_id,
    uint16_t instance_id,
    uint16_t event_id,
    const vsomeip::payload & payload);

  /// Service request arrival, on a vsomeip thread.
  void on_someip_request(const std::shared_ptr<vsomeip::message> & message);

  /// Client response arrival, on a vsomeip thread.
  void on_someip_response(const std::shared_ptr<vsomeip::message> & message);

  /// A participant appeared or went away, on a vsomeip thread.
  void on_availability(uint16_t instance_id, bool available);

  /// Report an instance_id collision, once per peer guid. The graph channel
  /// repeats, so without suppression this would flood the log.
  void report_instance_id_collision(uint64_t other_guid);

  std::set<uint64_t> reported_collisions_;

  /// After a graph update, connect local subscriptions to matching remote
  /// publishers. Call without entities_mutex held; it takes the lock itself.
  void match_subscriptions();

  // -- Pieces of add_publisher -----------------------------------------------
  void configure_publisher_qos(rmw_someip_cpp::PublisherData * publisher);
  void add_extern_publisher(
    rmw_someip_cpp::PublisherData * publisher,
    const rmw_someip_cpp::ExternTopic & mapped,
    rmw_someip_cpp::SomeipEndpoint * endpoint);
  void add_native_publisher(
    rmw_someip_cpp::PublisherData * publisher,
    rmw_someip_cpp::SomeipEndpoint * endpoint);

  bool graph_channel_started_ {false};
};

#endif  // DETAIL__CONTEXT_IMPL_HPP_
