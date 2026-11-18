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
// Running rmw_context_impl_s without SOME/IP.
//
// It is the largest file in the implementation, but what it does — interpreting
// the graph, matching subscriptions, maintaining routing tables, deriving QoS
// events — needs no actual traffic. With a fake SomeipEndpoint, events that
// cannot be provoked against the real thing (a peer appearing, a graph
// arriving, a subscription starting) can be raised in any order.

#include <array>
#include <fstream>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rmw/qos_profiles.h"

#include "detail/context_impl.hpp"
#include "detail/graph_message.hpp"
#include "detail/extern_mapping.hpp"
#include "detail/node_data.hpp"

#include "fake_endpoint.hpp"

using rmw_someip_cpp_test::FakeEndpoint;

namespace
{

constexpr uint16_t kServiceId = 0xF000;
constexpr uint16_t kOwnInstance = 0x0001;
constexpr uint16_t kPeerInstance = 0x0002;

rmw_someip_cpp::ParticipantIdentity make_identity()
{
  rmw_someip_cpp::ParticipantIdentity id;
  id.service_id = kServiceId;
  id.instance_id = kOwnInstance;
  id.guid = 0x1111222233334444ULL;
  id.application_name = "unit";
  return id;
}

rmw_qos_profile_t default_qos()
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  qos.depth = 10;
  return qos;
}

/// Build the graph channel payload a peer would send.
std::vector<uint8_t> peer_graph_with_publisher(
  const std::string & topic, const std::string & type,
  uint16_t event_id = 0x8001, uint16_t eventgroup_id = 0x0002,
  uint8_t durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
  uint32_t depth = 10)
{
  rmw_someip_cpp::ParticipantEntitiesInfo info;
  info.instance_id = kPeerInstance;
  info.guid = 0xAAAABBBBCCCCDDDDULL;
  info.nodes.push_back({"/", "peer", ""});

  rmw_someip_cpp::EndpointInfo publisher;
  publisher.kind = rmw_someip_cpp::EndpointKind::Publisher;
  publisher.node_namespace = "/";
  publisher.node_name = "peer";
  publisher.topic_name = topic;
  publisher.type_name = type;
  publisher.event_id = event_id;
  publisher.eventgroup_id = eventgroup_id;
  publisher.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  publisher.durability = durability;
  publisher.depth = depth;
  publisher.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  info.endpoints.push_back(publisher);
  return rmw_someip_cpp::encode_graph_message(info);
}

/// Build a vsomeip message. The runtime needs no routing manager, so this
/// works in tests that use the fake endpoint.
std::shared_ptr<vsomeip::message> make_notification(
  uint16_t service, uint16_t instance, uint16_t event, const std::vector<uint8_t> & payload)
{
  auto runtime = vsomeip::runtime::get();
  auto message = runtime->create_message();
  message->set_service(service);
  message->set_instance(instance);
  message->set_method(event);
  message->set_message_type(vsomeip::message_type_e::MT_NOTIFICATION);
  auto body = runtime->create_payload();
  if (!payload.empty()) {
    body->set_data(payload.data(), static_cast<uint32_t>(payload.size()));
  }
  message->set_payload(body);
  return message;
}

/// A context together with the fake endpoint injected into it.
class ContextImplTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto endpoint = std::make_unique<FakeEndpoint>();
    fake = endpoint.get();
    context = std::make_unique<rmw_context_impl_s>(
      0, make_identity(), std::move(endpoint));
    std::string error;
    ASSERT_TRUE(context->start_graph_channel(error)) << error;
  }

  void TearDown() override
  {
    for (auto & subscription : subscriptions_) {
      context->remove_subscription(subscription.get());
    }
    for (auto & publisher : publishers_) {
      context->remove_publisher(publisher.get());
    }
    context.reset();
  }

  rmw_someip_cpp::PublisherData * add_publisher(
    const std::string & topic, const std::string & type,
    const rmw_qos_profile_t & qos = default_qos())
  {
    publishers_.push_back(std::make_unique<rmw_someip_cpp::PublisherData>());
    auto * publisher = publishers_.back().get();
    publisher->context = context.get();
    publisher->topic_name = topic;
    publisher->type_name = type;
    publisher->qos = qos;
    publisher->node_namespace = "/";
    publisher->node_name = "unit";
    std::string error;
    EXPECT_TRUE(context->add_publisher(publisher, error)) << error;
    return publisher;
  }

  rmw_someip_cpp::SubscriptionData * add_subscription(
    const std::string & topic, const std::string & type,
    const rmw_qos_profile_t & qos = default_qos())
  {
    subscriptions_.push_back(std::make_unique<rmw_someip_cpp::SubscriptionData>());
    auto * subscription = subscriptions_.back().get();
    subscription->context = context.get();
    subscription->topic_name = topic;
    subscription->type_name = type;
    subscription->qos = qos;
    subscription->node_namespace = "/";
    subscription->node_name = "unit";
    std::string error;
    EXPECT_TRUE(context->add_subscription(subscription, error)) << error;
    return subscription;
  }

  FakeEndpoint * fake {nullptr};
  std::unique_ptr<rmw_context_impl_s> context;

private:
  std::vector<std::unique_ptr<rmw_someip_cpp::PublisherData>> publishers_;
  std::vector<std::unique_ptr<rmw_someip_cpp::SubscriptionData>> subscriptions_;
};

}  // namespace

/// A participant offers exactly one service, publishes the graph channel as a
/// field, and requests every instance with one request_service
/// (docs/design.md §5).
TEST_F(ContextImplTest, start_graph_channel_offers_the_participant_service)
{
  ASSERT_EQ(1u, fake->offered_services.size());
  EXPECT_EQ(kServiceId, fake->offered_services[0].first);
  EXPECT_EQ(kOwnInstance, fake->offered_services[0].second);

  ASSERT_GE(fake->offered_events.size(), 1u);
  const auto & graph_event = fake->offered_events[0];
  EXPECT_EQ(rmw_someip_cpp::kGraphEventId, graph_event.event);
  EXPECT_EQ(rmw_someip_cpp::kGraphEventgroupId, graph_event.eventgroup);
  // ET_FIELD, so a participant joining later gets the graph on subscribe.
  EXPECT_TRUE(graph_event.is_field);
  // TCP, because the graph exceeds UDP's 1416-byte limit.
  EXPECT_TRUE(graph_event.reliable);

  ASSERT_EQ(1u, fake->requested_services.size());
  EXPECT_EQ(vsomeip::ANY_INSTANCE, fake->requested_services[0].second);

  EXPECT_TRUE(fake->has_message_handler());
  EXPECT_TRUE(fake->has_availability_handler());

  // Publishes its own graph once at startup.
  EXPECT_FALSE(fake->notified.empty());
}

TEST_F(ContextImplTest, start_graph_channel_is_idempotent)
{
  std::string error;
  EXPECT_TRUE(context->start_graph_channel(error));
  EXPECT_EQ(1u, fake->offered_services.size()) << "offered twice";
}

/// A peer appearing gets its graph channel subscribed to.
TEST_F(ContextImplTest, availability_subscribes_to_the_peer_graph_channel)
{
  fake->fire_availability(kPeerInstance, true);

  ASSERT_EQ(1u, fake->subscribed.size());
  EXPECT_EQ(kServiceId, fake->subscribed[0].service);
  EXPECT_EQ(kPeerInstance, fake->subscribed[0].instance);
  EXPECT_EQ(rmw_someip_cpp::kGraphEventgroupId, fake->subscribed[0].eventgroup);

  // This participant's own availability is ignored.
  fake->subscribed.clear();
  fake->fire_availability(kOwnInstance, true);
  EXPECT_TRUE(fake->subscribed.empty());
}

TEST_F(ContextImplTest, availability_false_removes_the_participant)
{
  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/chatter", "std_msgs/msg/String")));
  EXPECT_EQ(1u, context->graph_cache.count_publishers("/chatter"));

  fake->fire_availability(kPeerInstance, false);
  EXPECT_EQ(0u, context->graph_cache.count_publishers("/chatter"));
}

/// An arriving graph connects subscriptions whose type matches.
TEST_F(ContextImplTest, matching_subscribes_to_the_remote_event)
{
  add_subscription("/chatter", "std_msgs/msg/String");
  fake->subscribed.clear();
  fake->requested_events.clear();

  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/chatter", "std_msgs/msg/String", 0x8001, 0x0002)));

  ASSERT_EQ(1u, fake->requested_events.size());
  EXPECT_EQ(0x8001, fake->requested_events[0].event);
  EXPECT_EQ(0x0002, fake->requested_events[0].eventgroup);
  ASSERT_EQ(1u, fake->subscribed.size());
  EXPECT_EQ(kPeerInstance, fake->subscribed[0].instance);
}

/// A different type name does not connect. SOME/IP knows nothing about
/// types, so not rejecting here would deserialize the wrong payload.
TEST_F(ContextImplTest, matching_ignores_a_different_type_on_the_same_topic)
{
  add_subscription("/chatter", "std_msgs/msg/String");
  fake->subscribed.clear();

  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/chatter", "std_msgs/msg/Int32")));

  EXPECT_TRUE(fake->subscribed.empty());
}

/// TRANSIENT_LOCAL above depth 1 is offered as ET_EVENT, because a field holds
/// only the latest value (docs/design.md §21).
TEST_F(ContextImplTest, transient_local_depth_selects_the_event_type)
{
  auto shallow = default_qos();
  shallow.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  shallow.depth = 1;
  add_publisher("/shallow", "T", shallow);
  ASSERT_GE(fake->offered_events.size(), 2u);
  EXPECT_TRUE(fake->offered_events.back().is_field);

  auto deep = shallow;
  deep.depth = 5;
  add_publisher("/deep", "T", deep);
  EXPECT_FALSE(fake->offered_events.back().is_field) << "offered depth>1 as a field";
}

/// The subscribing side picks the event type from the publisher's
/// durability and depth. Disagreeing with the sender breaks the subscription.
TEST_F(ContextImplTest, matching_mirrors_the_publisher_event_type)
{
  auto qos = default_qos();
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos.depth = 1;
  add_subscription("/tl", "T", qos);
  fake->requested_events.clear();

  // The peer offers at depth 5, so ET_EVENT.
  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher(
        "/tl", "T", 0x8001, 0x0002, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, 5)));

  ASSERT_EQ(1u, fake->requested_events.size());
  EXPECT_FALSE(fake->requested_events[0].is_field)
    << "used its own depth instead of the publisher's";
}

/// Two subscriptions on one peer subscribe once in vsomeip. Destroying one
/// does not unsubscribe; only destroying both does.
TEST_F(ContextImplTest, subscriptions_are_reference_counted)
{
  auto * first = add_subscription("/shared", "T");
  auto * second = add_subscription("/shared", "T");
  fake->subscribed.clear();

  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/shared", "T")));
  EXPECT_EQ(1u, fake->subscribed.size()) << "subscribed to the same peer twice";

  context->remove_subscription(first);
  EXPECT_TRUE(fake->unsubscribed.empty()) << "unsubscribed with a subscriber left";

  context->remove_subscription(second);
  ASSERT_EQ(1u, fake->unsubscribed.size());
  EXPECT_EQ(kPeerInstance, fake->unsubscribed[0].instance);
  ASSERT_EQ(1u, fake->released_events.size());
}

/// Received data is queued to the subscription the routing table names.
TEST_F(ContextImplTest, topic_data_is_delivered_to_matching_subscriptions)
{
  auto * subscription = add_subscription("/data", "T");
  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/data", "T", 0x8005, 0x0006)));

  const std::vector<uint8_t> payload = {0x00, 0x01, 0x00, 0x00, 0xDE, 0xAD};
  fake->fire_message(make_notification(kServiceId, kPeerInstance, 0x8005, payload));

  rmw_someip_cpp::ReceivedMessage message;
  ASSERT_TRUE(subscription->take(message));
  EXPECT_EQ(payload, message.payload);
  // The gid is rebuilt from (instance, event).
  EXPECT_EQ(
    0, std::memcmp(
      message.publisher_gid.data,
      rmw_someip_cpp::make_entity_gid(kPeerInstance, 0x8005).data, RMW_GID_STORAGE_SIZE));
}

/// An event that is not in the routing table is dropped.
TEST_F(ContextImplTest, unknown_events_are_dropped)
{
  auto * subscription = add_subscription("/data", "T");
  const std::vector<uint8_t> payload = {0x00, 0x01, 0x00, 0x00};
  fake->fire_message(make_notification(kServiceId, kPeerInstance, 0x9999, payload));

  rmw_someip_cpp::ReceivedMessage message;
  EXPECT_FALSE(subscription->take(message));
}

/// Zero length is a MANUAL_BY_TOPIC liveliness assertion, not a message.
TEST_F(ContextImplTest, a_zero_length_notification_is_a_liveliness_beacon)
{
  auto qos = default_qos();
  qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  qos.liveliness_lease_duration = {10, 0};
  auto * subscription = add_subscription("/beacon", "T", qos);

  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/beacon", "T", 0x8007, 0x0008)));

  fake->fire_message(make_notification(kServiceId, kPeerInstance, 0x8007, {}));

  rmw_someip_cpp::ReceivedMessage message;
  EXPECT_FALSE(subscription->take(message)) << "queued the assertion as a message";
  // Recorded as liveliness instead.
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid{};
  std::memcpy(
    gid.data(), rmw_someip_cpp::make_entity_gid(kPeerInstance, 0x8007).data,
    RMW_GID_STORAGE_SIZE);
  EXPECT_NE(0, subscription->last_seen(gid));
}

/// A participant does not ingest its own graph; the guid tells them apart.
TEST_F(ContextImplTest, own_graph_echo_is_ignored)
{
  rmw_someip_cpp::ParticipantEntitiesInfo own;
  own.instance_id = kOwnInstance;
  own.guid = make_identity().guid;
  own.nodes.push_back({"/", "echo", ""});
  fake->fire_message(
    make_notification(
      kServiceId, kOwnInstance, rmw_someip_cpp::kGraphEventId,
      rmw_someip_cpp::encode_graph_message(own)));

  // Only the node publish_local_graph added; "echo" was not taken in.
  EXPECT_FALSE(context->graph_cache.has_node("/", "echo"));
}

/// A malformed graph message must not crash.
TEST_F(ContextImplTest, a_corrupt_graph_message_is_ignored)
{
  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId, {0xDE, 0xAD, 0xBE, 0xEF}));
  fake->fire_message(
    make_notification(kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId, {}));
  SUCCEED();
}

/// Services and clients appear on the graph channel and resolve by method ID.
TEST_F(ContextImplTest, services_are_published_and_resolvable)
{
  rmw_someip_cpp::ParticipantEntitiesInfo info;
  info.instance_id = kPeerInstance;
  info.guid = 0x9999ULL;
  rmw_someip_cpp::EndpointInfo service;
  service.kind = rmw_someip_cpp::EndpointKind::Service;
  service.topic_name = "/add_two_ints";
  service.type_name = "example/srv/AddTwoInts";
  service.method_id = 0x0042;
  info.endpoints.push_back(service);

  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      rmw_someip_cpp::encode_graph_message(info)));

  const auto found = context->find_remote_service("/add_two_ints");
  EXPECT_EQ(kPeerInstance, found.first);
  EXPECT_EQ(0x0042, found.second);

  const auto missing = context->find_remote_service("/nope");
  EXPECT_EQ(0u, missing.first);
}

/// A late subscription to a TRANSIENT_LOCAL publisher above depth 1 gets the
/// history replayed.
TEST_F(ContextImplTest, late_subscribers_get_the_history_replayed)
{
  auto qos = default_qos();
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos.depth = 3;
  auto * publisher = add_publisher("/history", "T", qos);

  for (uint8_t i = 0; i < 3; ++i) {
    const std::vector<uint8_t> payload = {0x00, 0x01, 0x00, 0x00, i};
    publisher->mark_published(payload);
  }

  ASSERT_TRUE(
    fake->fire_subscription(
      kServiceId, kOwnInstance, publisher->eventgroup_id, 0x0077))
    << "no subscription handler was registered";

  // The housekeeping thread handles replay requests and cannot be driven from
  // here, so wait a little; the watchdog ticks every 20 ms.
  for (int i = 0; i < 100 && fake->notified_one.size() < 3; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(3u, fake->notified_one.size());
  EXPECT_EQ(0x0077, fake->notified_one[0].client);
  // Sent oldest first.
  EXPECT_EQ(0, fake->notified_one[0].payload.back());
  EXPECT_EQ(2, fake->notified_one[2].payload.back());
}

/// A late subscriber in the same process gets the history too: neither fields
/// nor replay reach a path that skips SOME/IP.
TEST_F(ContextImplTest, local_late_subscribers_get_the_history)
{
  auto qos = default_qos();
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos.depth = 2;
  auto * publisher = add_publisher("/local_history", "T", qos);
  publisher->mark_published({0x00, 0x01, 0x00, 0x00, 0xA});
  publisher->mark_published({0x00, 0x01, 0x00, 0x00, 0xB});

  auto * subscription = add_subscription("/local_history", "T", qos);
  rmw_someip_cpp::ReceivedMessage message;
  ASSERT_TRUE(subscription->take(message));
  EXPECT_EQ(0xA, message.payload.back());
  ASSERT_TRUE(subscription->take(message));
  EXPECT_EQ(0xB, message.payload.back());
}

/// With ignore_local_publications set, nothing is delivered from this process.
TEST_F(ContextImplTest, local_delivery_honours_ignore_local_publications)
{
  auto * publisher = add_publisher("/local", "T");

  auto * subscription = add_subscription("/local", "T");
  subscription->ignore_local_publications = true;

  context->deliver_locally(publisher, {0x00, 0x01, 0x00, 0x00, 0x01});
  rmw_someip_cpp::ReceivedMessage message;
  EXPECT_FALSE(subscription->take(message));
}

/// Matched events are derived from graph changes.
TEST_F(ContextImplTest, matched_events_follow_the_graph)
{
  auto * subscription = add_subscription("/matched", "T");
  auto event = std::make_unique<rmw_someip_cpp::EventData>(RMW_EVENT_SUBSCRIPTION_MATCHED);
  auto * event_ptr = event.get();
  {
    std::lock_guard<std::mutex> lock(subscription->events_mutex);
    subscription->events.emplace(RMW_EVENT_SUBSCRIPTION_MATCHED, std::move(event));
  }

  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/matched", "T")));

  rmw_matched_status_t status{};
  ASSERT_TRUE(event_ptr->take(status));
  EXPECT_EQ(1, status.current_count);
  EXPECT_EQ(1, status.current_count_change);

  // Drops when the peer leaves.
  fake->fire_availability(kPeerInstance, false);
  ASSERT_TRUE(event_ptr->take(status));
  EXPECT_EQ(0, status.current_count);
  EXPECT_EQ(-1, status.current_count_change);
}

/// A QoS-incompatible peer is not counted as matched but reported incompatible.
TEST_F(ContextImplTest, incompatible_peers_are_reported_not_matched)
{
  auto demanding = default_qos();
  demanding.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  auto * subscription = add_subscription("/incompat", "T", demanding);

  auto incompatible = std::make_unique<rmw_someip_cpp::EventData>(
    RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE);
  auto * incompatible_ptr = incompatible.get();
  {
    std::lock_guard<std::mutex> lock(subscription->events_mutex);
    subscription->events.emplace(
      RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE, std::move(incompatible));
  }

  // The peer is VOLATILE, so a TRANSIENT_LOCAL subscription is incompatible.
  fake->fire_message(
    make_notification(
      kServiceId, kPeerInstance, rmw_someip_cpp::kGraphEventId,
      peer_graph_with_publisher("/incompat", "T")));

  rmw_qos_incompatible_event_status_t status{};
  ASSERT_TRUE(incompatible_ptr->take(status));
  EXPECT_EQ(1, status.total_count);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY, status.last_policy_kind);
  EXPECT_EQ(0u, context->graph_cache.count_matched_publishers("/incompat", demanding));
}

/// Destroying a publisher withdraws both the offer and the subscription handler.
TEST_F(ContextImplTest, removing_a_publisher_stops_the_offer)
{
  auto qos = default_qos();
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  qos.depth = 3;
  auto * publisher = add_publisher("/gone", "T", qos);
  const uint16_t event_id = publisher->event_id;

  context->remove_publisher(publisher);
  ASSERT_EQ(1u, fake->stopped_events.size());
  EXPECT_EQ(event_id, std::get<2>(fake->stopped_events[0]));
  // The handler captures the publisher by raw pointer, so it has to go.
  EXPECT_EQ(1u, fake->cleared_subscription_handlers.size());
  EXPECT_EQ(0u, fake->subscription_handler_count());
}

/// With no endpoint, as after shutdown, registering an entity fails.
TEST(ContextImplWithoutEndpoint, entity_registration_fails_cleanly)
{
  rmw_context_impl_s context(0, make_identity(), nullptr);
  std::string error;
  EXPECT_FALSE(context.start_graph_channel(error));
  EXPECT_FALSE(error.empty());

  rmw_someip_cpp::PublisherData publisher;
  publisher.context = &context;
  publisher.qos = default_qos();
  error.clear();
  EXPECT_FALSE(context.add_publisher(&publisher, error));
  EXPECT_FALSE(error.empty());

  rmw_someip_cpp::SubscriptionData subscription;
  subscription.context = &context;
  subscription.qos = default_qos();
  error.clear();
  EXPECT_FALSE(context.add_subscription(&subscription, error));
  EXPECT_FALSE(error.empty());

  rmw_someip_cpp::ServiceData service;
  service.context = &context;
  error.clear();
  EXPECT_FALSE(context.add_service(&service, error));

  rmw_someip_cpp::ClientData client;
  client.context = &context;
  error.clear();
  EXPECT_FALSE(context.add_client(&client, error));
}

// ---------------------------------------------------------------------------
// Extern mode
// ---------------------------------------------------------------------------

namespace
{

constexpr const char * kExternYaml = R"(
externs:
  /to_ecu:
    direction: publish
    service_id: "0x1234"
    instance_id: "0x0011"
    event_id: "0x8001"
    eventgroup_id: "0x0001"
    serialization: cdr
    reliable: true
    port: "30510"
  /from_ecu:
    direction: subscribe
    service_id: "0x5678"
    instance_id: "0x0022"
    event_id: "0x8002"
    eventgroup_id: "0x0002"
    serialization: someip_native
    reliable: false
)";

/// A context carrying an extern mapping. Every ID comes from the configuration
/// file, so this is a wholly separate path from native mode, which uses the
/// participant's own service.
class ExternContextTest : public ContextImplTest
{
protected:
  void SetUp() override
  {
    ContextImplTest::SetUp();
    const std::string path = "/tmp/rmw_someip_extern_context.yaml";
    {
      std::ofstream out(path);
      out << kExternYaml;
    }
    std::string error;
    ASSERT_TRUE(
      rmw_someip_cpp::ExternMapping::load(path, context->extern_mapping, error)) << error;
    std::remove(path.c_str());
    fake->offered_services.clear();
    fake->offered_events.clear();
    fake->requested_services.clear();
    fake->requested_events.clear();
    fake->subscribed.clear();
  }
};

}  // namespace

TEST_F(ExternContextTest, a_mapped_publisher_offers_the_configured_ids)
{
  auto * publisher = add_publisher("/to_ecu", "std_msgs/msg/String");

  EXPECT_TRUE(publisher->is_extern);
  EXPECT_EQ(rmw_someip_cpp::Serialization::Cdr, publisher->serialization);
  EXPECT_EQ(0x1234, publisher->service_id);
  EXPECT_EQ(0x0011, publisher->instance_id);
  EXPECT_EQ(0x8001, publisher->event_id);

  ASSERT_EQ(1u, fake->offered_services.size());
  EXPECT_EQ(0x1234, fake->offered_services[0].first);
  EXPECT_EQ(0x0011, fake->offered_services[0].second);

  ASSERT_EQ(1u, fake->offered_events.size());
  EXPECT_EQ(0x8001, fake->offered_events[0].event);
  EXPECT_EQ(0x0001, fake->offered_events[0].eventgroup);
  // Always an event: an extern peer does not assume field semantics.
  EXPECT_FALSE(fake->offered_events[0].is_field);
  EXPECT_TRUE(fake->offered_events[0].reliable);
}

TEST_F(ExternContextTest, a_mapped_subscription_requests_the_configured_ids)
{
  auto * subscription = add_subscription("/from_ecu", "std_msgs/msg/String");

  EXPECT_TRUE(subscription->is_extern);
  EXPECT_EQ(rmw_someip_cpp::Serialization::SomeipNative, subscription->serialization);

  ASSERT_EQ(1u, fake->requested_services.size());
  EXPECT_EQ(0x5678, fake->requested_services[0].first);
  EXPECT_EQ(0x0022, fake->requested_services[0].second);

  ASSERT_EQ(1u, fake->requested_events.size());
  EXPECT_EQ(0x8002, fake->requested_events[0].event);
  EXPECT_FALSE(fake->requested_events[0].reliable);

  ASSERT_EQ(1u, fake->subscribed.size());
  EXPECT_EQ(0x0002, fake->subscribed[0].eventgroup);
}

TEST_F(ExternContextTest, two_subscriptions_on_one_mapped_topic_subscribe_once)
{
  add_subscription("/from_ecu", "std_msgs/msg/String");
  add_subscription("/from_ecu", "std_msgs/msg/String");

  // Subscriptions to one eventgroup are refcounted into a single subscribe.
  EXPECT_EQ(1u, fake->subscribed.size());
}

TEST_F(ExternContextTest, a_message_on_a_mapped_event_reaches_the_subscription)
{
  auto * subscription = add_subscription("/from_ecu", "std_msgs/msg/String");

  const std::vector<uint8_t> body {0x01, 0x02, 0x03};
  fake->fire_message(make_notification(0x5678, 0x0022, 0x8002, body));

  ASSERT_TRUE(subscription->has_data());
  rmw_someip_cpp::ReceivedMessage taken;
  ASSERT_TRUE(subscription->take(taken));
  EXPECT_EQ(body, taken.payload);
}

TEST_F(ExternContextTest, an_unmapped_topic_still_uses_the_native_path)
{
  auto * publisher = add_publisher("/plain", "std_msgs/msg/String");
  EXPECT_FALSE(publisher->is_extern);
  EXPECT_EQ(kServiceId, publisher->service_id);
  EXPECT_EQ(kOwnInstance, publisher->instance_id);
}
