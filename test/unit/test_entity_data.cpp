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
// The queues entities hold, and the correlation header.
//
// A vsomeip callback thread pushes; the executor thread takes. History depth,
// lifespan and the pending-request cap all exist to stop something growing
// without bound, so each is checked at its boundary.

#include <chrono>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rmw/qos_profiles.h"

#include "detail/entity_data.hpp"

using namespace std::chrono_literals;

namespace
{

rmw_someip_cpp::ReceivedMessage message_with(uint8_t marker)
{
  rmw_someip_cpp::ReceivedMessage message;
  message.payload = {marker};
  return message;
}

}  // namespace

// ---------------------------------------------------------------------------
// SubscriptionData
// ---------------------------------------------------------------------------

/// KEEP_LAST drops the oldest past the depth.
TEST(SubscriptionData, keep_last_drops_the_oldest)
{
  rmw_someip_cpp::SubscriptionData subscription;
  subscription.qos = rmw_qos_profile_default;
  subscription.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  subscription.qos.depth = 2;

  for (uint8_t i = 0; i < 5; ++i) {
    subscription.push(message_with(i));
  }

  rmw_someip_cpp::ReceivedMessage out;
  ASSERT_TRUE(subscription.take(out));
  EXPECT_EQ(3, out.payload[0]);
  ASSERT_TRUE(subscription.take(out));
  EXPECT_EQ(4, out.payload[0]);
  EXPECT_FALSE(subscription.take(out));
}

/// A depth of 0 is treated as 1; otherwise nothing could be received.
TEST(SubscriptionData, zero_depth_keeps_one)
{
  rmw_someip_cpp::SubscriptionData subscription;
  subscription.qos = rmw_qos_profile_default;
  subscription.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  subscription.qos.depth = 0;

  subscription.push(message_with(1));
  subscription.push(message_with(2));

  rmw_someip_cpp::ReceivedMessage out;
  ASSERT_TRUE(subscription.take(out));
  EXPECT_EQ(2, out.payload[0]);
  EXPECT_FALSE(subscription.take(out));
}

/// KEEP_ALL is still capped: unbounded growth eventually exhausts memory.
TEST(SubscriptionData, keep_all_still_has_a_ceiling)
{
  rmw_someip_cpp::SubscriptionData subscription;
  subscription.qos = rmw_qos_profile_default;
  subscription.qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  // The cap is 10000; pushing past it must not keep growing.
  for (int i = 0; i < 10050; ++i) {
    subscription.push(message_with(static_cast<uint8_t>(i)));
  }
  size_t taken = 0;
  rmw_someip_cpp::ReceivedMessage out;
  while (subscription.take(out)) {
    ++taken;
  }
  EXPECT_EQ(10000u, taken);
}

/// A message past its lifespan is dropped by take.
TEST(SubscriptionData, expired_messages_are_dropped_on_take)
{
  rmw_someip_cpp::SubscriptionData subscription;
  subscription.qos = rmw_qos_profile_default;
  subscription.qos.depth = 10;
  subscription.lifespan_ns = 20 * 1000 * 1000;  // 20ms

  subscription.push(message_with(1));
  EXPECT_TRUE(subscription.has_data());
  std::this_thread::sleep_for(60ms);

  // Expired, so it neither reports data nor can be taken.
  EXPECT_FALSE(subscription.has_data());
  rmw_someip_cpp::ReceivedMessage out;
  EXPECT_FALSE(subscription.take(out));

  // A fresh one still gets through.
  subscription.push(message_with(2));
  EXPECT_TRUE(subscription.has_data());
  ASSERT_TRUE(subscription.take(out));
  EXPECT_EQ(2, out.payload[0]);
}

/// With lifespan disabled nothing expires.
TEST(SubscriptionData, without_lifespan_nothing_expires)
{
  rmw_someip_cpp::SubscriptionData subscription;
  subscription.qos = rmw_qos_profile_default;
  subscription.qos.depth = 10;
  subscription.push(message_with(1));
  std::this_thread::sleep_for(30ms);
  EXPECT_TRUE(subscription.has_data());
  rmw_someip_cpp::ReceivedMessage out;
  EXPECT_TRUE(subscription.take(out));
}

/// A callback set later is told about everything already queued.
TEST(SubscriptionData, setting_a_callback_replays_the_backlog)
{
  rmw_someip_cpp::SubscriptionData subscription;
  subscription.qos = rmw_qos_profile_default;
  subscription.qos.depth = 10;
  subscription.push(message_with(1));
  subscription.push(message_with(2));

  size_t announced = 0;
  subscription.set_on_new_message_callback(
    [](const void * user_data, size_t number) {
      *const_cast<size_t *>(static_cast<const size_t *>(user_data)) += number;
    },
    &announced);
  EXPECT_EQ(2u, announced);

  subscription.push(message_with(3));
  EXPECT_EQ(3u, announced);

  subscription.set_on_new_message_callback(nullptr, nullptr);
  subscription.push(message_with(4));
  EXPECT_EQ(3u, announced);
}

// ---------------------------------------------------------------------------
// ServiceHeader
// ---------------------------------------------------------------------------

/// Correlation header round trip. A payload too short to hold one is not a
/// request from this RMW and is rejected.
TEST(ServiceHeader, round_trips_and_rejects_short_payloads)
{
  rmw_someip_cpp::ServiceHeader header;
  header.sequence_number = 0x1122334455667788LL;
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    header.client_gid[i] = static_cast<uint8_t>(i);
  }

  std::vector<uint8_t> buffer;
  header.write_to(buffer);
  EXPECT_EQ(rmw_someip_cpp::ServiceHeader::kSize, buffer.size());

  rmw_someip_cpp::ServiceHeader restored;
  ASSERT_TRUE(restored.read_from(buffer.data(), buffer.size()));
  EXPECT_EQ(header.sequence_number, restored.sequence_number);
  EXPECT_EQ(0, std::memcmp(header.client_gid, restored.client_gid, RMW_GID_STORAGE_SIZE));

  for (size_t size = 0; size < rmw_someip_cpp::ServiceHeader::kSize; ++size) {
    EXPECT_FALSE(restored.read_from(buffer.data(), size)) << "size=" << size;
  }
}

// ---------------------------------------------------------------------------
// ServiceData / ClientData
// ---------------------------------------------------------------------------

TEST(ServiceData, queues_requests_and_replays_the_backlog)
{
  rmw_someip_cpp::ServiceData service;
  rmw_someip_cpp::PendingRequest request;
  request.payload = {1, 2, 3};
  service.push(request);
  EXPECT_TRUE(service.has_data());

  size_t announced = 0;
  service.set_on_new_request_callback(
    [](const void * user_data, size_t number) {
      *const_cast<size_t *>(static_cast<const size_t *>(user_data)) += number;
    },
    &announced);
  EXPECT_EQ(1u, announced);

  rmw_someip_cpp::PendingRequest out;
  ASSERT_TRUE(service.take(out));
  EXPECT_EQ(3u, out.payload.size());
  EXPECT_FALSE(service.take(out));

  service.set_on_new_request_callback(nullptr, nullptr);
  service.push(request);
  EXPECT_EQ(1u, announced);
}

/// A pending entry survives being answered.
///
/// rmw does not require exactly one response per request, and rclcpp's
/// deferred-response pattern calls send_response repeatedly for the same
/// request_header (docs/design.md §16(p)). Instead of discarding, the map is
/// capped and the oldest entries dropped.
TEST(ServiceData, remembers_requests_and_evicts_the_oldest)
{
  rmw_someip_cpp::ServiceData service;
  auto message = vsomeip::runtime::get()->create_message();

  std::vector<rmw_someip_cpp::ServiceData::PendingKey> keys;
  for (int i = 0; i < 1030; ++i) {
    rmw_someip_cpp::ServiceData::PendingKey key{};
    key.second = i;
    keys.push_back(key);
    service.remember_request(key, message);
  }

  // The most recent one is still there.
  EXPECT_NE(nullptr, service.find_request(keys.back()));
  // Looking it up repeatedly works, so a repeated response does not fail.
  EXPECT_NE(nullptr, service.find_request(keys.back()));
  // Entries older than the 1024 cap are gone.
  EXPECT_EQ(nullptr, service.find_request(keys.front()));
  // An unknown key gives nullptr.
  rmw_someip_cpp::ServiceData::PendingKey unknown{};
  unknown.second = 999999;
  EXPECT_EQ(nullptr, service.find_request(unknown));
}

/// Recording the same key twice must not grow the ordering list twice.
TEST(ServiceData, remembering_the_same_key_twice_replaces_it)
{
  rmw_someip_cpp::ServiceData service;
  auto message = vsomeip::runtime::get()->create_message();
  rmw_someip_cpp::ServiceData::PendingKey key{};
  key.second = 7;
  service.remember_request(key, message);
  service.remember_request(key, message);
  EXPECT_NE(nullptr, service.find_request(key));
}

TEST(ClientData, queues_responses_and_replays_the_backlog)
{
  rmw_someip_cpp::ClientData client;
  rmw_someip_cpp::ReceivedResponse response;
  response.payload = {9};
  client.push(response);
  EXPECT_TRUE(client.has_data());

  size_t announced = 0;
  client.set_on_new_response_callback(
    [](const void * user_data, size_t number) {
      *const_cast<size_t *>(static_cast<const size_t *>(user_data)) += number;
    },
    &announced);
  EXPECT_EQ(1u, announced);

  rmw_someip_cpp::ReceivedResponse out;
  ASSERT_TRUE(client.take(out));
  EXPECT_EQ(9, out.payload[0]);
  EXPECT_FALSE(client.take(out));

  client.set_on_new_response_callback(nullptr, nullptr);
  client.push(response);
  EXPECT_EQ(1u, announced);
}

/// Sequence numbers start at 1 and increase.
TEST(ClientData, sequence_numbers_increase)
{
  rmw_someip_cpp::ClientData client;
  EXPECT_EQ(1, client.next_sequence_number++);
  EXPECT_EQ(2, client.next_sequence_number++);
}

// ---------------------------------------------------------------------------
// PublisherData
// ---------------------------------------------------------------------------

/// History drops the oldest past the depth.
TEST(PublisherData, history_is_bounded_by_the_depth)
{
  rmw_someip_cpp::PublisherData publisher;
  publisher.history_depth = 2;
  publisher.mark_published({1});
  publisher.mark_published({2});
  publisher.mark_published({3});

  const auto history = publisher.history_snapshot();
  ASSERT_EQ(2u, history.size());
  EXPECT_EQ(2, history[0][0]);
  EXPECT_EQ(3, history[1][0]);
}

TEST(PublisherData, without_history_nothing_is_kept)
{
  rmw_someip_cpp::PublisherData publisher;
  publisher.mark_published({1});
  EXPECT_TRUE(publisher.history_snapshot().empty());
}

/// Raising an event that was never registered must not crash.
TEST(PublisherData, firing_without_registered_events_is_safe)
{
  rmw_someip_cpp::PublisherData publisher;
  publisher.fire_deadline_missed();
  publisher.fire_liveliness_lost();
  rmw_someip_cpp::SubscriptionData subscription;
  subscription.fire_deadline_missed();
  SUCCEED();
}

// ---------------------------------------------------------------------------
// DeadlineTracker
// ---------------------------------------------------------------------------

TEST(DeadlineTracker, is_disabled_for_infinite_and_default_durations)
{
  rmw_someip_cpp::DeadlineTracker tracker;
  EXPECT_FALSE(tracker.enabled());

  tracker.configure({0, 0});
  EXPECT_FALSE(tracker.enabled());

  tracker.configure(RMW_DURATION_INFINITE);
  EXPECT_FALSE(tracker.enabled());
  EXPECT_FALSE(tracker.expired(rmw_someip_cpp::steady_now_ns()));
}

/// With no activity yet, the deadline has not started.
///
/// Counting from entity creation would report a miss that never happened
/// whenever discovery took longer than the deadline.
TEST(DeadlineTracker, does_not_expire_before_the_first_activity)
{
  rmw_someip_cpp::DeadlineTracker tracker;
  tracker.configure({0, 1000000});  // 1ms
  EXPECT_FALSE(tracker.expired(rmw_someip_cpp::steady_now_ns() + 1000000000LL));
}

TEST(DeadlineTracker, expires_once_per_period)
{
  rmw_someip_cpp::DeadlineTracker tracker;
  tracker.configure({1, 0});  // one second
  const int64_t start = 0;
  tracker.on_activity(start);

  EXPECT_FALSE(tracker.expired(start + 999000000LL));
  EXPECT_TRUE(tracker.expired(start + 1000000000LL));
  // The next deadline is measured from the previous one, so it does not drift.
  EXPECT_FALSE(tracker.expired(start + 1500000000LL));
  EXPECT_TRUE(tracker.expired(start + 2000000000LL));
}

/// More than one period behind, it does not catch up. Firing for every missed
/// period would make one take see a total_count_change above 1, which the
/// official tests assert against.
TEST(DeadlineTracker, does_not_catch_up_after_a_long_stall)
{
  rmw_someip_cpp::DeadlineTracker tracker;
  tracker.configure({1, 0});
  tracker.on_activity(0);

  EXPECT_TRUE(tracker.expired(10000000000LL));   // ten seconds later
  EXPECT_FALSE(tracker.expired(10500000000LL));  // the backlog does not fire
  EXPECT_TRUE(tracker.expired(11000000000LL));
}

/// Liveliness lost fires once, when it is lost. Staying lost must not keep
/// incrementing.
TEST(DeadlineTracker, one_shot_fires_once_until_the_next_activity)
{
  rmw_someip_cpp::DeadlineTracker tracker;
  tracker.configure({1, 0});
  tracker.set_one_shot();
  tracker.on_activity(0);

  EXPECT_TRUE(tracker.expired(1000000000LL));
  EXPECT_FALSE(tracker.expired(5000000000LL)) << "kept firing while still lost";

  tracker.on_activity(5000000000LL);
  EXPECT_TRUE(tracker.expired(6000000000LL));
}

TEST(FiniteDuration, converts_and_recognises_the_no_limit_values)
{
  EXPECT_EQ(0, rmw_someip_cpp::finite_duration_ns({0, 0}));
  EXPECT_EQ(0, rmw_someip_cpp::finite_duration_ns(RMW_DURATION_INFINITE));
  EXPECT_EQ(1000000000LL, rmw_someip_cpp::finite_duration_ns({1, 0}));
  EXPECT_EQ(1500000000LL, rmw_someip_cpp::finite_duration_ns({1, 500000000}));
}

// ---------------------------------------------------------------------------
// Service and client arrival callbacks
// ---------------------------------------------------------------------------

namespace
{

void count_call(const void * user_data, size_t number)
{
  auto * total = static_cast<size_t *>(const_cast<void *>(user_data));
  *total += number;
}

}  // namespace

TEST(ServiceDataCallback, a_new_request_notifies_the_registered_callback)
{
  rmw_someip_cpp::ServiceData service;
  size_t reported = 0;
  service.set_on_new_request_callback(count_call, &reported);

  rmw_someip_cpp::PendingRequest request;
  request.header.sequence_number = 7;
  service.push(std::move(request));
  EXPECT_EQ(1u, reported);

  rmw_someip_cpp::PendingRequest taken;
  ASSERT_TRUE(service.take(taken));
  EXPECT_EQ(7, taken.header.sequence_number);

  // Cleared, so no call.
  service.set_on_new_request_callback(nullptr, nullptr);
  service.push(rmw_someip_cpp::PendingRequest{});
  EXPECT_EQ(1u, reported);
}

TEST(ClientDataCallback, a_new_response_notifies_the_registered_callback)
{
  rmw_someip_cpp::ClientData client;
  size_t reported = 0;
  client.set_on_new_response_callback(count_call, &reported);

  rmw_someip_cpp::ReceivedResponse response;
  response.header.sequence_number = 42;
  client.push(std::move(response));
  EXPECT_EQ(1u, reported);

  rmw_someip_cpp::ReceivedResponse taken;
  ASSERT_TRUE(client.take(taken));
  EXPECT_EQ(42, taken.header.sequence_number);

  client.set_on_new_response_callback(nullptr, nullptr);
  client.push(rmw_someip_cpp::ReceivedResponse{});
  EXPECT_EQ(1u, reported);
}
