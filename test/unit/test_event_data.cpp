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
// Cumulative QoS event status.
//
// rmw_take_event() does not pop one queued event. Like a DDS StatusCondition
// it returns the running total plus the delta since the last read and clears
// the delta. Getting that wrong spins the executor (docs/design.md §15(e)).

#include <vector>

#include "gtest/gtest.h"

#include "detail/event_data.hpp"

using rmw_someip_cpp::EventData;

TEST(EventData, take_always_succeeds_even_without_a_change)
{
  EventData event(RMW_EVENT_PUBLICATION_MATCHED);
  rmw_matched_status_t status{};
  EXPECT_TRUE(event.take(status));
  EXPECT_EQ(0, status.total_count);
  EXPECT_EQ(0, status.total_count_change);
  EXPECT_EQ(0, status.current_count);
  EXPECT_EQ(0, status.current_count_change);
}

TEST(EventData, matched_accumulates_and_resets_the_delta)
{
  EventData event(RMW_EVENT_PUBLICATION_MATCHED);
  event.update_current_count(2);

  rmw_matched_status_t status{};
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(2, status.total_count);
  EXPECT_EQ(2, status.total_count_change);
  EXPECT_EQ(2, status.current_count);
  EXPECT_EQ(2, status.current_count_change);

  // A second take clears the delta and keeps the total.
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(2, status.total_count);
  EXPECT_EQ(0, status.total_count_change);
  EXPECT_EQ(2, status.current_count);
  EXPECT_EQ(0, status.current_count_change);
}

/// The total never decreases when a peer leaves, as in DDS.
TEST(EventData, matched_decrease_keeps_the_total)
{
  EventData event(RMW_EVENT_SUBSCRIPTION_MATCHED);
  event.update_current_count(3);
  rmw_matched_status_t status{};
  event.take(status);

  event.update_current_count(1);
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(3, status.total_count);
  EXPECT_EQ(0, status.total_count_change);
  EXPECT_EQ(1, status.current_count);
  EXPECT_EQ(-2, status.current_count_change);
}

TEST(EventData, updating_to_the_same_count_is_a_no_op)
{
  EventData event(RMW_EVENT_PUBLICATION_MATCHED);
  event.update_current_count(1);
  rmw_matched_status_t status{};
  event.take(status);

  event.update_current_count(1);
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(0, status.total_count_change);
  EXPECT_EQ(0, status.current_count_change);
}

TEST(EventData, liveliness_tracks_alive_and_not_alive)
{
  EventData event(RMW_EVENT_LIVELINESS_CHANGED);
  event.update_liveliness(1, 0);

  rmw_liveliness_changed_status_t status{};
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(1, status.alive_count);
  EXPECT_EQ(0, status.not_alive_count);
  EXPECT_EQ(1, status.alive_count_change);
  EXPECT_EQ(0, status.not_alive_count_change);

  // Lease expired: alive 1 -> 0, not_alive 0 -> 1.
  event.update_liveliness(0, 1);
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(0, status.alive_count);
  EXPECT_EQ(1, status.not_alive_count);
  EXPECT_EQ(-1, status.alive_count_change);
  EXPECT_EQ(1, status.not_alive_count_change);

  // The peer went away, so not_alive returns to 0.
  event.update_liveliness(0, 0);
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(0, status.not_alive_count);
  EXPECT_EQ(-1, status.not_alive_count_change);
}

TEST(EventData, liveliness_with_no_change_does_not_fire)
{
  EventData event(RMW_EVENT_LIVELINESS_CHANGED);
  event.update_liveliness(1, 1);
  rmw_liveliness_changed_status_t status{};
  event.take(status);

  event.update_liveliness(1, 1);
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(0, status.alive_count_change);
  EXPECT_EQ(0, status.not_alive_count_change);
}

TEST(EventData, increment_total_counts_one_at_a_time)
{
  EventData event(RMW_EVENT_OFFERED_DEADLINE_MISSED);
  event.increment_total();
  event.increment_total();

  int32_t total = 0;
  int32_t change = 0;
  ASSERT_TRUE(event.take_total_only(total, change));
  EXPECT_EQ(2, total);
  EXPECT_EQ(2, change);

  ASSERT_TRUE(event.take_total_only(total, change));
  EXPECT_EQ(2, total);
  EXPECT_EQ(0, change);
}

TEST(EventData, incompatible_qos_keeps_the_last_policy_kind)
{
  EventData event(RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE);
  event.update_incompatible(1, RMW_QOS_POLICY_RELIABILITY);

  rmw_qos_incompatible_event_status_t status{};
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(1, status.total_count);
  EXPECT_EQ(1, status.total_count_change);
  EXPECT_EQ(RMW_QOS_POLICY_RELIABILITY, status.last_policy_kind);

  // More incompatible peers raise the total; the policy is the most recent.
  event.update_incompatible(3, RMW_QOS_POLICY_DURABILITY);
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(3, status.total_count);
  EXPECT_EQ(2, status.total_count_change);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY, status.last_policy_kind);

  // Fewer peers do not lower the total.
  event.update_incompatible(0, RMW_QOS_POLICY_INVALID);
  ASSERT_TRUE(event.take(status));
  EXPECT_EQ(3, status.total_count);
  EXPECT_EQ(0, status.total_count_change);
  EXPECT_EQ(RMW_QOS_POLICY_DURABILITY, status.last_policy_kind);
}

namespace
{
void count_call(const void * user_data, size_t /*number*/)
{
  ++*const_cast<size_t *>(static_cast<const size_t *>(user_data));
}
}  // namespace

/// Every update path reaches the callback.
TEST(EventData, callback_fires_from_every_update_path)
{
  size_t calls = 0;

  EventData matched(RMW_EVENT_PUBLICATION_MATCHED);
  matched.set_callback(count_call, &calls);
  matched.update_current_count(1);
  EXPECT_EQ(1u, calls);

  EventData deadline(RMW_EVENT_OFFERED_DEADLINE_MISSED);
  deadline.set_callback(count_call, &calls);
  deadline.increment_total();
  EXPECT_EQ(2u, calls);

  EventData liveliness(RMW_EVENT_LIVELINESS_CHANGED);
  liveliness.set_callback(count_call, &calls);
  liveliness.update_liveliness(2, 0);
  EXPECT_EQ(3u, calls);

  EventData incompatible(RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE);
  incompatible.set_callback(count_call, &calls);
  incompatible.update_incompatible(1, RMW_QOS_POLICY_RELIABILITY);
  EXPECT_EQ(4u, calls);

  // No change, no call.
  matched.update_current_count(1);
  liveliness.update_liveliness(2, 0);
  incompatible.update_incompatible(0, RMW_QOS_POLICY_INVALID);
  EXPECT_EQ(4u, calls);
}

/// Updates work with no callback set.
TEST(EventData, updates_are_safe_without_a_callback)
{
  EventData event(RMW_EVENT_PUBLICATION_MATCHED);
  event.update_current_count(1);
  event.increment_total();
  event.update_liveliness(2, 1);
  event.update_incompatible(1, RMW_QOS_POLICY_RELIABILITY);
  SUCCEED();
}

/// Setting a callback later still reports the changes accumulated before it.
TEST(EventData, setting_a_callback_replays_pending_changes)
{
  EventData event(RMW_EVENT_PUBLICATION_MATCHED);
  event.update_current_count(3);

  size_t calls = 0;
  event.set_callback(count_call, &calls);
  EXPECT_EQ(1u, calls) << "changes from before the callback were not reported";

  // Later changes arrive as usual.
  event.increment_total();
  EXPECT_EQ(2u, calls);

  // Passing null clears it.
  event.set_callback(nullptr, nullptr);
  event.increment_total();
  EXPECT_EQ(2u, calls);
}

TEST(EventData, supported_event_types)
{
  const rmw_event_type_t supported[] = {
    RMW_EVENT_PUBLICATION_MATCHED,
    RMW_EVENT_SUBSCRIPTION_MATCHED,
    RMW_EVENT_OFFERED_DEADLINE_MISSED,
    RMW_EVENT_REQUESTED_DEADLINE_MISSED,
    RMW_EVENT_LIVELINESS_CHANGED,
    RMW_EVENT_LIVELINESS_LOST,
    RMW_EVENT_OFFERED_QOS_INCOMPATIBLE,
    RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE,
  };
  for (auto type : supported) {
    EXPECT_TRUE(rmw_someip_cpp::event_type_is_supported(type)) << type;
  }

  // Message lost is unsupported: detecting it needs a sender-side sequence
  // number, and changing the wire format would break CDR compatibility
  // (docs/design.md §21).
  EXPECT_FALSE(rmw_someip_cpp::event_type_is_supported(RMW_EVENT_MESSAGE_LOST));
  EXPECT_FALSE(rmw_someip_cpp::event_type_is_supported(RMW_EVENT_INVALID));
}
