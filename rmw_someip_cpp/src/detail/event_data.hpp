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

#ifndef DETAIL__EVENT_DATA_HPP_
#define DETAIL__EVENT_DATA_HPP_

#include <cstddef>
#include <cstdint>
#include <mutex>

#include "rmw/event.h"
#include "rmw/event_callback_type.h"
#include "rmw/events_statuses/events_statuses.h"
#include "rmw/qos_policy_kind.h"

#include "detail/wait_set.hpp"

namespace rmw_someip_cpp
{

/// Backing object for rmw_event_t::data.
///
/// SOME/IP has nothing like the DDS status mechanism, so every event is derived
/// here (docs/design.md §7.1):
///   - matched / incompatible QoS: comparing QoS on the graph channel
///   - liveliness changed: the graph for AUTOMATIC, arrival gaps for MANUAL_BY_TOPIC
///   - deadline / liveliness lost: the context's watchdog watching elapsed time
///
/// This is a cumulative status, not a queue. Like a DDS StatusCondition,
/// rmw_take_event() returns the running total plus the delta since the last
/// read and then clears the delta, so taken is true even when nothing fired.
struct EventData : public Waitable
{
  explicit EventData(rmw_event_type_t event_type)
  : type(event_type) {}

  rmw_event_type_t type;

  /// Record a new current count, for matched and liveliness_changed.
  /// Called on every graph channel update.
  void update_current_count(size_t new_current_count)
  {
    rmw_event_callback_t callback = nullptr;
    const void * user_data = nullptr;
    {
      std::lock_guard<std::mutex> lock(waitable_mutex_);
      if (new_current_count == current_count_) {
        return;
      }
      apply_current_count_locked(new_current_count);
      notify_locked();
      callback = callback_;
      user_data = user_data_;
    }
    // Call the callback outside the lock, so executor work that re-enters
    // does not deadlock on it.
    if (callback != nullptr) {
      callback(user_data, 1);
    }
  }

  /// Record both alive and not-alive counts, for liveliness_changed.
  ///
  /// An AUTOMATIC publisher is alive exactly while it is on the graph, so it
  /// only ever appears in the alive count. MANUAL_BY_TOPIC depends on whether
  /// it wrote within its lease, so it can be not-alive (docs/design.md §21).
  void update_liveliness(size_t alive, size_t not_alive)
  {
    rmw_event_callback_t callback = nullptr;
    const void * user_data = nullptr;
    {
      std::lock_guard<std::mutex> lock(waitable_mutex_);
      if (alive == current_count_ && not_alive == not_alive_count_) {
        return;
      }
      apply_current_count_locked(alive);
      not_alive_count_change_ +=
        static_cast<int32_t>(not_alive) - static_cast<int32_t>(not_alive_count_);
      not_alive_count_ = not_alive;
      notify_locked();
      callback = callback_;
      user_data = user_data_;
    }
    if (callback != nullptr) {
      callback(user_data, 1);
    }
  }

  /// Bump the total, for deadline missed and liveliness lost.
  void increment_total()
  {
    rmw_event_callback_t callback = nullptr;
    const void * user_data = nullptr;
    {
      std::lock_guard<std::mutex> lock(waitable_mutex_);
      ++total_count_;
      ++total_count_change_;
      notify_locked();
      callback = callback_;
      user_data = user_data_;
    }
    if (callback != nullptr) {
      callback(user_data, 1);
    }
  }

  /// Return the current status and clear the deltas.
  ///
  /// rmw_take_event() is required to return the current value even when
  /// nothing changed, so this always succeeds. The status struct differs per
  /// event kind, so the caller picks the overload.
  bool take(rmw_matched_status_t & out)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    out.total_count = total_count_;
    out.total_count_change = total_count_change_;
    out.current_count = current_count_;
    out.current_count_change = current_count_change_;
    reset_changes_locked();
    return true;
  }

  bool take(rmw_liveliness_changed_status_t & out)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    out.alive_count = static_cast<int32_t>(current_count_);
    out.not_alive_count = static_cast<int32_t>(not_alive_count_);
    out.alive_count_change = current_count_change_;
    out.not_alive_count_change = not_alive_count_change_;
    reset_changes_locked();
    return true;
  }

  /// For *_QOS_INCOMPATIBLE: total, delta and the offending policy.
  bool take(rmw_qos_incompatible_event_status_t & out)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    out.total_count = static_cast<int32_t>(total_count_);
    out.total_count_change = static_cast<int32_t>(total_count_change_);
    out.last_policy_kind = last_policy_kind_;
    reset_changes_locked();
    return true;
  }

  /// Record how many peers are currently incompatible.
  ///
  /// total_count counts newly discovered incompatible peers, so only the
  /// increase is added. rmw requires keeping the most recently seen policy.
  void update_incompatible(size_t count, rmw_qos_policy_kind_t policy_kind)
  {
    rmw_event_callback_t callback = nullptr;
    const void * user_data = nullptr;
    {
      std::lock_guard<std::mutex> lock(waitable_mutex_);
      if (count <= current_count_) {
        // Only went down, because a peer left. The total never decreases.
        current_count_ = count;
        return;
      }
      const size_t added = count - current_count_;
      current_count_ = count;
      total_count_ += added;
      total_count_change_ += added;
      last_policy_kind_ = policy_kind;
      notify_locked();
      callback = callback_;
      user_data = user_data_;
    }
    if (callback != nullptr) {
      callback(user_data, 1);
    }
  }

  /// deadline missed and liveliness lost carry only a total and a delta.
  /// The offered, requested and liveliness_lost statuses share this layout.
  bool take_total_only(int32_t & total_count, int32_t & total_count_change)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    total_count = static_cast<int32_t>(total_count_);
    total_count_change = static_cast<int32_t>(total_count_change_);
    reset_changes_locked();
    return true;
  }

  void set_callback(rmw_event_callback_t callback, const void * user_data)
  {
    size_t pending = 0;
    {
      std::lock_guard<std::mutex> lock(waitable_mutex_);
      user_data_ = user_data;
      callback_ = callback;
      pending = total_count_change_;
    }
    // Do not lose changes that happened before the callback was set.
    if (callback != nullptr && pending > 0) {
      callback(user_data, pending);
    }
  }

protected:
  /// Wake the wait set only when there is an unread change.
  bool has_data_locked() const override
  {
    return total_count_change_ != 0 || current_count_change_ != 0 ||
           not_alive_count_change_ != 0;
  }

private:
  /// Move current_count_ to new_count, updating the total and delta.
  void apply_current_count_locked(size_t new_count)
  {
    if (new_count > current_count_) {
      const size_t added = new_count - current_count_;
      total_count_ += added;
      total_count_change_ += added;
      current_count_change_ += static_cast<int32_t>(added);
    } else {
      current_count_change_ -= static_cast<int32_t>(current_count_ - new_count);
    }
    current_count_ = new_count;
  }

  void reset_changes_locked()
  {
    total_count_change_ = 0;
    current_count_change_ = 0;
    not_alive_count_change_ = 0;
  }

  size_t total_count_ {0};
  size_t total_count_change_ {0};
  size_t current_count_ {0};
  int32_t current_count_change_ {0};
  /// The not-alive side of liveliness_changed; nonzero only for MANUAL_BY_TOPIC.
  size_t not_alive_count_ {0};
  int32_t not_alive_count_change_ {0};
  /// Most recently seen offending policy, for *_QOS_INCOMPATIBLE.
  rmw_qos_policy_kind_t last_policy_kind_ {RMW_QOS_POLICY_INVALID};
  rmw_event_callback_t callback_ {nullptr};
  const void * user_data_ {nullptr};
};

/// Can this implementation actually raise this event kind?
///
/// SOME/IP has no counterpart for deadline or liveliness, so those are derived
/// here — by the QoS watchdog in detail/context_impl.cpp and the graph channel.
inline bool event_type_is_supported(rmw_event_type_t event_type)
{
  switch (event_type) {
    case RMW_EVENT_PUBLICATION_MATCHED:
    case RMW_EVENT_SUBSCRIPTION_MATCHED:
    case RMW_EVENT_OFFERED_DEADLINE_MISSED:
    case RMW_EVENT_REQUESTED_DEADLINE_MISSED:
    case RMW_EVENT_LIVELINESS_CHANGED:
    case RMW_EVENT_LIVELINESS_LOST:
    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE:
      return true;
    default:
      return false;
  }
}

}  // namespace rmw_someip_cpp

#endif  // DETAIL__EVENT_DATA_HPP_
