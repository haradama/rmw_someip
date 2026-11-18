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

#ifndef DETAIL__ENTITY_DATA_HPP_
#define DETAIL__ENTITY_DATA_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <vsomeip/vsomeip.hpp>

#include "rmw/event_callback_type.h"
#include "rmw/time.h"
#include "rmw/types.h"

#include "detail/event_data.hpp"
#include "detail/extern_mapping.hpp"
#include "detail/message_pool.hpp"
#include "detail/type_support.hpp"
#include "detail/wait_set.hpp"

struct rmw_context_impl_s;

namespace rmw_someip_cpp
{

/// steady_clock, because a deadline test breaks if the clock can go backwards.
/// Distinct from the reception timestamp rmw reports, which is a system clock.
inline int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// rmw_time_t to nanoseconds, collapsing "no deadline" to 0.
///
/// RMW_DURATION_INFINITE is an explicit infinity and {0, 0} is the default
/// (RMW_QOS_DEADLINE_DEFAULT and friends), which also means infinite; neither
/// is monitored.
inline int64_t finite_duration_ns(const rmw_time_t & duration)
{
  if ((duration.sec == 0 && duration.nsec == 0) ||
    rmw_time_equal(duration, RMW_DURATION_INFINITE))
  {
    return 0;
  }
  return static_cast<int64_t>(duration.sec) * 1000000000LL +
         static_cast<int64_t>(duration.nsec);
}

/// Deadline QoS tracking, shared by publishers and subscriptions.
///
/// SOME/IP has no deadline concept, so elapsed time has to be measured here;
/// the context's QoS watchdog thread polls expired().
///
/// The clock starts at the first write or receipt. Counting from entity
/// creation would report a missed deadline whenever discovery took longer than
/// the deadline itself, even though it was never actually missed. DDS also
/// counts a per-instance deadline from the first sample, so this matches.
class DeadlineTracker
{
public:
  void configure(const rmw_time_t & period)
  {
    period_ns_ = finite_duration_ns(period);
  }

  bool enabled() const {return period_ns_ > 0;}

  /// Report expiry once only, for liveliness lost.
  ///
  /// A deadline counts one miss per period, but liveliness is lost once;
  /// staying lost must not keep incrementing. Fires again only after the next
  /// on_activity.
  void set_one_shot() {one_shot_ = true;}

  /// A write or receipt happened; push the deadline out.
  void on_activity(int64_t now_ns)
  {
    if (period_ns_ == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    next_ns_ = now_ns + period_ns_;
  }

  /// Has the deadline passed? If so, advance to the next one.
  /// \return true if it should fire, at most once per call
  bool expired(int64_t now_ns)
  {
    if (period_ns_ == 0) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (next_ns_ == 0 || now_ns < next_ns_) {
      return false;
    }
    if (one_shot_) {
      // Disarm until the next on_activity; next_ns_ == 0 means "not started".
      next_ns_ = 0;
      return true;
    }
    // Measure the next deadline from the previous one, so the watchdog's
    // polling interval delays the report without shifting the deadline.
    next_ns_ += period_ns_;
    if (next_ns_ <= now_ns) {
      // More than one period behind, because the process was stalled. Firing
      // for every missed period would make one take see a total_count_change
      // above 1, so restart from now instead of catching up.
      next_ns_ = now_ns + period_ns_;
    }
    return true;
  }

private:
  int64_t period_ns_ {0};   ///< set once at creation, read-only afterwards
  bool one_shot_ {false};   ///< likewise
  mutable std::mutex mutex_;
  int64_t next_ns_ {0};     ///< 0 means no activity yet, so not armed
};

/// Backing object for rmw_publisher_t::data.
struct PublisherData
{
  rmw_context_impl_s * context {nullptr};

  std::string topic_name;
  std::string type_name;
  TypeSupportInfo type_support;
  rmw_qos_profile_t qos {};
  rmw_gid_t gid {};

  /// SOME/IP IDs this publisher uses.
  ///
  /// Native mode: service and instance are the participant's; event and
  /// eventgroup are assigned sequentially within it.
  /// Extern mode: all four are pinned by someip_mapping.yaml.
  uint16_t service_id {0};
  uint16_t instance_id {0};
  uint16_t event_id {0};
  uint16_t eventgroup_id {0};

  /// Extern mode, meaning the peer is not a ROS node.
  bool is_extern {false};
  Serialization serialization {Serialization::Cdr};

  /// The node that created this publisher, announced on the graph channel.
  std::string node_namespace;
  std::string node_name;

  /// QoS events on this publisher, one per kind.
  std::mutex events_mutex;
  std::map<rmw_event_type_t, std::unique_ptr<EventData>> events;

  /// Pool behind the loaned-message API.
  MessagePool loans;

  /// Offered deadline; every rmw_publish calls on_activity.
  DeadlineTracker deadline;

  /// MANUAL_BY_TOPIC liveliness lease.
  ///
  /// Unused for AUTOMATIC, where liveliness holds as long as the process does.
  /// MANUAL_BY_TOPIC means writing to the topic asserts liveliness, so both
  /// rmw_publish and rmw_publisher_assert_liveliness() call on_activity.
  DeadlineTracker liveliness;

  /// Send history for TRANSIENT_LOCAL; 0 means no history.
  ///
  /// Kept even at depth 1, because a field does not help a late subscriber
  /// in the same process: the deliver_locally() path never goes through a
  /// SOME/IP subscription and so has no history of its own. It is replayed
  /// from here.
  size_t history_depth {0};

  /// Does a late remote subscriber also need the history replayed?
  ///
  /// Not at depth 1: a vsomeip field delivers the latest value by itself.
  /// A field cannot express depth > 1, so those are offered as ET_EVENT and
  /// the RMW replays them.
  bool replay_to_remote {false};
  std::mutex history_mutex;
  std::deque<std::vector<uint8_t>> history;

  /// One message was sent: satisfy deadline and liveliness, and record history.
  void mark_published(const std::vector<uint8_t> & payload)
  {
    const int64_t now = steady_now_ns();
    deadline.on_activity(now);
    // MANUAL_BY_TOPIC asserts liveliness by writing to the topic, so
    // publishing is itself the assertion.
    liveliness.on_activity(now);
    if (history_depth == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(history_mutex);
    while (history.size() >= history_depth) {
      history.pop_front();
    }
    history.push_back(payload);
  }

  /// Copy of the history, oldest first.
  std::deque<std::vector<uint8_t>> history_snapshot()
  {
    std::lock_guard<std::mutex> lock(history_mutex);
    return history;
  }

  /// Raise OFFERED_DEADLINE_MISSED once. Called by the watchdog.
  void fire_deadline_missed()
  {
    fire(RMW_EVENT_OFFERED_DEADLINE_MISSED);
  }

  /// Raise LIVELINESS_LOST once. Called by the watchdog.
  void fire_liveliness_lost()
  {
    fire(RMW_EVENT_LIVELINESS_LOST);
  }

private:
  void fire(rmw_event_type_t event_type)
  {
    std::lock_guard<std::mutex> lock(events_mutex);
    auto it = events.find(event_type);
    if (it != events.end()) {
      it->second->increment_total();
    }
  }
};

/// A message that arrived but has not been taken yet.
struct ReceivedMessage
{
  std::vector<uint8_t> payload;
  rmw_gid_t publisher_gid {};
  /// Reception time reported by rmw_message_info_t, on the system clock.
  int64_t received_timestamp_ns {0};
  /// For lifespan; on the steady clock so a wall-clock jump cannot affect it.
  int64_t steady_received_ns {0};
};

/// Backing object for rmw_subscription_t::data.
///
/// A vsomeip callback thread pushes; the executor thread takes. Deriving from
/// Waitable is what wakes the wait set on push.
struct SubscriptionData : public Waitable
{
  rmw_context_impl_s * context {nullptr};

  std::string topic_name;
  std::string type_name;
  TypeSupportInfo type_support;
  rmw_qos_profile_t qos {};
  rmw_gid_t gid {};

  std::string node_namespace;
  std::string node_name;

  /// Extern mode, meaning the peer is not a ROS node.
  bool is_extern {false};
  Serialization serialization {Serialization::Cdr};

  /// Ignore messages from publishers in the same context
  /// (rmw_subscription_options_t::ignore_local_publications).
  bool ignore_local_publications {false};

  /// Subscriptions this entity holds, keyed by
  /// (service_id, instance_id, event_id, eventgroup_id).
  ///
  /// Prevents subscribing to the same peer twice; on destruction the context's
  /// refcount is decremented and vsomeip is only unsubscribed on the last one.
  using SubscribeKey = std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>;
  std::set<SubscribeKey> subscribed;

  /// QoS events on this subscription, one per kind.
  std::mutex events_mutex;
  std::map<rmw_event_type_t, std::unique_ptr<EventData>> events;

  /// Pool behind the loaned-message API.
  MessagePool loans;

  /// Requested deadline; every received message calls on_activity.
  DeadlineTracker deadline;

  /// Lifespan; 0 means infinite.
  ///
  /// Measured from reception. DDS measures from the sample's source
  /// timestamp, but these payloads are plain CDR (AUTOSAR in extern mode) with
  /// nowhere to put a send time. Adding a header would break both CDR
  /// compatibility with Fast DDS and byte compatibility with non-ROS peers, so
  /// elapsed-since-receipt is used as an approximation.
  int64_t lifespan_ns {0};

  /// Raise REQUESTED_DEADLINE_MISSED once. Called by the watchdog.
  void fire_deadline_missed()
  {
    std::lock_guard<std::mutex> lock(events_mutex);
    auto it = events.find(RMW_EVENT_REQUESTED_DEADLINE_MISSED);
    if (it != events.end()) {
      it->second->increment_total();
    }
  }

  /// Last arrival time per remote publisher, on the steady clock.
  ///
  /// MANUAL_BY_TOPIC liveliness turns on whether the peer wrote within its
  /// lease. All that has to be known is whether it wrote, so timing arrivals
  /// on the subscribing side is enough and nothing is added to the wire.
  std::mutex liveliness_mutex;
  std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, int64_t> publisher_last_seen;

  void note_publisher_alive(const rmw_gid_t & gid, int64_t now_ns)
  {
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> key{};
    std::memcpy(key.data(), gid.data, RMW_GID_STORAGE_SIZE);
    std::lock_guard<std::mutex> lock(liveliness_mutex);
    publisher_last_seen[key] = now_ns;
  }

  /// Last arrival for this gid, or 0 if nothing was ever received from it.
  int64_t last_seen(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid)
  {
    std::lock_guard<std::mutex> lock(liveliness_mutex);
    auto it = publisher_last_seen.find(gid);
    return (it == publisher_last_seen.end()) ? 0 : it->second;
  }

  /// Called from a vsomeip callback.
  void push(ReceivedMessage message)
  {
    const int64_t now = steady_now_ns();
    message.steady_received_ns = now;
    // The deadline is satisfied on arrival. Waiting until take would report a
    // violation whenever the subscriber is merely busy.
    deadline.on_activity(now);
    // MANUAL_BY_TOPIC liveliness is likewise satisfied by the peer writing.
    note_publisher_alive(message.publisher_gid, now);

    std::lock_guard<std::mutex> lock(waitable_mutex_);
    // KEEP_LAST drops the oldest past the depth. KEEP_ALL still gets a cap,
    // since an unbounded queue eventually exhausts memory.
    const size_t limit = (qos.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL) ?
      kKeepAllLimit :
      (qos.depth > 0 ? qos.depth : 1);
    while (queue_.size() >= limit) {
      queue_.pop_front();
    }
    queue_.push_back(std::move(message));
    notify_locked();
    if (on_new_message_callback_ != nullptr) {
      on_new_message_callback_(on_new_message_user_data_, 1);
    }
  }

  /// Set the callback used by an event-driven executor.
  /// Any messages already queued are reported at once, so those that arrived
  /// before the callback was set are not lost.
  void set_on_new_message_callback(rmw_event_callback_t callback, const void * user_data)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    on_new_message_user_data_ = user_data;
    on_new_message_callback_ = callback;
    if (callback != nullptr && !queue_.empty()) {
      callback(user_data, queue_.size());
    }
  }

  /// Called from the executor thread. Messages past their lifespan are
  /// dropped here.
  bool take(ReceivedMessage & out)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    drop_expired_locked();
    if (queue_.empty()) {
      return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

protected:
  /// Is there an unread message that has not expired?
  ///
  /// has_data_locked() is const and cannot prune the queue. Answering "yes"
  /// while only expired messages sit at the front would make the wait spin, so
  /// this looks for at least one live message; take() does the removal.
  bool has_data_locked() const override
  {
    if (lifespan_ns <= 0) {
      return !queue_.empty();
    }
    const int64_t now = steady_now_ns();
    for (const auto & message : queue_) {
      if (now - message.steady_received_ns < lifespan_ns) {
        return true;
      }
    }
    return false;
  }

private:
  void drop_expired_locked()
  {
    if (lifespan_ns <= 0) {
      return;
    }
    const int64_t now = steady_now_ns();
    while (!queue_.empty() && now - queue_.front().steady_received_ns >= lifespan_ns) {
      queue_.pop_front();
    }
  }

  static constexpr size_t kKeepAllLimit = 10000;
  std::deque<ReceivedMessage> queue_;
  rmw_event_callback_t on_new_message_callback_ {nullptr};
  const void * on_new_message_user_data_ {nullptr};
};

// ---------------------------------------------------------------------------
// service / client
// ---------------------------------------------------------------------------

/// Correlation header prefixed to a service payload.
///
/// SOME/IP correlates request and response natively, but vsomeip assigns the
/// session ID inside send(), so it cannot be read beforehand. Rather than tie
/// rmw_request_id_t to vsomeip internals, a sequence number and the client's
/// gid are carried in the payload.
///
/// The peer is always another instance of this RMW — non-ROS peers go through
/// extern mode — so the format is free to be whatever is convenient.
struct ServiceHeader
{
  static constexpr size_t kSize = sizeof(int64_t) + RMW_GID_STORAGE_SIZE;

  int64_t sequence_number {0};
  uint8_t client_gid[RMW_GID_STORAGE_SIZE] {};

  void write_to(std::vector<uint8_t> & buffer) const
  {
    const auto * seq = reinterpret_cast<const uint8_t *>(&sequence_number);
    buffer.insert(buffer.end(), seq, seq + sizeof(int64_t));
    buffer.insert(buffer.end(), client_gid, client_gid + RMW_GID_STORAGE_SIZE);
  }

  /// \return false if the payload is too short
  bool read_from(const uint8_t * data, size_t size)
  {
    if (size < kSize) {
      return false;
    }
    std::memcpy(&sequence_number, data, sizeof(int64_t));
    std::memcpy(client_gid, data + sizeof(int64_t), RMW_GID_STORAGE_SIZE);
    return true;
  }
};

/// A received request. The original SOME/IP message is kept so that
/// create_response() can inherit its request ID.
struct PendingRequest
{
  std::shared_ptr<vsomeip::message> original;
  std::vector<uint8_t> payload;   ///< CDR body, without the correlation header
  ServiceHeader header;
  int64_t received_timestamp_ns {0};
};

/// Backing object for rmw_service_t::data, mapped onto a SOME/IP method.
struct ServiceData : public Waitable
{
  rmw_context_impl_s * context {nullptr};

  std::string service_name;
  std::string type_name;
  ServiceTypeSupportInfo type_support;
  rmw_qos_profile_t qos {};

  std::string node_namespace;
  std::string node_name;

  /// SOME/IP method ID assigned to this service.
  uint16_t method_id {0};

  void push(PendingRequest request)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    queue_.push_back(std::move(request));
    notify_locked();
    if (on_new_request_callback_ != nullptr) {
      on_new_request_callback_(on_new_request_user_data_, 1);
    }
  }

  bool take(PendingRequest & out)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    if (queue_.empty()) {
      return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  void set_on_new_request_callback(rmw_event_callback_t callback, const void * user_data)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    on_new_request_user_data_ = user_data;
    on_new_request_callback_ = callback;
    if (callback != nullptr && !queue_.empty()) {
      callback(user_data, queue_.size());
    }
  }

  /// Original messages of taken requests, kept so a response can be sent:
  /// create_response() inherits the request ID from them.
  ///
  /// Not discarded once answered. rmw does not require exactly one
  /// response per request, and rclcpp's deferred-response pattern really does
  /// call send_response repeatedly for the same request_header — in
  /// test_rclcpp, add_two_ints_defered_cb_with_handle does so from a
  /// zero-period timer. DDS implementations just send the duplicate response,
  /// so this must not be an error either.
  ///
  /// Instead of discarding, the map is capped and the oldest entries dropped.
  using PendingKey = std::pair<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, int64_t>;

  void remember_request(const PendingKey & key, std::shared_ptr<vsomeip::message> original)
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (awaiting_response_.insert_or_assign(key, std::move(original)).second) {
      pending_order_.push_back(key);
      while (pending_order_.size() > kMaxPendingRequests) {
        awaiting_response_.erase(pending_order_.front());
        pending_order_.pop_front();
      }
    }
  }

  /// \return the original message, or nullptr
  std::shared_ptr<vsomeip::message> find_request(const PendingKey & key) const
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = awaiting_response_.find(key);
    return (it == awaiting_response_.end()) ? nullptr : it->second;
  }

protected:
  bool has_data_locked() const override {return !queue_.empty();}

private:
  static constexpr size_t kMaxPendingRequests = 1024;

  std::deque<PendingRequest> queue_;
  rmw_event_callback_t on_new_request_callback_ {nullptr};
  const void * on_new_request_user_data_ {nullptr};

  mutable std::mutex pending_mutex_;
  std::map<PendingKey, std::shared_ptr<vsomeip::message>> awaiting_response_;
  std::deque<PendingKey> pending_order_;
};

/// A received response.
struct ReceivedResponse
{
  std::vector<uint8_t> payload;   ///< CDR body, without the correlation header
  ServiceHeader header;
  int64_t received_timestamp_ns {0};
};

/// Backing object for rmw_client_t::data.
struct ClientData : public Waitable
{
  rmw_context_impl_s * context {nullptr};

  std::string service_name;
  std::string type_name;
  ServiceTypeSupportInfo type_support;
  rmw_qos_profile_t qos {};
  rmw_gid_t gid {};

  std::string node_namespace;
  std::string node_name;

  /// Sequence number for outgoing requests, assigned by rmw_send_request.
  std::atomic<int64_t> next_sequence_number {1};

  void push(ReceivedResponse response)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    queue_.push_back(std::move(response));
    notify_locked();
    if (on_new_response_callback_ != nullptr) {
      on_new_response_callback_(on_new_response_user_data_, 1);
    }
  }

  bool take(ReceivedResponse & out)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    if (queue_.empty()) {
      return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  void set_on_new_response_callback(rmw_event_callback_t callback, const void * user_data)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    on_new_response_user_data_ = user_data;
    on_new_response_callback_ = callback;
    if (callback != nullptr && !queue_.empty()) {
      callback(user_data, queue_.size());
    }
  }

protected:
  bool has_data_locked() const override {return !queue_.empty();}

private:
  std::deque<ReceivedResponse> queue_;
  rmw_event_callback_t on_new_response_callback_ {nullptr};
  const void * on_new_response_user_data_ {nullptr};
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__ENTITY_DATA_HPP_
