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
// Subscriptions, mapped onto SOME/IP request_event and subscribe.
//
// What to subscribe to comes from the graph channel: SOME/IP-SD carries no
// type names, so which instance's eventgroup is which topic is only knowable
// there.

#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <vector>

#include "rcutils/strdup.h"
#include "rcutils/types/uint8_array.h"

#include "rmw/dynamic_message_type_support.h"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#include "detail/context_impl.hpp"
#include "detail/entity_data.hpp"
#include "detail/event_data.hpp"
#include "detail/node_data.hpp"
#include "detail/rmw_checks.hpp"
#include "detail/someip_serializer.hpp"
#include "detail/type_support.hpp"
#include "rmw_someip_cpp/identifier.hpp"

namespace
{

rmw_someip_cpp::SubscriptionData * data_of(const rmw_subscription_t * subscription)
{
  return static_cast<rmw_someip_cpp::SubscriptionData *>(subscription->data);
}

void fill_message_info(
  rmw_message_info_t * message_info,
  const rmw_someip_cpp::ReceivedMessage & message)
{
  if (message_info == nullptr) {
    return;
  }
  *message_info = rmw_get_zero_initialized_message_info();
  message_info->received_timestamp = message.received_timestamp_ns;
  // SOME/IP headers carry no send time. Reporting source_timestamp would mean
  // adding a header to the payload, which would break CDR compatibility.
  message_info->source_timestamp = 0;
  message_info->publisher_gid = message.publisher_gid;
  message_info->from_intra_process = false;
}

rmw_ret_t take_impl(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * subscription_data = data_of(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_INVALID_ARGUMENT);

  *taken = false;
  rmw_someip_cpp::ReceivedMessage message;
  if (!subscription_data->take(message)) {
    return RMW_RET_OK;
  }

  std::string error;
  const bool ok =
    (subscription_data->serialization == rmw_someip_cpp::Serialization::SomeipNative) ?
    rmw_someip_cpp::deserialize_message_someip(
    subscription_data->type_support,
    message.payload.data(), message.payload.size(), ros_message, error) :
    rmw_someip_cpp::deserialize_message(
    subscription_data->type_support,
    message.payload.data(), message.payload.size(), ros_message, error);
  if (!ok) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to deserialize: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  fill_message_info(message_info, message);
  *taken = true;
  return RMW_RET_OK;
}

/// Take one message as a loan.
///
/// The only difference from take_impl is who owns the message; the number of
/// deserializations is the same. Not shared-memory transport
/// (docs/design.md §22).
rmw_ret_t take_loaned_impl(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * subscription_data = data_of(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_INVALID_ARGUMENT);

  *taken = false;

  // Borrow only once the queue is known to hold something; borrowing first
  // would cycle through the pool on every empty poll.
  rmw_someip_cpp::ReceivedMessage message;
  if (!subscription_data->take(message)) {
    return RMW_RET_OK;
  }

  void * ros_message = subscription_data->loans.borrow();
  if (ros_message == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate a loaned message");
    return RMW_RET_BAD_ALLOC;
  }

  std::string error;
  const bool ok =
    (subscription_data->serialization == rmw_someip_cpp::Serialization::SomeipNative) ?
    rmw_someip_cpp::deserialize_message_someip(
    subscription_data->type_support,
    message.payload.data(), message.payload.size(), ros_message, error) :
    rmw_someip_cpp::deserialize_message(
    subscription_data->type_support,
    message.payload.data(), message.payload.size(), ros_message, error);
  if (!ok) {
    subscription_data->loans.give_back(ros_message);
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to deserialize: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  (void)allocation;
  fill_message_info(message_info, message);
  *loaned_message = ros_message;
  *taken = true;
  return RMW_RET_OK;
}

rmw_ret_t take_serialized_impl(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * subscription_data = data_of(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_INVALID_ARGUMENT);

  *taken = false;
  rmw_someip_cpp::ReceivedMessage message;
  if (!subscription_data->take(message)) {
    return RMW_RET_OK;
  }

  // The payload is already CDR, so copying is enough.
  if (serialized_message->buffer_capacity < message.payload.size()) {
    if (rcutils_uint8_array_resize(serialized_message, message.payload.size()) !=
      RCUTILS_RET_OK)
    {
      RMW_SET_ERROR_MSG("failed to resize the serialized message buffer");
      return RMW_RET_BAD_ALLOC;
    }
  }
  std::memcpy(serialized_message->buffer, message.payload.data(), message.payload.size());
  serialized_message->buffer_length = message.payload.size();

  fill_message_info(message_info, message);
  *taken = true;
  return RMW_RET_OK;
}

}  // namespace

extern "C"
{
rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_subscription_options_t * subscription_options)
{
  RMW_SOMEIP_CHECK_NODE_FOR_NULL_RETURN(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);

  if (rmw_someip_cpp::validate_topic_name(
      topic_name, qos_policies->avoid_ros_namespace_conventions) != RMW_RET_OK)
  {
    return nullptr;
  }
  if (qos_policies->history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
    qos_policies->reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
    qos_policies->durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
  {
    RMW_SET_ERROR_MSG("unknown QoS policies are not allowed");
    return nullptr;
  }

  auto * context_impl = node->context->impl;
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, nullptr);
  auto * node_data = static_cast<rmw_someip_cpp::NodeData *>(node->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, nullptr);

  std::string error;
  rmw_someip_cpp::TypeSupportInfo resolved;
  if (!rmw_someip_cpp::resolve_type_support(type_support, resolved, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("%s", error.c_str());
    return nullptr;
  }

  auto * subscription_data = new (std::nothrow) rmw_someip_cpp::SubscriptionData();
  if (subscription_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the subscription data");
    return nullptr;
  }
  subscription_data->context = context_impl;
  subscription_data->topic_name = topic_name;
  subscription_data->type_name = resolved.type_name;
  subscription_data->type_support = resolved;
  subscription_data->loans.configure(resolved);
  // Resolve BEST_AVAILABLE and SYSTEM_DEFAULT to concrete values here.
  rmw_qos_profile_t resolved_qos = *qos_policies;
  rmw_someip_cpp::resolve_best_available_for_subscription(node, topic_name, resolved_qos);
  subscription_data->qos = rmw_someip_cpp::resolve_qos(resolved_qos);
  subscription_data->node_namespace = node_data->name_space;
  subscription_data->node_name = node_data->name;
  subscription_data->ignore_local_publications = subscription_options->ignore_local_publications;

  // Updates the graph channel and connects to already known publishers.
  if (!context_impl->add_subscription(subscription_data, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to register the subscription: %s", error.c_str());
    delete subscription_data;
    return nullptr;
  }

  rmw_subscription_t * subscription = rmw_subscription_allocate();
  if (subscription == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the subscription handle");
    context_impl->remove_subscription(subscription_data);
    delete subscription_data;
    return nullptr;
  }

  rcutils_allocator_t allocator = node->context->options.allocator;
  subscription->topic_name = rcutils_strdup(topic_name, allocator);
  if (subscription->topic_name == nullptr) {
    RMW_SET_ERROR_MSG("failed to copy the topic name");
    rmw_subscription_free(subscription);
    context_impl->remove_subscription(subscription_data);
    delete subscription_data;
    return nullptr;
  }

  subscription->implementation_identifier = rmw_someip_cpp::kIdentifier;
  subscription->data = subscription_data;
  subscription->options = *subscription_options;
  // Loans are offered; not shared-memory transport (design.md §22).
  subscription->can_loan_messages = subscription_data->loans.available();
  subscription->is_cft_enabled = false;
  return subscription;
}

rmw_ret_t
rmw_destroy_subscription(rmw_node_t * node, rmw_subscription_t * subscription)
{
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * subscription_data = data_of(subscription);
  if (subscription_data != nullptr && subscription_data->context != nullptr) {
    subscription_data->context->remove_subscription(subscription_data);
  }
  delete subscription_data;

  rcutils_allocator_t allocator = node->context->options.allocator;
  allocator.deallocate(const_cast<char *>(subscription->topic_name), allocator.state);
  rmw_subscription_free(subscription);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  // rmw_take_with_info() requires message_info, so this calls the shared
  // implementation rather than delegating to it.
  return take_impl(subscription, ros_message, taken, nullptr, allocation);
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  return take_impl(subscription, ros_message, taken, message_info, allocation);
}


rmw_ret_t
rmw_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (count == 0u) {
    RMW_SET_ERROR_MSG("count cannot be 0");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (count > message_sequence->capacity || count > message_info_sequence->capacity) {
    RMW_SET_ERROR_MSG("the provided sequence is too small for the requested count");
    return RMW_RET_INVALID_ARGUMENT;
  }

  *taken = 0;
  for (size_t i = 0; i < count; ++i) {
    bool one_taken = false;
    const rmw_ret_t ret = rmw_take_with_info(
      subscription, message_sequence->data[i], &one_taken,
      &message_info_sequence->data[i], allocation);
    if (ret != RMW_RET_OK) {
      return ret;
    }
    if (!one_taken) {
      break;
    }
    ++(*taken);
  }
  message_sequence->size = *taken;
  message_info_sequence->size = *taken;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return take_serialized_impl(subscription, serialized_message, taken, nullptr, allocation);
}

rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  return take_serialized_impl(
    subscription, serialized_message, taken, message_info, allocation);
}


rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * publisher_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_count, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * subscription_data = data_of(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_INVALID_ARGUMENT);
  *publisher_count = subscription_data->context->graph_cache.count_matched_publishers(
    subscription_data->topic_name, subscription_data->qos);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * subscription_data = data_of(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_INVALID_ARGUMENT);
  *qos = subscription_data->qos;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_set_on_new_message_callback(
  rmw_subscription_t * subscription,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  auto * subscription_data = data_of(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_INVALID_ARGUMENT);
  subscription_data->set_on_new_message_callback(callback, user_data);
  return RMW_RET_OK;
}

// -- Events -------------------------------------------------------------------

rmw_ret_t
rmw_subscription_event_init(
  rmw_event_t * rmw_event,
  const rmw_subscription_t * subscription,
  rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * data = static_cast<rmw_someip_cpp::SubscriptionData *>(subscription->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(data, RMW_RET_INVALID_ARGUMENT);

  std::lock_guard<std::mutex> lock(data->events_mutex);
  auto it = data->events.find(event_type);
  if (it == data->events.end()) {
    it = data->events.emplace(
      event_type, std::make_unique<rmw_someip_cpp::EventData>(event_type)).first;
  }

  rmw_event->implementation_identifier = rmw_someip_cpp::kIdentifier;
  rmw_event->data = it->second.get();
  rmw_event->event_type = event_type;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_event(const rmw_event_t * event_handle, void * event_info, bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(event_handle, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_info, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    event_handle,
    event_handle->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * event_data = static_cast<rmw_someip_cpp::EventData *>(event_handle->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(event_data, RMW_RET_INVALID_ARGUMENT);

  // event_info is a different struct per kind, so dispatch on the handle's
  // event_type. EventData keeps one set of counters; this shapes them.
  switch (event_handle->event_type) {
    case RMW_EVENT_PUBLICATION_MATCHED:
    case RMW_EVENT_SUBSCRIPTION_MATCHED: {
        rmw_matched_status_t status{};
        if (!event_data->take(status)) {
          break;
        }
        *static_cast<rmw_matched_status_t *>(event_info) = status;
        *taken = true;
        return RMW_RET_OK;
      }
    case RMW_EVENT_LIVELINESS_CHANGED: {
        rmw_liveliness_changed_status_t status{};
        if (!event_data->take(status)) {
          break;
        }
        *static_cast<rmw_liveliness_changed_status_t *>(event_info) = status;
        *taken = true;
        return RMW_RET_OK;
      }
    case RMW_EVENT_LIVELINESS_LOST: {
        // Only total_count and total_count_change.
        auto * status = static_cast<rmw_liveliness_lost_status_t *>(event_info);
        if (!event_data->take_total_only(status->total_count, status->total_count_change)) {
          break;
        }
        *taken = true;
        return RMW_RET_OK;
      }
    case RMW_EVENT_OFFERED_DEADLINE_MISSED: {
        auto * status = static_cast<rmw_offered_deadline_missed_status_t *>(event_info);
        if (!event_data->take_total_only(status->total_count, status->total_count_change)) {
          break;
        }
        *taken = true;
        return RMW_RET_OK;
      }
    case RMW_EVENT_REQUESTED_DEADLINE_MISSED: {
        auto * status = static_cast<rmw_requested_deadline_missed_status_t *>(event_info);
        if (!event_data->take_total_only(status->total_count, status->total_count_change)) {
          break;
        }
        *taken = true;
        return RMW_RET_OK;
      }
    case RMW_EVENT_OFFERED_QOS_INCOMPATIBLE:
    case RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE: {
        rmw_qos_incompatible_event_status_t status{};
        if (!event_data->take(status)) {
          break;
        }
        *static_cast<rmw_qos_incompatible_event_status_t *>(event_info) = status;
        *taken = true;
        return RMW_RET_OK;
      }
    default:
      // Kinds that never fire (message lost).
      // rmw_event_type_is_supported() is what reports them as unsupported.
      break;
  }

  *taken = false;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_subscription_set_content_filter(
  rmw_subscription_t * subscription,
  const rmw_subscription_content_filter_options_t * options)
{
  // SOME/IP has no counterpart to content filters.
  (void)subscription;
  (void)options;
  RMW_SET_ERROR_MSG("content filtered topics are not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_subscription_get_content_filter(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_subscription_content_filter_options_t * options)
{
  (void)subscription;
  (void)allocator;
  (void)options;
  RMW_SET_ERROR_MSG("content filtered topics are not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_subscription_get_network_flow_endpoints(
  const rmw_subscription_t * subscription,
  rcutils_allocator_t * allocator,
  rmw_network_flow_endpoint_array_t * network_flow_endpoint_array)
{
  (void)subscription;
  (void)allocator;
  (void)network_flow_endpoint_array;
  RMW_SET_ERROR_MSG("network flow endpoints are not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return take_loaned_impl(subscription, loaned_message, taken, nullptr, allocation);
}

rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  return take_loaned_impl(subscription, loaned_message, taken, message_info, allocation);
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    subscription,
    subscription->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * subscription_data = data_of(subscription);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_data, RMW_RET_INVALID_ARGUMENT);

  // Reject a pointer that was never lent. Letting it through would try to free
  // a variable on the application's stack.
  if (!subscription_data->loans.give_back(loaned_message)) {
    RMW_SET_ERROR_MSG("the message was not loaned by this subscription");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_dynamic_message(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)dynamic_message;
  (void)taken;
  (void)allocation;
  RMW_SET_ERROR_MSG("dynamic messages are not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_take_dynamic_message_with_info(
  const rmw_subscription_t * subscription,
  rosidl_dynamic_typesupport_dynamic_data_t * dynamic_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  (void)subscription;
  (void)dynamic_message;
  (void)taken;
  (void)message_info;
  (void)allocation;
  RMW_SET_ERROR_MSG("dynamic messages are not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
