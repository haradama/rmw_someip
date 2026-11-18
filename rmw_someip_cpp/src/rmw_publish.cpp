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
// Publishers, mapped onto SOME/IP offer_event and notify.

#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <vector>

#include <vsomeip/vsomeip.hpp>

#include "rcutils/strdup.h"

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

extern "C"
{
rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  RMW_SOMEIP_CHECK_NODE_FOR_NULL_RETURN(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);

  // Empty, relative and whitespace-bearing names are all invalid.
  if (rmw_someip_cpp::validate_topic_name(
      topic_name, qos_profile->avoid_ros_namespace_conventions) != RMW_RET_OK)
  {
    return nullptr;
  }
  if (qos_profile->history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
    qos_profile->reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
    qos_profile->durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
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

  auto * publisher_data = new (std::nothrow) rmw_someip_cpp::PublisherData();
  if (publisher_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the publisher data");
    return nullptr;
  }
  publisher_data->context = context_impl;
  publisher_data->topic_name = topic_name;
  publisher_data->type_name = resolved.type_name;
  publisher_data->type_support = resolved;
  // Resolve BEST_AVAILABLE and SYSTEM_DEFAULT here: get_actual_qos() and
  // *_info_by_topic() must report the values actually in use.
  rmw_qos_profile_t resolved_qos = *qos_profile;
  rmw_someip_cpp::resolve_best_available_for_publisher(node, topic_name, resolved_qos);
  publisher_data->qos = rmw_someip_cpp::resolve_qos(resolved_qos);
  publisher_data->node_namespace = node_data->name_space;
  publisher_data->node_name = node_data->name;

  // Assigns IDs, calls offer_event and updates the graph channel.
  if (!context_impl->add_publisher(publisher_data, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to register the publisher: %s", error.c_str());
    delete publisher_data;
    return nullptr;
  }
  rmw_publisher_t * publisher = rmw_publisher_allocate();
  if (publisher == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the publisher handle");
    context_impl->remove_publisher(publisher_data);
    delete publisher_data;
    return nullptr;
  }

  rcutils_allocator_t allocator = node->context->options.allocator;
  publisher->topic_name = rcutils_strdup(topic_name, allocator);
  if (publisher->topic_name == nullptr) {
    RMW_SET_ERROR_MSG("failed to copy the topic name");
    rmw_publisher_free(publisher);
    context_impl->remove_publisher(publisher_data);
    delete publisher_data;
    return nullptr;
  }

  publisher_data->loans.configure(resolved);

  publisher->implementation_identifier = rmw_someip_cpp::kIdentifier;
  publisher->data = publisher_data;
  publisher->options = *publisher_options;
  // Loans are offered. This is not shared-memory transport.
  publisher->can_loan_messages = publisher_data->loans.available();
  return publisher;
}

rmw_ret_t
rmw_destroy_publisher(rmw_node_t * node, rmw_publisher_t * publisher)
{
  // Check the node's implementation identifier too. Without it, a node from
  // another implementation would be accepted and the caller's retry would
  // double-free.
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  if (publisher_data != nullptr && publisher_data->context != nullptr) {
    publisher_data->context->remove_publisher(publisher_data);
  }
  delete publisher_data;

  rcutils_allocator_t allocator = node->context->options.allocator;
  allocator.deallocate(const_cast<char *>(publisher->topic_name), allocator.state);
  rmw_publisher_free(publisher);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish(
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);

  std::vector<uint8_t> buffer;
  std::string error;
  // Extern mode writes AUTOSAR SOME/IP Transformer: a non-ROS peer does not
  // understand CDR.
  const bool ok = (publisher_data->serialization == rmw_someip_cpp::Serialization::SomeipNative) ?
    rmw_someip_cpp::serialize_message_someip(
    publisher_data->type_support, ros_message, buffer, error) :
    rmw_someip_cpp::serialize_message(
    publisher_data->type_support, ros_message, buffer, error);
  if (!ok) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to serialize: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  auto * endpoint = publisher_data->context->endpoint();
  if (endpoint == nullptr) {
    RMW_SET_ERROR_MSG("the SOME/IP endpoint is not available");
    return RMW_RET_ERROR;
  }
  endpoint->notify(
    publisher_data->service_id, publisher_data->instance_id, publisher_data->event_id,
    buffer.data(), buffer.size());

  // Satisfies the offered deadline and MANUAL_BY_TOPIC liveliness.
  publisher_data->mark_published(buffer);

  // SOME/IP does not return an event to the process that notified it, so
  // subscriptions in this process are handed the message directly.
  publisher_data->context->deliver_locally(publisher_data, buffer);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_serialized_message(
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);

  auto * endpoint = publisher_data->context->endpoint();
  if (endpoint == nullptr) {
    RMW_SET_ERROR_MSG("the SOME/IP endpoint is not available");
    return RMW_RET_ERROR;
  }
  endpoint->notify(
    publisher_data->service_id, publisher_data->instance_id, publisher_data->event_id,
    serialized_message->buffer, serialized_message->buffer_length);

  std::vector<uint8_t> buffer(
    serialized_message->buffer, serialized_message->buffer + serialized_message->buffer_length);
  publisher_data->mark_published(buffer);
  publisher_data->context->deliver_locally(publisher_data, buffer);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_get_actual_qos(const rmw_publisher_t * publisher, rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);
  *qos = publisher_data->qos;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);
  *subscription_count = publisher_data->context->graph_cache.count_matched_subscriptions(
    publisher_data->topic_name, publisher_data->qos);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_wait_for_all_acked(const rmw_publisher_t * publisher, rmw_time_t wait_timeout)
{
  (void)wait_timeout;
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  // BEST_EFFORT has no acknowledgement, and even over TCP vsomeip does not
  // expose delivery confirmation. There is nothing to wait for, so succeed.
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_assert_liveliness(const rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);

  // Updating the local timestamp would stop this publisher's own
  // LIVELINESS_LOST, but would not make it look alive to subscribers: they
  // decide from whether anything arrived from this publisher within the lease,
  // so asserting without sending would still be reported not alive.
  //
  // So a zero-length notification is the assertion. CDR always begins with a
  // 4-byte encapsulation header, so zero length cannot collide with an
  // ordinary message.
  publisher_data->liveliness.on_activity(rmw_someip_cpp::steady_now_ns());

  auto * endpoint = publisher_data->context->endpoint();
  if (endpoint == nullptr) {
    RMW_SET_ERROR_MSG("the SOME/IP endpoint is not available");
    return RMW_RET_ERROR;
  }
  // The zero length is the assertion.
  endpoint->notify(
    publisher_data->service_id, publisher_data->instance_id, publisher_data->event_id,
    nullptr, 0);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publisher_event_init(
  rmw_event_t * rmw_event,
  const rmw_publisher_t * publisher,
  rmw_event_type_t event_type)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
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
rmw_publisher_get_network_flow_endpoints(
  const rmw_publisher_t * publisher,
  rcutils_allocator_t * allocator,
  rmw_network_flow_endpoint_array_t * network_flow_endpoint_array)
{
  (void)publisher;
  (void)allocator;
  (void)network_flow_endpoint_array;
  RMW_SET_ERROR_MSG("network flow endpoints are not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

// -- Loaned messages ----------------------------------------------------------
//
// Not shared-memory transport. vsomeip's local path is a UNIX domain
// socket, so CDR still goes over the wire. The rmw contract is that the caller
// borrows memory from the middleware and says nothing about transport, so
// taking over the application's allocation satisfies it.

rmw_ret_t
rmw_borrow_loaned_message(
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  void ** ros_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  // Lending into a pointer that already points somewhere invites the caller to
  // lose track and never return it. rmw requires it to be empty.
  if (*ros_message != nullptr) {
    RMW_SET_ERROR_MSG("ros_message must point to a null pointer");
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);

  void * message = publisher_data->loans.borrow();
  if (message == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate a loaned message");
    return RMW_RET_BAD_ALLOC;
  }
  *ros_message = message;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_return_loaned_message_from_publisher(
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);

  // Reject a pointer that was never lent. Letting it through would try to free
  // a variable on the application's stack.
  if (!publisher_data->loans.give_back(loaned_message)) {
    RMW_SET_ERROR_MSG("the message was not loaned by this publisher");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_publish_loaned_message(
  const rmw_publisher_t * publisher,
  void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // Sending is the ordinary publish. Even for a loan, what goes on the wire is
  // the CDR encoding, not the memory itself.
  const rmw_ret_t ret = rmw_publish(publisher, ros_message, allocation);

  // The loan ends whether or not the send succeeded; rmw returns ownership on
  // publish.
  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  if (publisher_data != nullptr && !publisher_data->loans.give_back(ros_message)) {
    RMW_SET_ERROR_MSG("the message was not loaned by this publisher");
    return RMW_RET_ERROR;
  }
  return ret;
}
}  // extern "C"
