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
// ROS service servers, mapped onto SOME/IP methods.
//
// DDS implementations build a service out of two topics, one per direction.
// SOME/IP has request/response natively, so the mapping is direct.

#include <cstring>
#include <new>
#include <string>
#include <utility>
#include <vector>

#include <vsomeip/vsomeip.hpp>

#include "rcutils/strdup.h"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#include "detail/context_impl.hpp"
#include "detail/entity_data.hpp"
#include "detail/node_data.hpp"
#include "detail/rmw_checks.hpp"
#include "detail/type_support.hpp"
#include "rmw_someip_cpp/identifier.hpp"

namespace
{

rmw_someip_cpp::ServiceData * data_of(const rmw_service_t * service)
{
  return static_cast<rmw_someip_cpp::ServiceData *>(service->data);
}

}  // namespace

extern "C"
{
rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RMW_SOMEIP_CHECK_NODE_FOR_NULL_RETURN(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);

  if (qos_profile->history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
    qos_profile->reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
    qos_profile->durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
  {
    RMW_SET_ERROR_MSG("unknown QoS policies are not allowed");
    return nullptr;
  }
  if (rmw_someip_cpp::validate_topic_name(
      service_name, qos_profile->avoid_ros_namespace_conventions) != RMW_RET_OK)
  {
    return nullptr;
  }

  auto * context_impl = node->context->impl;
  RMW_CHECK_ARGUMENT_FOR_NULL(context_impl, nullptr);
  auto * node_data = static_cast<rmw_someip_cpp::NodeData *>(node->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, nullptr);

  std::string error;
  rmw_someip_cpp::ServiceTypeSupportInfo resolved;
  if (!rmw_someip_cpp::resolve_service_type_support(type_support, resolved, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("%s", error.c_str());
    return nullptr;
  }

  auto * service_data = new (std::nothrow) rmw_someip_cpp::ServiceData();
  if (service_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the service data");
    return nullptr;
  }
  service_data->context = context_impl;
  service_data->service_name = service_name;
  service_data->type_name = resolved.type_name;
  service_data->type_support = resolved;
  // Services and clients fall back to defaults rather than inspecting the
  // peer's QoS, as rmw_dds_common does.
  service_data->qos = rmw_someip_cpp::resolve_qos(
    rmw_dds_common::qos_profile_update_best_available_for_services(*qos_profile));
  service_data->node_namespace = node_data->name_space;
  service_data->node_name = node_data->name;

  if (!context_impl->add_service(service_data, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to register the service: %s", error.c_str());
    delete service_data;
    return nullptr;
  }

  rmw_service_t * service = rmw_service_allocate();
  if (service == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the service handle");
    context_impl->remove_service(service_data);
    delete service_data;
    return nullptr;
  }

  rcutils_allocator_t allocator = node->context->options.allocator;
  service->service_name = rcutils_strdup(service_name, allocator);
  if (service->service_name == nullptr) {
    RMW_SET_ERROR_MSG("failed to copy the service name");
    rmw_service_free(service);
    context_impl->remove_service(service_data);
    delete service_data;
    return nullptr;
  }

  service->implementation_identifier = rmw_someip_cpp::kIdentifier;
  service->data = service_data;
  return service;
}

rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * service_data = data_of(service);
  if (service_data != nullptr && service_data->context != nullptr) {
    service_data->context->remove_service(service_data);
  }
  delete service_data;

  rcutils_allocator_t allocator = node->context->options.allocator;
  allocator.deallocate(const_cast<char *>(service->service_name), allocator.state);
  rmw_service_free(service);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * service_data = data_of(service);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);

  *taken = false;
  rmw_someip_cpp::PendingRequest request;
  if (!service_data->take(request)) {
    return RMW_RET_OK;
  }

  std::string error;
  if (!rmw_someip_cpp::deserialize_message(
      service_data->type_support.request,
      request.payload.data(), request.payload.size(), ros_request, error))
  {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to deserialize the request: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  request_header->source_timestamp = 0;
  request_header->received_timestamp = request.received_timestamp_ns;
  request_header->request_id.sequence_number = request.header.sequence_number;
  std::memcpy(
    request_header->request_id.writer_guid, request.header.client_gid, RMW_GID_STORAGE_SIZE);

  // Keep the original SOME/IP message so rmw_send_response() can reply.
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> key{};
  std::memcpy(key.data(), request.header.client_gid, RMW_GID_STORAGE_SIZE);
  service_data->remember_request({key, request.header.sequence_number}, request.original);

  *taken = true;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * service_data = data_of(service);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);

  std::array<uint8_t, RMW_GID_STORAGE_SIZE> key{};
  std::memcpy(key.data(), request_header->writer_guid, RMW_GID_STORAGE_SIZE);

  auto original = service_data->find_request({key, request_header->sequence_number});
  if (!original) {
    RMW_SET_ERROR_MSG("no pending request matches the given request header");
    return RMW_RET_ERROR;
  }

  std::vector<uint8_t> buffer;
  rmw_someip_cpp::ServiceHeader header;
  header.sequence_number = request_header->sequence_number;
  std::memcpy(header.client_gid, request_header->writer_guid, RMW_GID_STORAGE_SIZE);
  header.write_to(buffer);

  std::string error;
  if (!rmw_someip_cpp::serialize_message(
      service_data->type_support.response, ros_response, buffer, error))
  {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to serialize the response: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  auto * endpoint = service_data->context->endpoint();
  if (endpoint == nullptr) {
    RMW_SET_ERROR_MSG("the SOME/IP endpoint is not available");
    return RMW_RET_ERROR;
  }
  endpoint->send_response(original, buffer.data(), buffer.size());
  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * is_available)
{
  // rmw requires ERROR rather than INVALID_ARGUMENT for a null argument here.
  if (node == nullptr || client == nullptr || is_available == nullptr) {
    RMW_SET_ERROR_MSG("node, client and is_available must not be null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * client_data = static_cast<rmw_someip_cpp::ClientData *>(client->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);

  // Answered from whether a server of that name is on the graph channel.
  const auto found = client_data->context->find_remote_service(client_data->service_name);
  *is_available = (found.first != 0);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_set_on_new_request_callback(
  rmw_service_t * service,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  auto * service_data = data_of(service);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);
  service_data->set_on_new_request_callback(callback, user_data);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_request_subscription_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  auto * service_data = data_of(service);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_data, RMW_RET_INVALID_ARGUMENT);
  *qos = service_data->qos;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_service_response_publisher_get_actual_qos(
  const rmw_service_t * service,
  rmw_qos_profile_t * qos)
{
  return rmw_service_request_subscription_get_actual_qos(service, qos);
}
}  // extern "C"
