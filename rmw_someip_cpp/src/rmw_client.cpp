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
// ROS service clients.
//
// The destination (instance_id, method_id) comes from the graph channel.
// Correlation with rmw_request_id_t uses the ServiceHeader at the front of the
// payload; see detail/entity_data.hpp for why.

#include <cstring>
#include <new>
#include <string>
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

rmw_someip_cpp::ClientData * data_of(const rmw_client_t * client)
{
  return static_cast<rmw_someip_cpp::ClientData *>(client->data);
}

}  // namespace

extern "C"
{
rmw_client_t *
rmw_create_client(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_support,
  const char * service_name,
  const rmw_qos_profile_t * qos_policies)
{
  RMW_SOMEIP_CHECK_NODE_FOR_NULL_RETURN(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);

  if (qos_policies->history == RMW_QOS_POLICY_HISTORY_UNKNOWN ||
    qos_policies->reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
    qos_policies->durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
  {
    RMW_SET_ERROR_MSG("unknown QoS policies are not allowed");
    return nullptr;
  }
  if (rmw_someip_cpp::validate_topic_name(
      service_name, qos_policies->avoid_ros_namespace_conventions) != RMW_RET_OK)
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

  auto * client_data = new (std::nothrow) rmw_someip_cpp::ClientData();
  if (client_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the client data");
    return nullptr;
  }
  client_data->context = context_impl;
  client_data->service_name = service_name;
  client_data->type_name = resolved.type_name;
  client_data->type_support = resolved;
  // Services and clients fall back to defaults rather than inspecting the
  // peer's QoS, as rmw_dds_common does.
  client_data->qos = rmw_someip_cpp::resolve_qos(
    rmw_dds_common::qos_profile_update_best_available_for_services(*qos_policies));
  client_data->node_namespace = node_data->name_space;
  client_data->node_name = node_data->name;

  if (!context_impl->add_client(client_data, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to register the client: %s", error.c_str());
    delete client_data;
    return nullptr;
  }

  rmw_client_t * client = rmw_client_allocate();
  if (client == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the client handle");
    context_impl->remove_client(client_data);
    delete client_data;
    return nullptr;
  }

  rcutils_allocator_t allocator = node->context->options.allocator;
  client->service_name = rcutils_strdup(service_name, allocator);
  if (client->service_name == nullptr) {
    RMW_SET_ERROR_MSG("failed to copy the service name");
    rmw_client_free(client);
    context_impl->remove_client(client_data);
    delete client_data;
    return nullptr;
  }

  client->implementation_identifier = rmw_someip_cpp::kIdentifier;
  client->data = client_data;
  return client;
}

rmw_ret_t
rmw_destroy_client(rmw_node_t * node, rmw_client_t * client)
{
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * client_data = data_of(client);
  if (client_data != nullptr && client_data->context != nullptr) {
    client_data->context->remove_client(client_data);
  }
  delete client_data;

  rcutils_allocator_t allocator = node->context->options.allocator;
  allocator.deallocate(const_cast<char *>(client->service_name), allocator.state);
  rmw_client_free(client);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_send_request(const rmw_client_t * client, const void * ros_request, int64_t * sequence_id)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_request, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(sequence_id, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * client_data = data_of(client);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);

  // Resolve the destination from the graph channel: SOME/IP-SD carries no
  // service names, so it is unknown until the server appears there.
  const auto target = client_data->context->find_remote_service(client_data->service_name);
  if (target.first == 0) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "no service server is available for '%s'", client_data->service_name.c_str());
    return RMW_RET_ERROR;
  }

  const int64_t sequence_number = client_data->next_sequence_number++;

  std::vector<uint8_t> buffer;
  rmw_someip_cpp::ServiceHeader header;
  header.sequence_number = sequence_number;
  std::memcpy(header.client_gid, client_data->gid.data, RMW_GID_STORAGE_SIZE);
  header.write_to(buffer);

  std::string error;
  if (!rmw_someip_cpp::serialize_message(
      client_data->type_support.request, ros_request, buffer, error))
  {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to serialize the request: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  auto * endpoint = client_data->context->endpoint();
  if (endpoint == nullptr) {
    RMW_SET_ERROR_MSG("the SOME/IP endpoint is not available");
    return RMW_RET_ERROR;
  }
  endpoint->send_request(
    client_data->context->identity.service_id, target.first, target.second,
    buffer.data(), buffer.size());

  *sequence_id = sequence_number;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_response(
  const rmw_client_t * client,
  rmw_service_info_t * request_header,
  void * ros_response,
  bool * taken)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(request_header, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_response, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(taken, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * client_data = data_of(client);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);

  *taken = false;
  rmw_someip_cpp::ReceivedResponse response;
  if (!client_data->take(response)) {
    return RMW_RET_OK;
  }

  std::string error;
  if (!rmw_someip_cpp::deserialize_message(
      client_data->type_support.response,
      response.payload.data(), response.payload.size(), ros_response, error))
  {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to deserialize the response: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  request_header->source_timestamp = 0;
  request_header->received_timestamp = response.received_timestamp_ns;
  request_header->request_id.sequence_number = response.header.sequence_number;
  std::memcpy(
    request_header->request_id.writer_guid, response.header.client_gid, RMW_GID_STORAGE_SIZE);

  *taken = true;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_client_set_on_new_response_callback(
  rmw_client_t * client,
  rmw_event_callback_t callback,
  const void * user_data)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  auto * client_data = data_of(client);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);
  client_data->set_on_new_response_callback(callback, user_data);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_client_request_publisher_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos, RMW_RET_INVALID_ARGUMENT);
  auto * client_data = data_of(client);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);
  *qos = client_data->qos;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_client_response_subscription_get_actual_qos(
  const rmw_client_t * client,
  rmw_qos_profile_t * qos)
{
  return rmw_client_request_publisher_get_actual_qos(client, qos);
}
}  // extern "C"
