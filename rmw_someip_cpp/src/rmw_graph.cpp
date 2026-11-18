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
// Graph introspection, all of it queries against GraphCache.
//
// SOME/IP-SD carries only numeric IDs, so everything returned here was
// gathered over the graph channel.

#include <algorithm>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/strdup.h"
#include "rcutils/types/string_array.h"

#include "rmw/error_handling.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/names_and_types.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"
#include "rmw/rmw.h"
#include "rmw/qos_profiles.h"
#include "rmw/sanity_checks.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rosidl_runtime_c/type_hash.h"

#include "detail/context_impl.hpp"
#include "detail/entity_data.hpp"
#include "detail/graph_cache.hpp"
#include "detail/node_data.hpp"
#include "detail/rmw_checks.hpp"
#include "rmw_someip_cpp/identifier.hpp"

namespace
{

rmw_context_impl_s * context_of(const rmw_node_t * node)
{
  if (node == nullptr || node->context == nullptr) {
    return nullptr;
  }
  return node->context->impl;
}

/// Fill an rmw_names_and_types_t from a name-to-types map.
rmw_ret_t fill_names_and_types(
  const std::map<std::string, std::vector<std::string>> & source,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * destination)
{
  if (source.empty()) {
    return RMW_RET_OK;
  }
  rmw_ret_t ret = rmw_names_and_types_init(destination, source.size(), allocator);
  if (ret != RMW_RET_OK) {
    // rcutils_string_array_init() leaves size at the requested value even
    // when allocation fails, with only data null. Passing that back would let
    // the caller loop size times over a null pointer, so zero the output.
    *destination = rmw_get_zero_initialized_names_and_types();
    return ret;
  }

  size_t index = 0;
  for (const auto & [name, types] : source) {
    destination->names.data[index] = rcutils_strdup(name.c_str(), *allocator);
    if (destination->names.data[index] == nullptr) {
      goto fail;
    }
    if (rcutils_string_array_init(&destination->types[index], types.size(), allocator) !=
      RCUTILS_RET_OK)
    {
      goto fail;
    }
    for (size_t t = 0; t < types.size(); ++t) {
      destination->types[index].data[t] = rcutils_strdup(types[t].c_str(), *allocator);
      if (destination->types[index].data[t] == nullptr) {
        goto fail;
      }
    }
    ++index;
  }
  return RMW_RET_OK;

fail:
  RMW_SET_ERROR_MSG("failed to allocate the names and types arrays");
  const rmw_ret_t fini_ret = rmw_names_and_types_fini(destination);
  (void)fini_ret;
  return RMW_RET_BAD_ALLOC;
}

/// Fill an rmw_topic_endpoint_info_array_t from GraphCache endpoints.
rmw_ret_t fill_endpoint_info(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  rmw_someip_cpp::EndpointKind kind,
  rmw_endpoint_type_t endpoint_type,
  rmw_topic_endpoint_info_array_t * out)
{
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator is invalid", return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(out, RMW_RET_INVALID_ARGUMENT);
  const rmw_ret_t name_ret = rmw_someip_cpp::validate_topic_name(topic_name);
  if (name_ret != RMW_RET_OK) {
    return name_ret;
  }
  if (rmw_topic_endpoint_info_array_check_zero(out) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto * context = context_of(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  const auto endpoints = context->graph_cache.endpoints_for_topic(topic_name, kind);
  if (endpoints.empty()) {
    return RMW_RET_OK;
  }

  rmw_ret_t ret = rmw_topic_endpoint_info_array_init_with_size(out, endpoints.size(), allocator);
  if (ret != RMW_RET_OK) {
    *out = rmw_get_zero_initialized_topic_endpoint_info_array();
    return ret;
  }

  for (size_t i = 0; i < endpoints.size(); ++i) {
    const auto & endpoint = endpoints[i];
    auto & info = out->info_array[i];

    const rmw_qos_profile_t qos = rmw_someip_cpp::GraphCache::qos_of(endpoint);

    // The graph channel carries the type hash (since v3). A peer that could
    // not obtain one sends the zero value, with version UNSET.
    const rosidl_type_hash_t type_hash = endpoint.type_hash;

    if (rmw_topic_endpoint_info_set_node_name(
        &info, endpoint.node_name.c_str(), allocator) != RMW_RET_OK ||
      rmw_topic_endpoint_info_set_node_namespace(
        &info, endpoint.node_namespace.c_str(), allocator) != RMW_RET_OK ||
      rmw_topic_endpoint_info_set_topic_type(
        &info, endpoint.type_name.c_str(), allocator) != RMW_RET_OK ||
      rmw_topic_endpoint_info_set_topic_type_hash(&info, &type_hash) != RMW_RET_OK ||
      rmw_topic_endpoint_info_set_endpoint_type(&info, endpoint_type) != RMW_RET_OK ||
      rmw_topic_endpoint_info_set_gid(&info, endpoint.gid, RMW_GID_STORAGE_SIZE) != RMW_RET_OK ||
      rmw_topic_endpoint_info_set_qos_profile(&info, &qos) != RMW_RET_OK)
    {
      const rmw_ret_t fini_ret = rmw_topic_endpoint_info_array_fini(out, allocator);
      (void)fini_ret;
      return RMW_RET_ERROR;
    }
  }
  return RMW_RET_OK;
}

/// Body of the *_names_and_types_by_node family, which backs `ros2 node info`.
///
/// rmw requires the node name and namespace to be validated as well.
rmw_ret_t names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_someip_cpp::EndpointKind kind,
  rmw_names_and_types_t * names_and_types)
{
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator is invalid", return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
  if (rmw_names_and_types_check_zero(names_and_types) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  int validation_result = RMW_NODE_NAME_VALID;
  if (rmw_validate_node_name(node_name, &validation_result, nullptr) != RMW_RET_OK) {
    return RMW_RET_ERROR;
  }
  if (validation_result != RMW_NODE_NAME_VALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "invalid node name: %s", rmw_node_name_validation_result_string(validation_result));
    return RMW_RET_INVALID_ARGUMENT;
  }
  validation_result = RMW_NAMESPACE_VALID;
  if (rmw_validate_namespace(node_namespace, &validation_result, nullptr) != RMW_RET_OK) {
    return RMW_RET_ERROR;
  }
  if (validation_result != RMW_NAMESPACE_VALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "invalid node namespace: %s",
      rmw_namespace_validation_result_string(validation_result));
    return RMW_RET_INVALID_ARGUMENT;
  }

  // A query about a node that does not exist has its own error code.
  auto * context = context_of(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  if (!context->graph_cache.has_node(node_namespace, node_name)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "node '%s%s' does not exist", node_namespace, node_name);
    return RMW_RET_NODE_NAME_NON_EXISTENT;
  }

  return fill_names_and_types(
    context->graph_cache.names_and_types_of_node(node_namespace, node_name, kind),
    allocator, names_and_types);
}

/// Shared body of rmw_count_{publishers,subscribers,services,clients}.
///
/// All four validate the name and ask the graph to count; only what is
/// counted differs.
using CountFn = size_t (rmw_someip_cpp::GraphCache::*)(const std::string &) const;

rmw_ret_t count_by_name(
  const rmw_node_t * node,
  const char * name,
  size_t * count,
  CountFn counter)
{
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(count, RMW_RET_INVALID_ARGUMENT);
  const rmw_ret_t name_ret = rmw_someip_cpp::validate_topic_name(name);
  if (name_ret != RMW_RET_OK) {
    return name_ret;
  }
  auto * context = context_of(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  *count = (context->graph_cache.*counter)(name);
  return RMW_RET_OK;
}

}  // namespace

extern "C"
{
rmw_ret_t
rmw_get_node_names(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces)
{
  RMW_SOMEIP_CHECK_NODE(node);
  // The output array is contractually zero-initialized on entry. Filling one
  // that already holds data would leak it for the caller, so check first.
  if (rmw_check_zero_rmw_string_array(node_names) != RMW_RET_OK ||
    rmw_check_zero_rmw_string_array(node_namespaces) != RMW_RET_OK)
  {
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto * context = context_of(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  const auto nodes = context->graph_cache.node_names();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (rcutils_string_array_init(node_names, nodes.size(), &allocator) != RCUTILS_RET_OK ||
    rcutils_string_array_init(node_namespaces, nodes.size(), &allocator) != RCUTILS_RET_OK)
  {
    RMW_SET_ERROR_MSG("failed to allocate the node name arrays");
    return RMW_RET_BAD_ALLOC;
  }
  for (size_t i = 0; i < nodes.size(); ++i) {
    node_names->data[i] = rcutils_strdup(nodes[i].node_name.c_str(), allocator);
    node_namespaces->data[i] = rcutils_strdup(nodes[i].node_namespace.c_str(), allocator);
    if (node_names->data[i] == nullptr || node_namespaces->data[i] == nullptr) {
      RMW_SET_ERROR_MSG("failed to copy a node name");
      return RMW_RET_BAD_ALLOC;
    }
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_node_names_with_enclaves(
  const rmw_node_t * node,
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_string_array_t * enclaves)
{
  if (rmw_check_zero_rmw_string_array(enclaves) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  const rmw_ret_t ret = rmw_get_node_names(node, node_names, node_namespaces);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  auto * context = context_of(node);
  const auto nodes = context->graph_cache.node_names();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (rcutils_string_array_init(enclaves, nodes.size(), &allocator) != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("failed to allocate the enclave array");
    return RMW_RET_BAD_ALLOC;
  }
  for (size_t i = 0; i < nodes.size(); ++i) {
    const char * enclave = nodes[i].enclave.empty() ? "/" : nodes[i].enclave.c_str();
    enclaves->data[i] = rcutils_strdup(enclave, allocator);
    if (enclaves->data[i] == nullptr) {
      RMW_SET_ERROR_MSG("failed to copy an enclave");
      return RMW_RET_BAD_ALLOC;
    }
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_topic_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  (void)no_demangle;
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator is invalid", return RMW_RET_INVALID_ARGUMENT);
  if (rmw_names_and_types_check_zero(topic_names_and_types) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto * context = context_of(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  return fill_names_and_types(
    context->graph_cache.topic_names_and_types(true, true), allocator, topic_names_and_types);
}

rmw_ret_t
rmw_get_service_names_and_types(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * service_names_and_types)
{
  RMW_SOMEIP_CHECK_NODE(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(allocator, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "allocator is invalid", return RMW_RET_INVALID_ARGUMENT);
  if (rmw_names_and_types_check_zero(service_names_and_types) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  auto * context = context_of(node);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);

  return fill_names_and_types(
    context->graph_cache.service_names_and_types(true, true),
    allocator, service_names_and_types);
}

rmw_ret_t
rmw_count_publishers(const rmw_node_t * node, const char * topic_name, size_t * count)
{
  return count_by_name(node, topic_name, count, &rmw_someip_cpp::GraphCache::count_publishers);
}

rmw_ret_t
rmw_count_subscribers(const rmw_node_t * node, const char * topic_name, size_t * count)
{
  return count_by_name(node, topic_name, count, &rmw_someip_cpp::GraphCache::count_subscriptions);
}

rmw_ret_t
rmw_count_services(const rmw_node_t * node, const char * service_name, size_t * count)
{
  return count_by_name(node, service_name, count, &rmw_someip_cpp::GraphCache::count_services);
}

rmw_ret_t
rmw_count_clients(const rmw_node_t * node, const char * service_name, size_t * count)
{
  return count_by_name(node, service_name, count, &rmw_someip_cpp::GraphCache::count_clients);
}

rmw_ret_t
rmw_get_gid_for_publisher(const rmw_publisher_t * publisher, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * publisher_data = static_cast<rmw_someip_cpp::PublisherData *>(publisher->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_data, RMW_RET_INVALID_ARGUMENT);
  *gid = publisher_data->gid;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_gid_for_client(const rmw_client_t * client, rmw_gid_t * gid)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(client, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    client,
    client->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  auto * client_data = static_cast<rmw_someip_cpp::ClientData *>(client->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(client_data, RMW_RET_INVALID_ARGUMENT);
  *gid = client_data->gid;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_compare_gids_equal(const rmw_gid_t * gid1, const rmw_gid_t * gid2, bool * result)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(gid1, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(gid2, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(result, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid1,
    gid1->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    gid2,
    gid2->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  *result = std::memcmp(gid1->data, gid2->data, RMW_GID_STORAGE_SIZE) == 0;
  return RMW_RET_OK;
}

// -- Per-node introspection, backing `ros2 node info` -------------------------
//
// The graph channel carries each endpoint's node, so filtering happens there.
// no_demangle is a DDS-specific notion — whether to strip ROS naming
// conventions — and makes no difference here, where ROS names are carried
// verbatim to begin with.

rmw_ret_t
rmw_get_publisher_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  (void)no_demangle;
  return names_and_types_by_node(
    node, allocator, node_name, node_namespace,
    rmw_someip_cpp::EndpointKind::Publisher, topic_names_and_types);
}

rmw_ret_t
rmw_get_subscriber_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  bool no_demangle,
  rmw_names_and_types_t * topic_names_and_types)
{
  (void)no_demangle;
  return names_and_types_by_node(
    node, allocator, node_name, node_namespace,
    rmw_someip_cpp::EndpointKind::Subscription, topic_names_and_types);
}

rmw_ret_t
rmw_get_service_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  return names_and_types_by_node(
    node, allocator, node_name, node_namespace,
    rmw_someip_cpp::EndpointKind::Service, service_names_and_types);
}

rmw_ret_t
rmw_get_client_names_and_types_by_node(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * node_name,
  const char * node_namespace,
  rmw_names_and_types_t * service_names_and_types)
{
  return names_and_types_by_node(
    node, allocator, node_name, node_namespace,
    rmw_someip_cpp::EndpointKind::Client, service_names_and_types);
}

rmw_ret_t
rmw_get_publishers_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * publishers_info)
{
  (void)no_mangle;
  return fill_endpoint_info(
    node, allocator, topic_name,
    rmw_someip_cpp::EndpointKind::Publisher, RMW_ENDPOINT_PUBLISHER, publishers_info);
}

rmw_ret_t
rmw_get_subscriptions_info_by_topic(
  const rmw_node_t * node,
  rcutils_allocator_t * allocator,
  const char * topic_name,
  bool no_mangle,
  rmw_topic_endpoint_info_array_t * subscriptions_info)
{
  (void)no_mangle;
  return fill_endpoint_info(
    node, allocator, topic_name,
    rmw_someip_cpp::EndpointKind::Subscription, RMW_ENDPOINT_SUBSCRIPTION, subscriptions_info);
}
}  // extern "C"
