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
// ROS nodes. SOME/IP has no equivalent, so a node is a logical unit announced
// on the graph channel; communication happens per context (per participant).

#include <mutex>
#include <new>
#include <string>

#include "rcutils/strdup.h"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

#include "detail/context_impl.hpp"
#include "detail/node_data.hpp"
#include "rmw_someip_cpp/identifier.hpp"

extern "C"
{
rmw_node_t *
rmw_create_node(rmw_context_t * context, const char * name, const char * namespace_)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(name, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(namespace_, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return nullptr);
  if (context->impl == nullptr || context->impl->is_shutdown) {
    RMW_SET_ERROR_MSG("context has already been shut down");
    return nullptr;
  }

  int validation_result = RMW_NODE_NAME_VALID;
  if (rmw_validate_node_name(name, &validation_result, nullptr) != RMW_RET_OK) {
    return nullptr;
  }
  if (validation_result != RMW_NODE_NAME_VALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "invalid node name: %s", rmw_node_name_validation_result_string(validation_result));
    return nullptr;
  }

  validation_result = RMW_NAMESPACE_VALID;
  if (rmw_validate_namespace(namespace_, &validation_result, nullptr) != RMW_RET_OK) {
    return nullptr;
  }
  if (validation_result != RMW_NAMESPACE_VALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "invalid node namespace: %s", rmw_namespace_validation_result_string(validation_result));
    return nullptr;
  }

  auto * node_data = new (std::nothrow) rmw_someip_cpp::NodeData();
  if (node_data == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the node data");
    return nullptr;
  }
  node_data->name = name;
  node_data->name_space = namespace_;
  node_data->enclave = (context->options.enclave != nullptr) ? context->options.enclave : "";
  node_data->graph_guard_condition_handle.implementation_identifier =
    rmw_someip_cpp::kIdentifier;
  node_data->graph_guard_condition_handle.data = &node_data->graph_guard_condition;
  node_data->graph_guard_condition_handle.context = context;

  rmw_node_t * node = rmw_node_allocate();
  if (node == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the node handle");
    delete node_data;
    return nullptr;
  }

  rcutils_allocator_t & allocator = context->options.allocator;
  node->name = rcutils_strdup(name, allocator);
  node->namespace_ = rcutils_strdup(namespace_, allocator);
  if (node->name == nullptr || node->namespace_ == nullptr) {
    RMW_SET_ERROR_MSG("failed to copy the node name or namespace");
    allocator.deallocate(const_cast<char *>(node->name), allocator.state);
    allocator.deallocate(const_cast<char *>(node->namespace_), allocator.state);
    rmw_node_free(node);
    delete node_data;
    return nullptr;
  }

  node->implementation_identifier = rmw_someip_cpp::kIdentifier;
  node->context = context;
  node->data = node_data;

  // Announce the node to other participants.
  context->impl->add_node(node_data);

  return node;
}

rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * node_data = static_cast<rmw_someip_cpp::NodeData *>(node->data);
  rmw_context_t * context = node->context;

  if (context != nullptr && context->impl != nullptr) {
    context->impl->remove_node(node_data);
  }

  if (context != nullptr) {
    rcutils_allocator_t & allocator = context->options.allocator;
    allocator.deallocate(const_cast<char *>(node->name), allocator.state);
    allocator.deallocate(const_cast<char *>(node->namespace_), allocator.state);
  }
  delete node_data;
  rmw_node_free(node);
  return RMW_RET_OK;
}

const rmw_guard_condition_t *
rmw_node_get_graph_guard_condition(const rmw_node_t * node)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return nullptr);

  auto * node_data = static_cast<rmw_someip_cpp::NodeData *>(node->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(node_data, nullptr);
  return &node_data->graph_guard_condition_handle;
}
}  // extern "C"
