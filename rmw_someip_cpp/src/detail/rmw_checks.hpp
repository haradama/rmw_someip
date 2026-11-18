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

#ifndef DETAIL__RMW_CHECKS_HPP_
#define DETAIL__RMW_CHECKS_HPP_

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/validate_full_topic_name.h"

#include "rmw_dds_common/qos.hpp"

#include "rmw_someip_cpp/identifier.hpp"

// Argument checking is part of the rmw contract, not a nicety: the
// *_with_bad_arguments tests in test_rmw_implementation pin down which error
// code each bad argument must produce. In particular every function taking a
// handle must verify that handle's implementation_identifier. Skipping it
// means a handle from another implementation gets reinterpreted as the wrong
// type — that produced a double-free SIGSEGV in rmw_destroy_publisher.

/// Check a node handle for null and for a matching implementation identifier.
#define RMW_SOMEIP_CHECK_NODE(node) \
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT); \
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH( \
    node, \
    node->implementation_identifier, \
    rmw_someip_cpp::kIdentifier, \
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION)

/// Same, for functions that return a pointer.
#define RMW_SOMEIP_CHECK_NODE_FOR_NULL_RETURN(node) \
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr); \
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH( \
    node, \
    node->implementation_identifier, \
    rmw_someip_cpp::kIdentifier, \
    return nullptr)

namespace rmw_someip_cpp
{

/// Validate a topic or service name as a fully qualified ROS name.
///
/// Empty, relative ("foo") and whitespace-bearing ("/foo bar") names are all
/// invalid — unless the QoS sets avoid_ros_namespace_conventions, which means
/// the name is a native one that deliberately ignores ROS conventions (used to
/// name a DDS topic directly), and must not be validated.
///
/// \return RMW_RET_OK, or RMW_RET_INVALID_ARGUMENT
inline rmw_ret_t validate_topic_name(
  const char * topic_name, bool avoid_ros_namespace_conventions = false)
{
  if (topic_name == nullptr) {
    RMW_SET_ERROR_MSG("topic name is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (avoid_ros_namespace_conventions) {
    if (topic_name[0] == '\0') {
      RMW_SET_ERROR_MSG("topic name must not be empty");
      return RMW_RET_INVALID_ARGUMENT;
    }
    return RMW_RET_OK;
  }
  int validation_result = RMW_TOPIC_VALID;
  const rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  if (validation_result != RMW_TOPIC_VALID) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "invalid topic name '%s': %s",
      topic_name, rmw_full_topic_name_validation_result_string(validation_result));
    return RMW_RET_INVALID_ARGUMENT;
  }
  return RMW_RET_OK;
}

/// Resolve BEST_AVAILABLE against the QoS of peers already on the graph.
///
/// This is the RMW's job, not rcl's; rmw_dds_common provides the shared
/// utility. Without it rmw_*_get_actual_qos() and *_info_by_topic() would
/// return BEST_AVAILABLE, leaving the caller unable to learn the real setting.
inline void resolve_best_available_for_subscription(
  const rmw_node_t * node, const char * topic_name, rmw_qos_profile_t & qos)
{
  const rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_topic_subscription(
    node, topic_name, &qos, rmw_get_publishers_info_by_topic);
  if (ret != RMW_RET_OK) {
    rmw_reset_error();   // fall back to the default if it cannot be resolved
  }
}

inline void resolve_best_available_for_publisher(
  const rmw_node_t * node, const char * topic_name, rmw_qos_profile_t & qos)
{
  const rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_topic_publisher(
    node, topic_name, &qos, rmw_get_subscriptions_info_by_topic);
  if (ret != RMW_RET_OK) {
    rmw_reset_error();
  }
}

/// Resolve SYSTEM_DEFAULT and UNKNOWN to concrete values.
///
/// rmw_*_get_actual_qos() is contractually required to return the value
/// actually in use, so SYSTEM_DEFAULT must not be passed through.
inline rmw_qos_profile_t resolve_qos(const rmw_qos_profile_t & requested)
{
  rmw_qos_profile_t qos = requested;
  if (qos.history == RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT ||
    qos.history == RMW_QOS_POLICY_HISTORY_UNKNOWN)
  {
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }
  if (qos.depth == 0) {
    qos.depth = 1;
  }
  if (qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
    qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN)
  {
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
    qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
  {
    qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
  if (qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
    qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN)
  {
    qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  }
  return qos;
}

}  // namespace rmw_someip_cpp

#endif  // DETAIL__RMW_CHECKS_HPP_
