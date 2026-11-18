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
// The API that belongs nowhere else: implementation identifier, feature
// queries, QoS compatibility.
//
// Dynamic type support (rmw_serialization_support_init) and log severity are
// unimplemented and return RMW_RET_UNSUPPORTED.

#include <algorithm>
#include <cstring>

#include "rmw/dynamic_message_type_support.h"
#include "rmw/error_handling.h"
#include "rmw/features.h"
#include "rmw/qos_profiles.h"
#include "rmw/rmw.h"

#include "detail/qos_compat.hpp"
#include "rmw_someip_cpp/identifier.hpp"

extern "C"
{
bool
rmw_feature_supported(rmw_feature_t feature)
{
  (void)feature;
  return false;
}

rmw_ret_t
rmw_fini_publisher_allocation(rmw_publisher_allocation_t * allocation)
{
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_fini_publisher_allocation() is not implemented in rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_fini_subscription_allocation(rmw_subscription_allocation_t * allocation)
{
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_fini_subscription_allocation() is not implemented in rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

const char *
rmw_get_implementation_identifier(void)
{
  return rmw_someip_cpp::kIdentifier;
}

const char *
rmw_get_serialization_format(void)
{
  return rmw_someip_cpp::kSerializationFormat;
}

rmw_ret_t
rmw_qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_profile,
  const rmw_qos_profile_t subscription_profile,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(compatibility, RMW_RET_INVALID_ARGUMENT);
  if (reason == nullptr && reason_size != 0u) {
    RMW_SET_ERROR_MSG("reason is null but reason_size is not zero");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (reason != nullptr && reason_size > 0u) {
    reason[0] = '\0';
  }

  // Incompatibility wins; only when compatible is the "cannot be decided"
  // warning considered. The rule lives in detail/qos_compat.hpp so that match
  // counts and the *_QOS_INCOMPATIBLE events always reach the same conclusion.
  const auto incompatibility =
    rmw_someip_cpp::check_incompatible(publisher_profile, subscription_profile);
  const char * message = incompatibility.reason;
  if (message != nullptr) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
  } else {
    message = rmw_someip_cpp::check_warning(publisher_profile, subscription_profile);
    *compatibility = (message != nullptr) ?
      RMW_QOS_COMPATIBILITY_WARNING :
      RMW_QOS_COMPATIBILITY_OK;
  }

  if (message != nullptr && reason != nullptr && reason_size > 0u) {
    const size_t length = std::min(std::strlen(message), reason_size - 1u);
    std::memcpy(reason, message, length);
    reason[length] = '\0';
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_serialization_support_init(
  const char * serialization_lib_name, rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support)
{
  (void)serialization_lib_name;
  (void)allocator;
  (void)serialization_support;
  RMW_SET_ERROR_MSG("rmw_serialization_support_init() is not implemented in rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_set_log_severity(rmw_log_severity_t severity)
{
  (void)severity;
  RMW_SET_ERROR_MSG("rmw_set_log_severity() is not implemented in rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
