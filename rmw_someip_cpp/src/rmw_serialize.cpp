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
// Serialization, delegating to the generic CDR implementation that walks the
// rosidl introspection at runtime (detail/type_support.cpp).

#include <cstring>
#include <string>
#include <vector>

#include "rcutils/types/uint8_array.h"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/serialized_message.h"

#include "detail/type_support.hpp"
#include "rmw_someip_cpp/identifier.hpp"

extern "C"
{
rmw_ret_t
rmw_serialize(
  const void * ros_message,
  const rosidl_message_type_support_t * type_support,
  rmw_serialized_message_t * serialized_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

  std::string error;
  rmw_someip_cpp::TypeSupportInfo info;
  if (!rmw_someip_cpp::resolve_type_support(type_support, info, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("%s", error.c_str());
    return RMW_RET_ERROR;
  }

  std::vector<uint8_t> buffer;
  if (!rmw_someip_cpp::serialize_message(info, ros_message, buffer, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to serialize: %s", error.c_str());
    return RMW_RET_ERROR;
  }

  if (serialized_message->buffer_capacity < buffer.size()) {
    const rcutils_ret_t ret = rcutils_uint8_array_resize(serialized_message, buffer.size());
    if (ret != RCUTILS_RET_OK) {
      RMW_SET_ERROR_MSG("failed to resize the serialized message buffer");
      return RMW_RET_BAD_ALLOC;
    }
  }
  std::memcpy(serialized_message->buffer, buffer.data(), buffer.size());
  serialized_message->buffer_length = buffer.size();
  return RMW_RET_OK;
}

rmw_ret_t
rmw_deserialize(
  const rmw_serialized_message_t * serialized_message,
  const rosidl_message_type_support_t * type_support,
  void * ros_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

  std::string error;
  rmw_someip_cpp::TypeSupportInfo info;
  if (!rmw_someip_cpp::resolve_type_support(type_support, info, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("%s", error.c_str());
    return RMW_RET_ERROR;
  }

  if (!rmw_someip_cpp::deserialize_message(
      info, serialized_message->buffer, serialized_message->buffer_length, ros_message, error))
  {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("failed to deserialize: %s", error.c_str());
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
rmw_get_serialized_message_size(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  size_t * size)
{
  // Bounding the size without a message means walking every member's bounds,
  // which is undecidable once a member is unbounded. Other RMW implementations
  // leave this unsupported too.
  (void)type_support;
  (void)message_bounds;
  (void)size;
  RMW_SET_ERROR_MSG("rmw_get_serialized_message_size() is not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
