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
// Generic CDR serializer that walks the rosidl introspection at runtime.
//
// The C / C++ differences live in CTraits and CppTraits
// (detail/introspection_traits.hpp); the traversal takes them as a template
// parameter.

#include "detail/type_support.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "rcutils/error_handling.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

#include "detail/cdr.hpp"
#include "detail/introspection_traits.hpp"

namespace rmw_someip_cpp
{

namespace
{

/// Normalize an introspection namespace into ROS type-name form.
///
/// C spells it "std_msgs__msg" and C++ "std_msgs::msg". Both have to become
/// "std_msgs/msg", or an rclcpp publisher and an rclpy subscription on the
/// same topic stop matching, since the graph channel compares type names.
///
/// Only "::" and "__" are separators; a lone underscore inside a package name
/// is kept
/// ("example_interfaces__srv" -> "example_interfaces/srv").
std::string normalize_namespace(const char * message_namespace)
{
  std::string result = (message_namespace != nullptr) ? message_namespace : "";
  for (const char * separator : {"::", "__"}) {
    const std::string token = separator;
    for (size_t pos = result.find(token); pos != std::string::npos;
      pos = result.find(token, pos + 1))
    {
      result.replace(pos, token.size(), "/");
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

template<typename Traits>
bool write_members(
  const typename Traits::Members * members,
  const void * ros_message,
  cdr::Writer & writer,
  std::string & error);

template<typename Traits>
bool write_single(
  const typename Traits::Member & member,
  const void * field,
  cdr::Writer & writer,
  std::string & error)
{
  const size_t width = primitive_size(member.type_id_);
  if (width != 0) {
    writer.align(width);
    writer.write_bytes(field, width);
    return true;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_STRING) {
    writer.write_string(Traits::string_data(field), Traits::string_size(field));
    return true;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING) {
    writer.write_wstring(Traits::wstring_data(field), Traits::wstring_size(field));
    return true;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE) {
    const auto * nested =
      static_cast<const typename Traits::Members *>(member.members_->data);
    return write_members<Traits>(nested, field, writer, error);
  }

  error = "unknown introspection type id " + std::to_string(member.type_id_);
  return false;
}

template<typename Traits>
bool write_members(
  const typename Traits::Members * members,
  const void * ros_message,
  cdr::Writer & writer,
  std::string & error)
{
  const auto * base = static_cast<const uint8_t *>(ros_message);

  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const void * field = base + member.offset_;

    if (!member.is_array_) {
      if (!write_single<Traits>(member, field, writer, error)) {
        return false;
      }
      continue;
    }

    // Fixed-size array: no length, just the elements.
    const bool is_fixed = (member.array_size_ > 0 && !member.is_upper_bound_);
    size_t count = member.array_size_;
    if (!is_fixed) {
      if (member.size_function == nullptr) {
        error = std::string("member '") + member.name_ + "' has no size_function";
        return false;
      }
      count = member.size_function(field);
      writer.write<uint32_t>(static_cast<uint32_t>(count));
    }

    const size_t width = primitive_size(member.type_id_);
    // std::vector<bool> is bit-packed and cannot hand out element pointers.
    // rosidl leaves get_const_function null for such types and provides
    // fetch_function, which copies the value out, instead.
    if (!is_fixed && member.get_const_function == nullptr &&
      member.fetch_function == nullptr)
    {
      error = std::string("member '") + member.name_ +
        "' is a sequence but provides neither get_const_function nor fetch_function";
      return false;
    }

    // Copy primitive arrays in one go.
    //
    // Calling write_single() per element means one call per byte for a byte
    // array, which makes latency grow linearly with the payload
    // (docs/design.md §24).
    //
    // Whether elements are contiguous follows from get_const_function being
    // non-null: rosidl leaves it null only for types that are not contiguous
    // (bit-packed std::vector<bool>) and supplies fetch_function there. With
    // no accessor at all, `field` itself can only be used for a fixed-size
    // array — a dynamic sequence's field points at the container, and reading
    // its raw bytes would emit the container's internal pointers.
    if (width != 0 && count > 0) {
      const void * elements = (member.get_const_function != nullptr) ?
        member.get_const_function(field, 0) :
        (is_fixed && member.fetch_function == nullptr ? field : nullptr);
      if (elements != nullptr) {
        writer.align(width);
        writer.write_bytes(elements, width * count);
        continue;
      }
    }

    // Fixed-size array without accessors: step by the element's real size.
    // width is 0 for strings and nested structs, and stepping by that alone
    // would make every element point at the first.
    size_t stride = 0;
    if (member.get_const_function == nullptr && member.fetch_function == nullptr) {
      stride = element_stride<Traits>(member);
      if (stride == 0 && count > 0) {
        error = std::string("member '") + member.name_ +
          "' is an array without accessors and its element size is unknown";
        return false;
      }
    }

    for (size_t index = 0; index < count; ++index) {
      if (member.get_const_function != nullptr) {
        if (!write_single<Traits>(member, member.get_const_function(field, index), writer, error)) {
          return false;
        }
      } else if (member.fetch_function != nullptr) {
        alignas(16) uint8_t scratch[16] {};
        member.fetch_function(field, index, scratch);
        if (!write_single<Traits>(member, scratch, writer, error)) {
          return false;
        }
      } else {
        const void * element = static_cast<const uint8_t *>(field) + stride * index;
        if (!write_single<Traits>(member, element, writer, error)) {
          return false;
        }
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Deserialization
// ---------------------------------------------------------------------------

template<typename Traits>
bool read_members(
  const typename Traits::Members * members,
  void * ros_message,
  cdr::Reader & reader,
  std::string & error);

template<typename Traits>
bool read_single(
  const typename Traits::Member & member,
  void * field,
  cdr::Reader & reader,
  std::string & error)
{
  const size_t width = primitive_size(member.type_id_);
  if (width != 0) {
    reader.align(width);
    if (!reader.read_bytes(field, width)) {
      error = "truncated CDR payload while reading a primitive";
      return false;
    }
    return true;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_STRING) {
    const char * data = nullptr;
    size_t length = 0;
    if (!reader.read_string(data, length)) {
      error = "truncated CDR payload while reading a string";
      return false;
    }
    if (!Traits::string_assign(field, data, length)) {
      error = "failed to assign a string field";
      return false;
    }
    return true;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING) {
    std::vector<uint16_t> characters;
    if (!reader.read_wstring(characters)) {
      error = "truncated CDR payload while reading a wstring";
      return false;
    }
    if (!Traits::wstring_assign(field, characters.data(), characters.size())) {
      error = "failed to assign a wstring field";
      return false;
    }
    return true;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE) {
    const auto * nested =
      static_cast<const typename Traits::Members *>(member.members_->data);
    return read_members<Traits>(nested, field, reader, error);
  }

  error = "unknown introspection type id " + std::to_string(member.type_id_);
  return false;
}

template<typename Traits>
bool read_members(
  const typename Traits::Members * members,
  void * ros_message,
  cdr::Reader & reader,
  std::string & error)
{
  auto * base = static_cast<uint8_t *>(ros_message);

  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    void * field = base + member.offset_;

    if (!member.is_array_) {
      if (!read_single<Traits>(member, field, reader, error)) {
        return false;
      }
      continue;
    }

    const bool is_fixed = (member.array_size_ > 0 && !member.is_upper_bound_);
    size_t count = member.array_size_;
    if (!is_fixed) {
      uint32_t encoded_count = 0;
      if (!reader.read<uint32_t>(encoded_count)) {
        error = "truncated CDR payload while reading a sequence length";
        return false;
      }
      count = encoded_count;
      if (member.is_upper_bound_ && count > member.array_size_) {
        error = std::string("sequence '") + member.name_ + "' exceeds its upper bound";
        return false;
      }
      if (member.resize_function == nullptr) {
        error = std::string("member '") + member.name_ + "' has no resize_function";
        return false;
      }
      if (!Traits::resize(member, field, count)) {
        error = std::string("failed to resize the sequence '") + member.name_ + "'";
        return false;
      }
    }

    const size_t width = primitive_size(member.type_id_);
    if (!is_fixed && member.get_function == nullptr && member.assign_function == nullptr) {
      error = std::string("member '") + member.name_ +
        "' is a sequence but provides neither get_function nor assign_function";
      return false;
    }

    // Bulk read for the same reason as the write side above. resize_function
    // has already run, so the element storage exists.
    if (width != 0 && count > 0) {
      void * elements = (member.get_function != nullptr) ?
        member.get_function(field, 0) :
        (is_fixed && member.assign_function == nullptr ? field : nullptr);
      if (elements != nullptr) {
        reader.align(width);
        if (!reader.read_bytes(elements, width * count)) {
          error = "truncated CDR payload while reading a primitive array";
          return false;
        }
        continue;
      }
    }

    size_t stride = 0;
    if (member.get_function == nullptr && member.assign_function == nullptr) {
      stride = element_stride<Traits>(member);
      if (stride == 0 && count > 0) {
        error = std::string("member '") + member.name_ +
          "' is an array without accessors and its element size is unknown";
        return false;
      }
    }

    for (size_t index = 0; index < count; ++index) {
      if (member.get_function != nullptr) {
        if (!read_single<Traits>(member, member.get_function(field, index), reader, error)) {
          return false;
        }
      } else if (member.assign_function != nullptr) {
        alignas(16) uint8_t scratch[16] {};
        if (!read_single<Traits>(member, scratch, reader, error)) {
          return false;
        }
        member.assign_function(field, index, scratch);
      } else {
        void * element = static_cast<uint8_t *>(field) + stride * index;
        if (!read_single<Traits>(member, element, reader, error)) {
          return false;
        }
      }
    }
  }
  return true;
}

/// Extract the REP-2011 (RIHS) type hash.
///
/// The outer handle rcl passes is a dispatcher and carries the hash. The inner
/// introspection handle usually has it too, but that is not guaranteed, so try
/// outer then inner and fall back to zero
/// (version = ROSIDL_TYPE_HASH_VERSION_UNSET). Not finding one is not an
/// error: typesupports without a type hash exist, dynamic types among them.
template<typename Handle>
rosidl_type_hash_t type_hash_of(const Handle * outer, const Handle * inner)
{
  for (const Handle * candidate : {outer, inner}) {
    if (candidate == nullptr || candidate->get_type_hash_func == nullptr) {
      continue;
    }
    const rosidl_type_hash_t * hash = candidate->get_type_hash_func(candidate);
    if (hash != nullptr) {
      return *hash;
    }
  }
  return rosidl_get_zero_initialized_type_hash();
}

}  // namespace

bool resolve_type_support(
  const rosidl_message_type_support_t * type_support,
  TypeSupportInfo & out,
  std::string & error)
{
  if (type_support == nullptr) {
    error = "type support handle is null";
    return false;
  }

  // A rosidl_typesupport_c/cpp handle is a dispatcher; ask it explicitly for
  // the introspection implementation.
  //
  // get_message_typesupport_handle() sets the rcutils error state when it does
  // not find one. Having no C version but a C++ version is normal, so the
  // state must be cleared before trying the next; otherwise a later
  // RMW_SET_ERROR_MSG is rejected because an error is already set.
  const rosidl_message_type_support_t * handle = get_message_typesupport_handle(
    type_support, rosidl_typesupport_introspection_c__identifier);
  if (handle == nullptr) {
    rcutils_reset_error();
  }
  if (handle != nullptr) {
    out.is_c = true;
    out.members = handle->data;
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(handle->data);
    out.type_name = normalize_namespace(members->message_namespace_) + "/" + members->message_name_;
  } else {
    handle = get_message_typesupport_handle(
      type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (handle == nullptr) {
      rcutils_reset_error();
      error =
        std::string("type support handle '") + type_support->typesupport_identifier +
        "' does not provide an introspection type support";
      return false;
    }
    out.is_c = false;
    out.members = handle->data;
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(handle->data);
    out.type_name = normalize_namespace(members->message_namespace_) + "/" + members->message_name_;
  }

  out.type_hash = type_hash_of(type_support, handle);
  return true;
}

bool resolve_service_type_support(
  const rosidl_service_type_support_t * type_support,
  ServiceTypeSupportInfo & out,
  std::string & error)
{
  if (type_support == nullptr) {
    error = "service type support handle is null";
    return false;
  }

  // As above: clear the rcutils error state before trying the next candidate.
  const rosidl_service_type_support_t * handle = get_service_typesupport_handle(
    type_support, rosidl_typesupport_introspection_c__identifier);
  if (handle == nullptr) {
    rcutils_reset_error();
  }

  const char * service_namespace = nullptr;
  const char * service_name = nullptr;

  if (handle != nullptr) {
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_c__ServiceMembers *>(handle->data);
    out.request.is_c = true;
    out.request.members = members->request_members_;
    out.response.is_c = true;
    out.response.members = members->response_members_;
    service_namespace = members->service_namespace_;
    service_name = members->service_name_;
  } else {
    handle = get_service_typesupport_handle(
      type_support, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (handle == nullptr) {
      rcutils_reset_error();
      error =
        std::string("service type support handle '") + type_support->typesupport_identifier +
        "' does not provide an introspection type support";
      return false;
    }
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(handle->data);
    out.request.is_c = false;
    out.request.members = members->request_members_;
    out.response.is_c = false;
    out.response.members = members->response_members_;
    service_namespace = members->service_namespace_;
    service_name = members->service_name_;
  }

  out.type_name = normalize_namespace(service_namespace) + "/" + service_name;
  out.request.type_name = out.type_name + "_Request";
  out.response.type_name = out.type_name + "_Response";
  // A service has one type hash; it does not distinguish request from response.
  out.request.type_hash = type_hash_of(type_support, handle);
  out.response.type_hash = out.request.type_hash;
  return true;
}

bool serialize_message(
  const TypeSupportInfo & type_support,
  const void * ros_message,
  std::vector<uint8_t> & out,
  std::string & error)
{
  if (type_support.members == nullptr || ros_message == nullptr) {
    error = "null type support or message";
    return false;
  }
  cdr::Writer writer(out);
  if (type_support.is_c) {
    return write_members<CTraits>(
      static_cast<const CTraits::Members *>(type_support.members), ros_message, writer, error);
  }
  return write_members<CppTraits>(
    static_cast<const CppTraits::Members *>(type_support.members), ros_message, writer, error);
}

bool deserialize_message(
  const TypeSupportInfo & type_support,
  const uint8_t * data,
  size_t size,
  void * ros_message,
  std::string & error)
{
  if (type_support.members == nullptr || ros_message == nullptr) {
    error = "null type support or message";
    return false;
  }
  if (size < cdr::kEncapsulationSize) {
    error = "payload is smaller than the CDR encapsulation header";
    return false;
  }
  cdr::Reader reader(data, size);
  if (reader.big_endian()) {
    error = "big endian CDR payloads are not supported yet by rmw_someip_cpp";
    return false;
  }
  if (type_support.is_c) {
    return read_members<CTraits>(
      static_cast<const CTraits::Members *>(type_support.members), ros_message, reader, error);
  }
  return read_members<CppTraits>(
    static_cast<const CppTraits::Members *>(type_support.members), ros_message, reader, error);
}

}  // namespace rmw_someip_cpp
