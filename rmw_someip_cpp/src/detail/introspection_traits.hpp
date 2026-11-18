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

#ifndef DETAIL__INTROSPECTION_TRAITS_HPP_
#define DETAIL__INTROSPECTION_TRAITS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rmw_someip_cpp
{

/// Sentinel passed when assigning an empty string.
///
/// rosidl_runtime_c__{String,U16String}__assignn() is specified to return
/// false for a null value pointer, and an empty std::vector's data() may be
/// null. Passing it straight through makes assigning an empty string fail
/// ("failed to assign a wstring field" on the rclpy side).
constexpr uint16_t kEmptyWideString[1] = {0};
constexpr char kEmptyString[1] = {'\0'};

// The C and C++ introspections share the MessageMember/MessageMembers layout
// and differ in only two ways: how strings are represented
// (rosidl_runtime_c__String vs std::string), and resize_function's return type
// (bool vs void). Both provide size_function / get_function / resize_function
// for dynamic arrays, so the traversal itself can be one template with those
// two differences absorbed here.
//
// Shared by the CDR (detail/type_support.cpp) and SOME/IP
// (detail/someip_serializer.cpp) serializers.

struct CTraits
{
  using Members = rosidl_typesupport_introspection_c__MessageMembers;
  using Member = rosidl_typesupport_introspection_c__MessageMember;

  static const char * string_data(const void * field)
  {
    return static_cast<const rosidl_runtime_c__String *>(field)->data;
  }
  static size_t string_size(const void * field)
  {
    return static_cast<const rosidl_runtime_c__String *>(field)->size;
  }
  static bool string_assign(void * field, const char * data, size_t size)
  {
    return rosidl_runtime_c__String__assignn(
      static_cast<rosidl_runtime_c__String *>(field),
      (data != nullptr) ? data : kEmptyString, size);
  }
  static bool resize(const Member & member, void * field, size_t size)
  {
    return member.resize_function(field, size);
  }

  static constexpr size_t kStringSize = sizeof(rosidl_runtime_c__String);
  static constexpr size_t kWideStringSize = sizeof(rosidl_runtime_c__U16String);

  // ROS 2 wstrings are UTF-16, so elements are uint16.
  static const uint16_t * wstring_data(const void * field)
  {
    return static_cast<const rosidl_runtime_c__U16String *>(field)->data;
  }
  static size_t wstring_size(const void * field)
  {
    return static_cast<const rosidl_runtime_c__U16String *>(field)->size;
  }
  static bool wstring_assign(void * field, const uint16_t * data, size_t size)
  {
    return rosidl_runtime_c__U16String__assignn(
      static_cast<rosidl_runtime_c__U16String *>(field),
      (data != nullptr) ? data : kEmptyWideString, size);
  }
};

struct CppTraits
{
  using Members = rosidl_typesupport_introspection_cpp::MessageMembers;
  using Member = rosidl_typesupport_introspection_cpp::MessageMember;

  static const char * string_data(const void * field)
  {
    return static_cast<const std::string *>(field)->data();
  }
  static size_t string_size(const void * field)
  {
    return static_cast<const std::string *>(field)->size();
  }
  static bool string_assign(void * field, const char * data, size_t size)
  {
    static_cast<std::string *>(field)->assign((data != nullptr) ? data : kEmptyString, size);
    return true;
  }
  static bool resize(const Member & member, void * field, size_t size)
  {
    member.resize_function(field, size);
    return true;
  }

  static constexpr size_t kStringSize = sizeof(std::string);
  static constexpr size_t kWideStringSize = sizeof(std::u16string);

  static const uint16_t * wstring_data(const void * field)
  {
    return reinterpret_cast<const uint16_t *>(
      static_cast<const std::u16string *>(field)->data());
  }
  static size_t wstring_size(const void * field)
  {
    return static_cast<const std::u16string *>(field)->size();
  }
  static bool wstring_assign(void * field, const uint16_t * data, size_t size)
  {
    static_cast<std::u16string *>(field)->assign(
      reinterpret_cast<const char16_t *>((data != nullptr) ? data : kEmptyWideString), size);
    return true;
  }
};

/// Width of a primitive in bytes; 0 means it is not a primitive.
inline size_t primitive_size(uint8_t type_id)
{
  switch (type_id) {
    case rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN:
    case rosidl_typesupport_introspection_c__ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT8:
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT8:
    case rosidl_typesupport_introspection_c__ROS_TYPE_CHAR:
      return 1;
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT16:
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT16:
    case rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR:
      return 2;
    case rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT:
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT32:
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT32:
      return 4;
    case rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE:
    case rosidl_typesupport_introspection_c__ROS_TYPE_UINT64:
    case rosidl_typesupport_introspection_c__ROS_TYPE_INT64:
      return 8;
    case rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE:
      return 16;
    default:
      return 0;
  }
}

/// Fixed-size array rather than a dynamic sequence?
template<typename MemberT>
inline bool is_fixed_array(const MemberT & member)
{
  return member.array_size_ > 0 && !member.is_upper_bound_;
}

/// Stride between elements when walking an array without accessor functions.
///
/// rosidl always emits get_(const_)function for arrays, so this path is only
/// reached with a hand-written descriptor — from a dynamic type bridge, or a
/// conversion out of another middleware — that supplies a fixed-size array
/// with no accessors.
///
/// Stepping by the primitive width alone would make every element of a string
/// or nested-struct array (primitive_size 0) point at the first one, so the
/// real size of the element type is returned instead.
///
/// \return 0 when the stride cannot be determined, in which case do not walk
template<typename Traits>
inline size_t element_stride(const typename Traits::Member & member)
{
  const size_t width = primitive_size(member.type_id_);
  if (width != 0) {
    return width;
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_STRING) {
    return Traits::kStringSize;
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING) {
    return Traits::kWideStringSize;
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE &&
    member.members_ != nullptr && member.members_->data != nullptr)
  {
    return static_cast<const typename Traits::Members *>(member.members_->data)->size_of_;
  }
  return 0;
}

}  // namespace rmw_someip_cpp

#endif  // DETAIL__INTROSPECTION_TRAITS_HPP_
