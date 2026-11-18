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

#ifndef DETAIL__TYPE_SUPPORT_HPP_
#define DETAIL__TYPE_SUPPORT_HPP_

#include <string>
#include <vector>

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_hash.h"

namespace rmw_someip_cpp
{

/// A resolved introspection typesupport.
///
/// There is no per-type code generation here. Serialization walks the rosidl
/// introspection description of the message layout at runtime, the same way
/// rmw_cyclonedds_cpp does, so any ROS message works without extra work.
struct TypeSupportInfo
{
  /// True for the C introspection, false for C++. The struct layouts are
  /// identical; only the string representation differs
  /// (rosidl_runtime_c__String vs std::string).
  bool is_c {false};

  /// Points at a rosidl_typesupport_introspection_{c,cpp} MessageMembers.
  const void * members {nullptr};

  /// Type name as "std_msgs/msg/String". Used to match types on the graph channel.
  std::string type_name;

  /// REP-2011 (RIHS) type hash.
  ///
  /// The name alone cannot tell two types apart that share it but differ in
  /// their fields. Carrying the hash on the graph channel lets
  /// `ros2 topic info -v` show the real value. Zero
  /// (version = ROSIDL_TYPE_HASH_VERSION_UNSET) when the typesupport has none.
  rosidl_type_hash_t type_hash {};
};

/// Request and response typesupports of a ROS service.
struct ServiceTypeSupportInfo
{
  TypeSupportInfo request;
  TypeSupportInfo response;
  /// Service type name as "example_interfaces/srv/AddTwoInts".
  std::string type_name;
};

/// Pull the introspection typesupport out of a rosidl_message_type_support_t.
/// \return false on failure, with a message in `error`
bool resolve_type_support(
  const rosidl_message_type_support_t * type_support,
  TypeSupportInfo & out,
  std::string & error);

/// Same for a service's request and response.
bool resolve_service_type_support(
  const rosidl_service_type_support_t * type_support,
  ServiceTypeSupportInfo & out,
  std::string & error);

/// Serialize a ROS message to CDR.
bool serialize_message(
  const TypeSupportInfo & type_support,
  const void * ros_message,
  std::vector<uint8_t> & out,
  std::string & error);

/// Deserialize CDR into a ROS message. `ros_message` must already be initialized.
bool deserialize_message(
  const TypeSupportInfo & type_support,
  const uint8_t * data,
  size_t size,
  void * ros_message,
  std::string & error);

}  // namespace rmw_someip_cpp

#endif  // DETAIL__TYPE_SUPPORT_HPP_
