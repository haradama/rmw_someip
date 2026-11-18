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

#ifndef DETAIL__SOMEIP_SERIALIZER_HPP_
#define DETAIL__SOMEIP_SERIALIZER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "detail/type_support.hpp"

namespace rmw_someip_cpp
{

/// AUTOSAR SOME/IP Transformer serialization, used in extern mode.
///
/// For peers that are not ROS nodes (an AUTOSAR ECU, say). Byte order,
/// alignment and the encoding of dynamic array lengths all differ from CDR;
/// see the comment at the top of someip_serializer.cpp.
bool serialize_message_someip(
  const TypeSupportInfo & type_support,
  const void * ros_message,
  std::vector<uint8_t> & out,
  std::string & error);

bool deserialize_message_someip(
  const TypeSupportInfo & type_support,
  const uint8_t * data,
  size_t size,
  void * ros_message,
  std::string & error);

}  // namespace rmw_someip_cpp

#endif  // DETAIL__SOMEIP_SERIALIZER_HPP_
