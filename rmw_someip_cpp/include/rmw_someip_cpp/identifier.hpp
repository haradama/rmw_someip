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

#ifndef RMW_SOMEIP_CPP__IDENTIFIER_HPP_
#define RMW_SOMEIP_CPP__IDENTIFIER_HPP_

namespace rmw_someip_cpp
{

/// Identifier of this RMW implementation.
///
/// Every rmw handle carries this pointer as its implementation_identifier, and
/// RMW_CHECK_TYPE_IDENTIFIERS_MATCH uses it to reject handles from another
/// implementation. It is compared by pointer, so there must be exactly one
/// instance of the string.
extern const char * const kIdentifier;

/// What rmw_get_serialization_format() returns.
///
/// "cdr", because payloads are plain CDR. That is what keeps rosbag2 and DDS
/// implementations interoperable.
extern const char * const kSerializationFormat;

}  // namespace rmw_someip_cpp

#endif  // RMW_SOMEIP_CPP__IDENTIFIER_HPP_
