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

/// How the serializers behave with hand-written introspection descriptors.
///
/// A descriptor rosidl generates always satisfies certain properties — arrays
/// carry accessor functions, type_ids are known — so real messages alone never
/// reach the serializers' defensive branches. But descriptors can be written by
/// hand, which is exactly what dynamic type support and conversions from other
/// middleware do, so a malformed one, or one in a shape rosidl never produces,
/// has to return an error rather than crash.
///
/// The descriptors are built directly here. TypeSupportInfo only holds resolved
/// MessageMembers, so no typesupport handle is needed.

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/u16string_functions.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "detail/someip_serializer.hpp"
#include "detail/type_support.hpp"

using rmw_someip_cpp::TypeSupportInfo;
using rmw_someip_cpp::deserialize_message;
using rmw_someip_cpp::deserialize_message_someip;
using rmw_someip_cpp::serialize_message;
using rmw_someip_cpp::serialize_message_someip;

using CMember = rosidl_typesupport_introspection_c__MessageMember;
using CMembers = rosidl_typesupport_introspection_c__MessageMembers;

namespace
{

CMember scalar(const char * name, uint8_t type_id, uint32_t offset)
{
  CMember member {};
  member.name_ = name;
  member.type_id_ = type_id;
  member.offset_ = offset;
  return member;
}

/// A fixed-size array with no accessors at all. rosidl does not generate this,
/// but it is the most direct way to say "the elements sit inline in the struct".
CMember bare_array(const char * name, uint8_t type_id, uint32_t offset, size_t size)
{
  CMember member = scalar(name, type_id, offset);
  member.is_array_ = true;
  member.array_size_ = size;
  return member;
}

/// A dynamic sequence (array_size_ == 0).
CMember sequence(const char * name, uint8_t type_id, uint32_t offset)
{
  CMember member = scalar(name, type_id, offset);
  member.is_array_ = true;
  return member;
}

CMembers describe(CMember * members, uint32_t count, size_t size_of)
{
  CMembers description {};
  description.message_namespace_ = "synthetic::msg";
  description.message_name_ = "Probe";
  description.member_count_ = count;
  description.size_of_ = size_of;
  description.members_ = members;
  return description;
}

TypeSupportInfo info_for(const CMembers & members)
{
  TypeSupportInfo info;
  info.is_c = true;
  info.members = &members;
  info.type_name = "synthetic/msg/Probe";
  return info;
}

/// CDR starts with a 4-byte encapsulation header.
std::vector<uint8_t> cdr_payload(std::initializer_list<uint8_t> body)
{
  std::vector<uint8_t> payload {0x00, 0x01, 0x00, 0x00};
  payload.insert(payload.end(), body);
  return payload;
}

constexpr uint8_t kUnknownTypeId = 200;

}  // namespace

// ---------------------------------------------------------------------------
// Unknown type IDs
// ---------------------------------------------------------------------------

TEST(SyntheticMembers, an_unknown_type_id_is_reported_not_skipped)
{
  int32_t message = 0;
  CMember member = scalar("mystery", kUnknownTypeId, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  std::vector<uint8_t> out;
  std::string error;
  EXPECT_FALSE(serialize_message(info, &message, out, error));
  EXPECT_NE(error.find("unknown introspection type id 200"), std::string::npos) << error;

  error.clear();
  EXPECT_FALSE(serialize_message_someip(info, &message, out, error));
  EXPECT_NE(error.find("does not support"), std::string::npos) << error;

  const auto payload = cdr_payload({0, 0, 0, 0});
  error.clear();
  EXPECT_FALSE(deserialize_message(info, payload.data(), payload.size(), &message, error));
  EXPECT_NE(error.find("unknown introspection type id 200"), std::string::npos) << error;
}

// ---------------------------------------------------------------------------
// Malformed sequence descriptors
// ---------------------------------------------------------------------------

TEST(SyntheticMembers, a_sequence_without_a_size_function_is_rejected)
{
  std::vector<int32_t> message;
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  std::vector<uint8_t> out;
  std::string error;
  EXPECT_FALSE(serialize_message(info, &message, out, error));
  EXPECT_NE(error.find("has no size_function"), std::string::npos) << error;

  error.clear();
  EXPECT_FALSE(serialize_message_someip(info, &message, out, error));
  EXPECT_NE(error.find("has no size_function"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_sequence_without_element_accessors_is_rejected_when_writing)
{
  std::vector<int32_t> message {1, 2};
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  member.size_function = [](const void * field) {
      return static_cast<const std::vector<int32_t> *>(field)->size();
    };
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  std::vector<uint8_t> out;
  std::string error;
  EXPECT_FALSE(serialize_message(info, &message, out, error));
  EXPECT_NE(error.find("neither get_const_function nor fetch_function"), std::string::npos) << error;

  error.clear();
  EXPECT_FALSE(serialize_message_someip(info, &message, out, error));
  EXPECT_NE(error.find("neither get_const_function nor fetch_function"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_sequence_without_a_resize_function_is_rejected_when_reading)
{
  std::vector<int32_t> message;
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  // A one-element sequence: CDR counts elements, SOME/IP counts bytes.
  const auto cdr = cdr_payload({1, 0, 0, 0, 7, 0, 0, 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("has no resize_function"), std::string::npos) << error;

  const std::vector<uint8_t> someip {0, 0, 0, 4, 0, 0, 0, 7};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("has no resize_function"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_sequence_whose_resize_fails_is_reported)
{
  std::vector<int32_t> message;
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  member.resize_function = [](void *, size_t) {return false;};
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  const auto cdr = cdr_payload({1, 0, 0, 0, 7, 0, 0, 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("failed to resize the sequence"), std::string::npos) << error;

  const std::vector<uint8_t> someip {0, 0, 0, 4, 0, 0, 0, 7};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("failed to resize the sequence"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_sequence_without_element_accessors_is_rejected_when_reading)
{
  std::vector<int32_t> message;
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  member.resize_function = [](void * field, size_t size) {
      static_cast<std::vector<int32_t> *>(field)->resize(size);
      return true;
    };
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  const auto cdr = cdr_payload({1, 0, 0, 0, 7, 0, 0, 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("neither get_function nor assign_function"), std::string::npos) << error;

  const std::vector<uint8_t> someip {0, 0, 0, 4, 0, 0, 0, 7};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("neither get_function nor assign_function"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_bounded_sequence_longer_than_its_bound_is_rejected)
{
  std::vector<int32_t> message;
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  member.array_size_ = 2;
  member.is_upper_bound_ = true;
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  const auto cdr = cdr_payload({5, 0, 0, 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("exceeds its upper bound"), std::string::npos) << error;

  // SOME/IP counts bytes, so five elements is 20.
  std::vector<uint8_t> someip {0, 0, 0, 20};
  someip.resize(24, 0);
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("exceeds its upper bound"), std::string::npos) << error;
}

// ---------------------------------------------------------------------------
// Fixed-size arrays without accessors, which rosidl never emits but are valid
// ---------------------------------------------------------------------------

TEST(SyntheticMembers, a_bare_fixed_primitive_array_round_trips)
{
  struct Probe
  {
    int32_t values[3];
  };
  Probe message {{11, 22, 33}};
  CMember member =
    bare_array("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0, 3);
  const CMembers members = describe(&member, 1, sizeof(Probe));
  const TypeSupportInfo info = info_for(members);

  for (int pass = 0; pass < 2; ++pass) {
    const bool someip = (pass == 1);
    std::vector<uint8_t> out;
    std::string error;
    ASSERT_TRUE(
      someip ?
      serialize_message_someip(info, &message, out, error) :
      serialize_message(info, &message, out, error)) << error;

    Probe decoded {{0, 0, 0}};
    ASSERT_TRUE(
      someip ?
      deserialize_message_someip(info, out.data(), out.size(), &decoded, error) :
      deserialize_message(info, out.data(), out.size(), &decoded, error)) << error;
    EXPECT_EQ(decoded.values[0], 11);
    EXPECT_EQ(decoded.values[1], 22);
    EXPECT_EQ(decoded.values[2], 33);
  }
}

/// While the stride came from primitive_size alone, a fixed-size string array
/// had a stride of 0 and every element read the first one.
TEST(SyntheticMembers, a_bare_fixed_string_array_addresses_each_element)
{
  struct Probe
  {
    rosidl_runtime_c__String values[2];
  };
  Probe message {};
  for (auto & value : message.values) {
    ASSERT_TRUE(rosidl_runtime_c__String__init(&value));
  }
  ASSERT_TRUE(rosidl_runtime_c__String__assign(&message.values[0], "alpha"));
  ASSERT_TRUE(rosidl_runtime_c__String__assign(&message.values[1], "beta"));

  CMember member =
    bare_array("values", rosidl_typesupport_introspection_c__ROS_TYPE_STRING, 0, 2);
  const CMembers members = describe(&member, 1, sizeof(Probe));
  const TypeSupportInfo info = info_for(members);

  std::vector<uint8_t> out;
  std::string error;
  ASSERT_TRUE(serialize_message(info, &message, out, error)) << error;

  Probe decoded {};
  for (auto & value : decoded.values) {
    ASSERT_TRUE(rosidl_runtime_c__String__init(&value));
  }
  ASSERT_TRUE(deserialize_message(info, out.data(), out.size(), &decoded, error)) << error;
  EXPECT_STREQ(decoded.values[0].data, "alpha");
  EXPECT_STREQ(decoded.values[1].data, "beta");

  for (auto & value : message.values) {
    rosidl_runtime_c__String__fini(&value);
  }
  for (auto & value : decoded.values) {
    rosidl_runtime_c__String__fini(&value);
  }
}

TEST(SyntheticMembers, a_bare_fixed_nested_array_addresses_each_element)
{
  struct Inner
  {
    int32_t x;
    int32_t y;
  };
  struct Probe
  {
    Inner values[2];
  };

  CMember inner_members[2] = {
    scalar("x", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, offsetof(Inner, x)),
    scalar("y", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, offsetof(Inner, y)),
  };
  const CMembers inner = describe(inner_members, 2, sizeof(Inner));
  rosidl_message_type_support_t inner_handle {};
  inner_handle.typesupport_identifier = rosidl_typesupport_introspection_c__identifier;
  inner_handle.data = &inner;

  CMember member =
    bare_array("values", rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE, 0, 2);
  member.members_ = &inner_handle;
  const CMembers members = describe(&member, 1, sizeof(Probe));
  const TypeSupportInfo info = info_for(members);

  Probe message {{{1, 2}, {3, 4}}};
  for (int pass = 0; pass < 2; ++pass) {
    const bool someip = (pass == 1);
    std::vector<uint8_t> out;
    std::string error;
    ASSERT_TRUE(
      someip ?
      serialize_message_someip(info, &message, out, error) :
      serialize_message(info, &message, out, error)) << error;

    Probe decoded {};
    ASSERT_TRUE(
      someip ?
      deserialize_message_someip(info, out.data(), out.size(), &decoded, error) :
      deserialize_message(info, out.data(), out.size(), &decoded, error)) << error;
    EXPECT_EQ(decoded.values[0].x, 1);
    EXPECT_EQ(decoded.values[0].y, 2);
    EXPECT_EQ(decoded.values[1].x, 3);
    EXPECT_EQ(decoded.values[1].y, 4);
  }
}

TEST(SyntheticMembers, a_bare_array_whose_element_size_is_unknown_is_rejected)
{
  struct Probe
  {
    int32_t placeholder;
  };
  Probe message {0};
  // A nested type with no members_, so the element size is unknowable.
  CMember member =
    bare_array("values", rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE, 0, 2);
  const CMembers members = describe(&member, 1, sizeof(Probe));
  const TypeSupportInfo info = info_for(members);

  std::vector<uint8_t> out;
  std::string error;
  EXPECT_FALSE(serialize_message(info, &message, out, error));
  EXPECT_NE(error.find("element size is unknown"), std::string::npos) << error;

  error.clear();
  EXPECT_FALSE(serialize_message_someip(info, &message, out, error));
  EXPECT_NE(error.find("element size is unknown"), std::string::npos) << error;

  const auto cdr = cdr_payload({0, 0, 0, 0});
  error.clear();
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("element size is unknown"), std::string::npos) << error;

  const std::vector<uint8_t> someip {0, 0, 0, 0};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("element size is unknown"), std::string::npos) << error;
}

// ---------------------------------------------------------------------------
// Truncated payloads
// ---------------------------------------------------------------------------

TEST(SyntheticMembers, a_truncated_primitive_is_reported)
{
  int32_t message = 0;
  CMember member = scalar("value", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  const auto cdr = cdr_payload({1, 2});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("truncated CDR payload while reading a primitive"), std::string::npos)
    << error;

  const std::vector<uint8_t> someip {1, 2};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("truncated SOME/IP payload while reading a primitive"), std::string::npos)
    << error;
}

TEST(SyntheticMembers, a_truncated_primitive_array_is_reported)
{
  struct Probe
  {
    int32_t values[3];
  };
  Probe message {{0, 0, 0}};
  CMember member =
    bare_array("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0, 3);
  const CMembers members = describe(&member, 1, sizeof(Probe));
  const TypeSupportInfo info = info_for(members);

  const auto cdr = cdr_payload({1, 0, 0, 0, 2, 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("truncated CDR payload while reading a primitive array"), std::string::npos)
    << error;
}

TEST(SyntheticMembers, a_truncated_sequence_length_is_reported)
{
  std::vector<int32_t> message;
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  const auto cdr = cdr_payload({1, 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(
    error.find("truncated CDR payload while reading a sequence length"),
    std::string::npos) << error;

  const std::vector<uint8_t> someip {0, 0};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("at the length field"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_truncated_string_is_reported)
{
  rosidl_runtime_c__String message {};
  ASSERT_TRUE(rosidl_runtime_c__String__init(&message));
  CMember member = scalar("value", rosidl_typesupport_introspection_c__ROS_TYPE_STRING, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  // Claims length 8 with no body.
  const auto cdr = cdr_payload({8, 0, 0, 0, 'a'});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("truncated CDR payload while reading a string"), std::string::npos)
    << error;

  const std::vector<uint8_t> someip {0, 0, 0, 8, 'a'};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("truncated SOME/IP payload while reading a string"), std::string::npos)
    << error;

  rosidl_runtime_c__String__fini(&message);
}

TEST(SyntheticMembers, a_truncated_wstring_is_reported)
{
  rosidl_runtime_c__U16String message {};
  ASSERT_TRUE(rosidl_runtime_c__U16String__init(&message));
  CMember member = scalar("value", rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  const auto cdr = cdr_payload({4, 0, 0, 0, 'a', 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("truncated CDR payload while reading a wstring"), std::string::npos)
    << error;

  rosidl_runtime_c__U16String__fini(&message);
}

// ---------------------------------------------------------------------------
// SOME/IP-specific length checks
// ---------------------------------------------------------------------------

TEST(SyntheticMembers, a_someip_sequence_length_that_is_not_a_multiple_of_the_element_is_rejected)
{
  std::vector<int32_t> message;
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  // 6 bytes for a sequence of int32.
  const std::vector<uint8_t> someip {0, 0, 0, 6, 1, 2, 3, 4, 5, 6};
  std::string error;
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("is not a multiple of the element size"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_someip_sequence_of_strings_that_does_not_add_up_is_rejected)
{
  std::vector<int32_t> message;  // fails before reaching it, so the type is moot
  CMember member = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_STRING, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  // Total length 6, but the first string needs 8 bytes.
  const std::vector<uint8_t> someip {0, 0, 0, 6, 0, 0, 0, 8, 'a', 'b'};
  std::string error;
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_NE(error.find("failed to determine the element count"), std::string::npos) << error;
}

// ---------------------------------------------------------------------------
// Resolving a typesupport handle
// ---------------------------------------------------------------------------

TEST(SyntheticMembers, resolving_a_handle_without_a_type_hash_yields_the_unset_hash)
{
  CMember member = scalar("value", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);
  const CMembers members = describe(&member, 1, sizeof(int32_t));

  rosidl_message_type_support_t handle {};
  handle.typesupport_identifier = rosidl_typesupport_introspection_c__identifier;
  handle.data = &members;
  // get_message_typesupport_handle() asserts on func before it looks at the
  // identifier.
  handle.func = [](const rosidl_message_type_support_t * self, const char *) {return self;};

  TypeSupportInfo info;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::resolve_type_support(&handle, info, error)) << error;
  EXPECT_TRUE(info.is_c);
  // The "synthetic::msg" separator normalizes to "/".
  EXPECT_EQ(info.type_name, "synthetic/msg/Probe");
  EXPECT_EQ(info.type_hash.version, ROSIDL_TYPE_HASH_VERSION_UNSET);
}

// ---------------------------------------------------------------------------
// Failures partway through an array propagate out
// ---------------------------------------------------------------------------

namespace
{

/// A fixed-size array whose elements always fail to write, one per accessor
/// variant. The type_id is unknown, so write_single fails on the first element.
struct BrokenArray
{
  int32_t values[2];
};

const void * get_const_element(const void * field, size_t index)
{
  return static_cast<const int32_t *>(field) + index;
}

void * get_element(void * field, size_t index)
{
  return static_cast<int32_t *>(field) + index;
}

void fetch_element(const void * field, size_t index, void * out)
{
  *static_cast<int32_t *>(out) = static_cast<const int32_t *>(field)[index];
}

}  // namespace

TEST(SyntheticMembers, a_failure_inside_an_array_propagates_through_every_accessor)
{
  BrokenArray message {{1, 2}};

  struct Variant
  {
    const char * label;
    const void * (* get_const)(const void *, size_t);
    void (* fetch)(const void *, size_t, void *);
  };
  const Variant variants[] = {
    {"get_const_function", get_const_element, nullptr},
    {"fetch_function", nullptr, fetch_element},
  };

  for (const auto & variant : variants) {
    CMember member = bare_array("values", kUnknownTypeId, 0, 2);
    member.get_const_function = variant.get_const;
    member.fetch_function = variant.fetch;
    const CMembers members = describe(&member, 1, sizeof(BrokenArray));
    const TypeSupportInfo info = info_for(members);

    std::vector<uint8_t> out;
    std::string error;
    EXPECT_FALSE(serialize_message(info, &message, out, error)) << variant.label;
    EXPECT_NE(error.find("unknown introspection type id"), std::string::npos) << variant.label;

    error.clear();
    EXPECT_FALSE(serialize_message_someip(info, &message, out, error)) << variant.label;
    EXPECT_FALSE(error.empty()) << variant.label;
  }
}

/// The accessorless path is only taken for types with a known stride, so this
/// uses one whose size is known but whose contents are broken: a nested struct
/// with a single member of unknown type.
TEST(SyntheticMembers, a_failure_inside_a_bare_array_propagates)
{
  struct Inner
  {
    int32_t broken;
  };
  struct Probe
  {
    Inner values[2];
  };

  CMember inner_member = scalar("broken", kUnknownTypeId, 0);
  const CMembers inner = describe(&inner_member, 1, sizeof(Inner));
  rosidl_message_type_support_t inner_handle {};
  inner_handle.typesupport_identifier = rosidl_typesupport_introspection_c__identifier;
  inner_handle.data = &inner;

  CMember member =
    bare_array("values", rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE, 0, 2);
  member.members_ = &inner_handle;
  const CMembers members = describe(&member, 1, sizeof(Probe));
  const TypeSupportInfo info = info_for(members);

  Probe message {};
  std::vector<uint8_t> out;
  std::string error;
  EXPECT_FALSE(serialize_message(info, &message, out, error));
  EXPECT_NE(error.find("unknown introspection type id"), std::string::npos) << error;

  error.clear();
  EXPECT_FALSE(serialize_message_someip(info, &message, out, error));
  EXPECT_FALSE(error.empty());

  const auto cdr = cdr_payload({0, 0, 0, 0});
  error.clear();
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("unknown introspection type id"), std::string::npos) << error;

  const std::vector<uint8_t> someip {0, 0, 0, 0};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_FALSE(error.empty());
}

TEST(SyntheticMembers, a_someip_fixed_array_that_runs_out_mid_way_is_reported)
{
  struct Probe
  {
    int32_t values[3];
  };
  Probe message {{0, 0, 0}};

  // With and without accessors, the payload runs out inside the second element.
  for (int pass = 0; pass < 2; ++pass) {
    CMember member =
      bare_array("values", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0, 3);
    if (pass == 1) {
      member.get_function = get_element;
    }
    const CMembers members = describe(&member, 1, sizeof(Probe));
    const TypeSupportInfo info = info_for(members);

    const std::vector<uint8_t> someip {0, 0, 0, 1, 0, 0};
    std::string error;
    EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error))
      << "pass=" << pass;
    EXPECT_NE(error.find("truncated SOME/IP payload"), std::string::npos) << error;
  }
}

// ---------------------------------------------------------------------------
// Skipping, the path that counts elements of a SOME/IP dynamic array
// ---------------------------------------------------------------------------

namespace
{

/// A sequence of nested structs. SOME/IP lengths are byte counts, so counting
/// elements means skipping over them; these make that skip fail.
TypeSupportInfo sequence_of_nested(
  CMember & outer, CMembers & inner, CMember * inner_members, uint32_t inner_count,
  rosidl_message_type_support_t & inner_handle, CMembers & outer_members, size_t probe_size)
{
  inner = describe(inner_members, inner_count, 16);
  inner_handle = rosidl_message_type_support_t {};
  inner_handle.typesupport_identifier = rosidl_typesupport_introspection_c__identifier;
  inner_handle.data = &inner;

  outer = sequence("values", rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE, 0);
  outer.members_ = &inner_handle;
  outer_members = describe(&outer, 1, probe_size);
  return info_for(outer_members);
}

}  // namespace

TEST(SyntheticMembers, skipping_a_nested_member_that_cannot_be_skipped_is_reported)
{
  struct Probe
  {
    void * placeholder;
  };
  Probe message {nullptr};

  struct Case
  {
    const char * label;
    CMember inner_member;
  };
  CMember unknown_scalar = scalar("mystery", kUnknownTypeId, 0);
  CMember truncated_fixed =
    bare_array("fixed", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0, 4);
  CMember truncated_sequence =
    sequence("nested_sequence", rosidl_typesupport_introspection_c__ROS_TYPE_INT32, 0);

  CMember * cases[] = {&unknown_scalar, &truncated_fixed, &truncated_sequence};
  const char * labels[] = {"unknown scalar", "fixed array", "dynamic sequence"};

  for (size_t i = 0; i < 3; ++i) {
    CMember outer {};
    CMembers inner {};
    CMembers outer_members {};
    rosidl_message_type_support_t inner_handle {};
    const TypeSupportInfo info = sequence_of_nested(
      outer, inner, cases[i], 1, inner_handle, outer_members, sizeof(Probe));

    // A 4-byte sequence whose contents cannot be counted.
    const std::vector<uint8_t> someip {0, 0, 0, 4, 0, 0, 0, 2};
    std::string error;
    EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error))
      << labels[i];
    EXPECT_NE(error.find("failed to determine the element count"), std::string::npos)
      << labels[i] << ": " << error;
  }
}

// ---------------------------------------------------------------------------
// Null handles
// ---------------------------------------------------------------------------

TEST(SyntheticMembers, a_null_type_support_handle_is_reported)
{
  TypeSupportInfo message_info;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::resolve_type_support(nullptr, message_info, error));
  EXPECT_FALSE(error.empty());

  rmw_someip_cpp::ServiceTypeSupportInfo service_info;
  error.clear();
  EXPECT_FALSE(rmw_someip_cpp::resolve_service_type_support(nullptr, service_info, error));
  EXPECT_FALSE(error.empty());
}

TEST(SyntheticMembers, serializing_without_members_or_a_message_is_refused)
{
  TypeSupportInfo info;
  info.is_c = true;
  info.members = nullptr;

  int32_t message = 0;
  std::vector<uint8_t> out;
  std::string error;
  EXPECT_FALSE(serialize_message(info, &message, out, error));
  EXPECT_FALSE(serialize_message_someip(info, &message, out, error));
  EXPECT_FALSE(deserialize_message(info, nullptr, 0, &message, error));
  EXPECT_FALSE(deserialize_message_someip(info, nullptr, 0, &message, error));
}

// ---------------------------------------------------------------------------
// The C++ introspection
// ---------------------------------------------------------------------------

/// The C and C++ descriptors share a layout and go through the same template.
/// Only the string representation (rosidl_runtime_c__String vs std::string) and
/// the element stride differ, so those are what the C++ side checks.
TEST(SyntheticMembers, the_cpp_introspection_walks_the_same_way)
{
  using CppMember = rosidl_typesupport_introspection_cpp::MessageMember;
  using CppMembers = rosidl_typesupport_introspection_cpp::MessageMembers;

  struct Probe
  {
    std::string values[2];
  };
  Probe message;
  message.values[0] = "alpha";
  message.values[1] = "beta";

  CppMember member {};
  member.name_ = "values";
  member.type_id_ = rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
  member.is_array_ = true;
  member.array_size_ = 2;

  CppMembers members {};
  members.message_namespace_ = "synthetic::msg";
  members.message_name_ = "CppProbe";
  members.member_count_ = 1;
  members.size_of_ = sizeof(Probe);
  members.members_ = &member;

  TypeSupportInfo info;
  info.is_c = false;
  info.members = &members;
  info.type_name = "synthetic/msg/CppProbe";

  std::vector<uint8_t> out;
  std::string error;
  ASSERT_TRUE(serialize_message(info, &message, out, error)) << error;

  Probe decoded;
  ASSERT_TRUE(deserialize_message(info, out.data(), out.size(), &decoded, error)) << error;
  EXPECT_EQ("alpha", decoded.values[0]);
  EXPECT_EQ("beta", decoded.values[1]);

  // An undeterminable stride is rejected on the C++ side too.
  CppMember broken = member;
  broken.type_id_ = rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
  broken.members_ = nullptr;
  members.members_ = &broken;
  error.clear();
  EXPECT_FALSE(serialize_message(info, &message, out, error));
  EXPECT_NE(error.find("element size is unknown"), std::string::npos) << error;
}

TEST(SyntheticMembers, a_wstring_whose_length_field_is_truncated_is_reported)
{
  rosidl_runtime_c__U16String message {};
  ASSERT_TRUE(rosidl_runtime_c__U16String__init(&message));
  CMember member = scalar("value", rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING, 0);
  const CMembers members = describe(&member, 1, sizeof(message));
  const TypeSupportInfo info = info_for(members);

  // Cut off inside the uint32 length field.
  const auto cdr = cdr_payload({4, 0});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("truncated CDR payload while reading a wstring"), std::string::npos)
    << error;

  rosidl_runtime_c__U16String__fini(&message);
}

// ---------------------------------------------------------------------------
// Bulk copying arrays of 1-byte elements
// ---------------------------------------------------------------------------

namespace
{

using ByteVector = std::vector<uint8_t>;

size_t byte_size(const void * field)
{
  return static_cast<const ByteVector *>(field)->size();
}
const void * byte_get_const(const void * field, size_t index)
{
  return &(*static_cast<const ByteVector *>(field))[index];
}
void * byte_get(void * field, size_t index)
{
  return &(*static_cast<ByteVector *>(field))[index];
}
bool byte_resize(void * field, size_t size)
{
  static_cast<ByteVector *>(field)->resize(size);
  return true;
}

/// A uint8[] sequence shaped like what rosidl generates, accessors included.
CMember byte_sequence(const char * name)
{
  CMember member = sequence(name, rosidl_typesupport_introspection_c__ROS_TYPE_UINT8, 0);
  member.size_function = byte_size;
  member.get_const_function = byte_get_const;
  member.get_function = byte_get;
  member.resize_function = byte_resize;
  return member;
}

}  // namespace

/// Writing a byte array one element at a time makes latency grow with the
/// payload (docs/design.md §24). This exercises the bulk path.
TEST(SyntheticMembers, a_byte_sequence_is_copied_in_one_go)
{
  ByteVector message;
  for (size_t i = 0; i < 300; ++i) {
    message.push_back(static_cast<uint8_t>(i & 0xFF));
  }

  CMember member = byte_sequence("data");
  const CMembers members = describe(&member, 1, sizeof(ByteVector));
  const TypeSupportInfo info = info_for(members);

  for (int pass = 0; pass < 2; ++pass) {
    const bool someip = (pass == 1);
    std::vector<uint8_t> out;
    std::string error;
    ASSERT_TRUE(
      someip ?
      serialize_message_someip(info, &message, out, error) :
      serialize_message(info, &message, out, error)) << error;

    ByteVector decoded;
    ASSERT_TRUE(
      someip ?
      deserialize_message_someip(info, out.data(), out.size(), &decoded, error) :
      deserialize_message(info, out.data(), out.size(), &decoded, error)) << error;
    EXPECT_EQ(message, decoded) << "someip=" << someip;
  }
}

TEST(SyntheticMembers, an_empty_byte_sequence_round_trips)
{
  ByteVector message;
  CMember member = byte_sequence("data");
  const CMembers members = describe(&member, 1, sizeof(ByteVector));
  const TypeSupportInfo info = info_for(members);

  for (int pass = 0; pass < 2; ++pass) {
    const bool someip = (pass == 1);
    std::vector<uint8_t> out;
    std::string error;
    ASSERT_TRUE(
      someip ?
      serialize_message_someip(info, &message, out, error) :
      serialize_message(info, &message, out, error)) << error;

    ByteVector decoded {1, 2, 3};
    ASSERT_TRUE(
      someip ?
      deserialize_message_someip(info, out.data(), out.size(), &decoded, error) :
      deserialize_message(info, out.data(), out.size(), &decoded, error)) << error;
    EXPECT_TRUE(decoded.empty()) << "someip=" << someip;
  }
}

TEST(SyntheticMembers, a_byte_sequence_cut_short_is_reported)
{
  ByteVector message;
  CMember member = byte_sequence("data");
  const CMembers members = describe(&member, 1, sizeof(ByteVector));
  const TypeSupportInfo info = info_for(members);

  // CDR counts elements, SOME/IP bytes; neither has enough body.
  const auto cdr = cdr_payload({8, 0, 0, 0, 1, 2});
  std::string error;
  EXPECT_FALSE(deserialize_message(info, cdr.data(), cdr.size(), &message, error));
  EXPECT_NE(error.find("truncated CDR payload"), std::string::npos) << error;

  // SOME/IP rejects as soon as the length exceeds what remains.
  const std::vector<uint8_t> someip {0, 0, 0, 8, 1, 2};
  error.clear();
  EXPECT_FALSE(deserialize_message_someip(info, someip.data(), someip.size(), &message, error));
  EXPECT_FALSE(error.empty());
}
