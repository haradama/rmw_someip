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
// The CDR (native mode) and AUTOSAR SOME/IP Transformer (extern mode)
// serializers.
//
// Each type is checked to survive a round trip. Both the C and C++
// introspections are exercised: the struct layouts match, but string
// representation and the resize/fetch/assign handling differ, so passing under
// one says little about the other (docs/design.md §15(b)).

#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/bounded_sequences.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/nested.hpp"
#include "test_msgs/msg/strings.h"
#include "test_msgs/msg/strings.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"
#include "test_msgs/msg/w_strings.hpp"

#include "detail/someip_serializer.hpp"
#include "detail/type_support.hpp"

namespace
{

template<typename T>
rmw_someip_cpp::TypeSupportInfo resolve_cpp()
{
  rmw_someip_cpp::TypeSupportInfo info;
  std::string error;
  EXPECT_TRUE(
    rmw_someip_cpp::resolve_type_support(
      rosidl_typesupport_cpp::get_message_type_support_handle<T>(), info, error)) << error;
  return info;
}

/// Round trip through CDR and compare.
template<typename T>
void expect_cdr_round_trip(const T & original)
{
  const auto ts = resolve_cpp<T>();
  std::vector<uint8_t> buffer;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::serialize_message(ts, &original, buffer, error)) << error;

  T restored;
  ASSERT_TRUE(
    rmw_someip_cpp::deserialize_message(
      ts, buffer.data(), buffer.size(), &restored, error)) << error;
  EXPECT_EQ(original, restored);
}

/// Round trip through SOME/IP (AUTOSAR) and compare.
template<typename T>
void expect_someip_round_trip(const T & original)
{
  const auto ts = resolve_cpp<T>();
  std::vector<uint8_t> buffer;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::serialize_message_someip(ts, &original, buffer, error)) << error;

  T restored;
  ASSERT_TRUE(
    rmw_someip_cpp::deserialize_message_someip(
      ts, buffer.data(), buffer.size(), &restored, error)) << error;
  EXPECT_EQ(original, restored);
}

template<typename T>
void expect_both_round_trip(const T & original)
{
  expect_cdr_round_trip(original);
  expect_someip_round_trip(original);
}

test_msgs::msg::BasicTypes make_basic_types()
{
  test_msgs::msg::BasicTypes message;
  message.bool_value = true;
  message.byte_value = 0xAB;
  message.char_value = 'z';
  message.float32_value = -1.25f;
  message.float64_value = 3.5;
  message.int8_value = -8;
  message.uint8_value = 200;
  message.int16_value = -1234;
  message.uint16_value = 60000;
  message.int32_value = -123456;
  message.uint32_value = 4000000000u;
  message.int64_value = -1234567890123LL;
  message.uint64_value = 12345678901234567890ULL;
  return message;
}

}  // namespace

TEST(Serialization, empty_message)
{
  // test_msgs/Empty has no fields, but a C/C++ struct cannot be zero-sized, so
  // it carries one uint8 named structure_needs_at_least_one_member. That makes
  // it one byte over SOME/IP as well.
  test_msgs::msg::Empty message;
  expect_both_round_trip(message);

  const auto ts = resolve_cpp<test_msgs::msg::Empty>();
  std::vector<uint8_t> buffer;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::serialize_message_someip(ts, &message, buffer, error)) << error;
  EXPECT_EQ(1u, buffer.size());
}

TEST(Serialization, basic_types)
{
  expect_both_round_trip(make_basic_types());
}

TEST(Serialization, strings)
{
  test_msgs::msg::Strings message;
  message.string_value = "";
  message.bounded_string_value = "bounded";
  expect_both_round_trip(message);

  message.string_value = std::string(300, 'x');
  expect_both_round_trip(message);
}

TEST(Serialization, wstrings_round_trip_in_cdr)
{
  test_msgs::msg::WStrings message;
  message.wstring_value = u"";
  expect_cdr_round_trip(message);

  message.wstring_value = u"ab";
  // A non-ASCII value, so the test would notice a UTF-16 encoding mistake.
  message.array_of_wstrings[0] = u"あい";
  expect_cdr_round_trip(message);
}

/// The SOME/IP serializer cannot handle wstrings.
///
/// The AUTOSAR SOME/IP Transformer has no standard mapping for UTF-16 strings,
/// and there is no format to agree on with a non-ROS ECU, so it is
/// unimplemented. Failing is safer than silently emitting a broken encoding.
/// A type containing a wstring is rejected here in extern mode.
TEST(Serialization, someip_rejects_wstrings)
{
  const auto ts = resolve_cpp<test_msgs::msg::WStrings>();
  test_msgs::msg::WStrings message;
  message.wstring_value = u"ab";

  std::vector<uint8_t> buffer;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::serialize_message_someip(ts, &message, buffer, error));
  EXPECT_NE(std::string::npos, error.find("does not support"));

  error.clear();
  const uint8_t junk[16] = {};
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message_someip(ts, junk, sizeof(junk), &message, error));
  EXPECT_NE(std::string::npos, error.find("does not support"));
}

TEST(Serialization, fixed_arrays)
{
  test_msgs::msg::Arrays message;
  message.bool_values = {{true, false, true}};
  message.int32_values = {{1, -2, 3}};
  message.float64_values = {{1.5, -2.5, 3.5}};
  message.string_values = {{"a", "", "ccc"}};
  message.basic_types_values[0] = make_basic_types();
  expect_both_round_trip(message);
}

TEST(Serialization, unbounded_sequences)
{
  test_msgs::msg::UnboundedSequences message;
  expect_both_round_trip(message);  // everything empty

  message.bool_values = {true, false};
  message.int32_values = {1, 2, 3};
  message.string_values = {"a", "bb", ""};
  message.basic_types_values.push_back(make_basic_types());
  message.basic_types_values.push_back(make_basic_types());
  expect_both_round_trip(message);
}

TEST(Serialization, bounded_sequences)
{
  test_msgs::msg::BoundedSequences message;
  message.int32_values = {7, 8};
  message.string_values = {"x"};
  message.basic_types_values.push_back(make_basic_types());
  expect_both_round_trip(message);
}

TEST(Serialization, nested_and_multi_nested)
{
  test_msgs::msg::Nested nested;
  nested.basic_types_value = make_basic_types();
  expect_both_round_trip(nested);

  test_msgs::msg::MultiNested multi;
  multi.array_of_arrays[0].int32_values = {{1, 2, 3}};
  multi.unbounded_sequence_of_arrays.push_back(multi.array_of_arrays[0]);
  expect_both_round_trip(multi);
}

/// The C introspection gives the same result. Same struct layout as C++, but a
/// different string representation (rosidl_runtime_c__String) and a different
/// resize_function return type.
TEST(Serialization, c_introspection_round_trip)
{
  rmw_someip_cpp::TypeSupportInfo ts;
  std::string error;
  ASSERT_TRUE(
    rmw_someip_cpp::resolve_type_support(
      ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, Strings), ts, error)) << error;
  EXPECT_TRUE(ts.is_c);

  test_msgs__msg__Strings original;
  ASSERT_TRUE(test_msgs__msg__Strings__init(&original));
  ASSERT_TRUE(rosidl_runtime_c__String__assign(&original.string_value, "hello from C"));

  std::vector<uint8_t> buffer;
  ASSERT_TRUE(rmw_someip_cpp::serialize_message(ts, &original, buffer, error)) << error;

  test_msgs__msg__Strings restored;
  ASSERT_TRUE(test_msgs__msg__Strings__init(&restored));
  ASSERT_TRUE(
    rmw_someip_cpp::deserialize_message(
      ts, buffer.data(), buffer.size(), &restored, error)) << error;
  EXPECT_STREQ("hello from C", restored.string_value.data);

  test_msgs__msg__Strings__fini(&original);
  test_msgs__msg__Strings__fini(&restored);
}

// ---------------------------------------------------------------------------
// Malformed input
// ---------------------------------------------------------------------------

TEST(Serialization, rejects_a_null_type_support)
{
  rmw_someip_cpp::TypeSupportInfo empty;
  std::vector<uint8_t> buffer;
  std::string error;
  test_msgs::msg::BasicTypes message;

  EXPECT_FALSE(rmw_someip_cpp::serialize_message(empty, &message, buffer, error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(rmw_someip_cpp::serialize_message_someip(empty, &message, buffer, error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(rmw_someip_cpp::deserialize_message(empty, nullptr, 0, &message, error));
  EXPECT_FALSE(error.empty());

  error.clear();
  EXPECT_FALSE(rmw_someip_cpp::deserialize_message_someip(empty, nullptr, 0, &message, error));
  EXPECT_FALSE(error.empty());
}

TEST(Serialization, rejects_a_null_message)
{
  const auto ts = resolve_cpp<test_msgs::msg::BasicTypes>();
  std::vector<uint8_t> buffer;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::serialize_message(ts, nullptr, buffer, error));
  error.clear();
  EXPECT_FALSE(rmw_someip_cpp::serialize_message_someip(ts, nullptr, buffer, error));
}

/// A truncated payload fails rather than crashing. Malformed or hostile input
/// must never bring the RMW down.
TEST(Serialization, truncated_payload_fails_cleanly)
{
  const auto ts = resolve_cpp<test_msgs::msg::Strings>();
  test_msgs::msg::Strings original;
  original.string_value = "some text that will be cut";

  std::vector<uint8_t> buffer;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::serialize_message(ts, &original, buffer, error)) << error;

  for (size_t length = 0; length < buffer.size(); ++length) {
    test_msgs::msg::Strings restored;
    std::string ignored;
    // Not crashing is the requirement; either outcome is legitimate, since
    // some short lengths are still formally readable.
    (void)rmw_someip_cpp::deserialize_message(ts, buffer.data(), length, &restored, ignored);
  }

  std::vector<uint8_t> someip_buffer;
  ASSERT_TRUE(
    rmw_someip_cpp::serialize_message_someip(ts, &original, someip_buffer, error)) << error;
  for (size_t length = 0; length < someip_buffer.size(); ++length) {
    test_msgs::msg::Strings restored;
    std::string ignored;
    (void)rmw_someip_cpp::deserialize_message_someip(
      ts, someip_buffer.data(), length, &restored, ignored);
  }
}

/// A corrupt CDR encapsulation header.
TEST(Serialization, rejects_a_bad_encapsulation_header)
{
  const auto ts = resolve_cpp<test_msgs::msg::BasicTypes>();
  test_msgs::msg::BasicTypes restored;
  std::string error;

  // Big endian (0x0000) is unsupported.
  const uint8_t big_endian[] = {0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0};
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message(
      ts, big_endian, sizeof(big_endian), &restored, error));

  // Shorter than the header.
  const uint8_t too_short[] = {0x00};
  error.clear();
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message(ts, too_short, sizeof(too_short), &restored, error));
}

/// A handle whose type cannot be resolved.
TEST(Serialization, rejects_an_unresolvable_type_support)
{
  rmw_someip_cpp::TypeSupportInfo info;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::resolve_type_support(nullptr, info, error));
  EXPECT_FALSE(error.empty());
}

// ---------------------------------------------------------------------------
// Hostile payloads
//
// This input arrives from the network, so a corrupt value must not crash the
// process or force a huge allocation.
// ---------------------------------------------------------------------------

namespace
{

/// Build CDR bytes by hand, encapsulation header included.
class RawCdr
{
public:
  RawCdr() {bytes_ = {0x00, 0x01, 0x00, 0x00};}

  RawCdr & u32(uint32_t value)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      bytes_.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
    }
    return *this;
  }
  RawCdr & u8(uint8_t value)
  {
    bytes_.push_back(value);
    return *this;
  }

  const std::vector<uint8_t> & bytes() const {return bytes_;}

private:
  void align(size_t alignment)
  {
    const size_t offset = bytes_.size() - 4;
    bytes_.insert(bytes_.end(), (alignment - (offset % alignment)) % alignment, 0);
  }
  std::vector<uint8_t> bytes_;
};

}  // namespace

/// A bounded sequence declaring more than its bound is rejected. Letting it
/// through would resize to the declared count.
TEST(Serialization, rejects_a_bounded_sequence_over_its_bound)
{
  const auto ts = resolve_cpp<test_msgs::msg::BoundedSequences>();
  test_msgs::msg::BoundedSequences message;

  // Declare a huge count for the first member, bool_values.
  RawCdr raw;
  raw.u32(0xFFFFFFFFu);

  std::string error;
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message(
      ts, raw.bytes().data(), raw.bytes().size(), &message, error));
  EXPECT_FALSE(error.empty());
}

/// Even an unbounded sequence is rejected without a body for the declared count.
TEST(Serialization, rejects_an_unbounded_sequence_without_its_elements)
{
  const auto ts = resolve_cpp<test_msgs::msg::UnboundedSequences>();
  test_msgs::msg::UnboundedSequences message;

  RawCdr raw;
  raw.u32(1000000);  // declares a million elements with no body

  std::string error;
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message(
      ts, raw.bytes().data(), raw.bytes().size(), &message, error));
  EXPECT_FALSE(error.empty());
}

/// Same on the SOME/IP side. A dynamic array's length is a byte count, and
/// one larger than what remains is rejected.
TEST(Serialization, someip_rejects_an_oversized_sequence_length)
{
  const auto ts = resolve_cpp<test_msgs::msg::UnboundedSequences>();
  test_msgs::msg::UnboundedSequences message;

  // Declares 0xFFFFFFFF bytes, big endian.
  const uint8_t payload[] = {0xFF, 0xFF, 0xFF, 0xFF};
  std::string error;
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message_someip(
      ts, payload, sizeof(payload), &message, error));
  EXPECT_FALSE(error.empty());
}

/// A string length beyond the remaining bytes.
TEST(Serialization, rejects_an_oversized_string_length)
{
  const auto ts = resolve_cpp<test_msgs::msg::Strings>();
  test_msgs::msg::Strings message;

  RawCdr raw;
  raw.u32(0x7FFFFFFF);  // declares a 2 GB string

  std::string error;
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message(
      ts, raw.bytes().data(), raw.bytes().size(), &message, error));
  EXPECT_FALSE(error.empty());
}

/// An all-zero payload must not crash. A string length includes the NUL, so 0
/// is impossible and this fails.
TEST(Serialization, rejects_an_all_zero_payload)
{
  const auto ts = resolve_cpp<test_msgs::msg::Strings>();
  test_msgs::msg::Strings message;
  const std::vector<uint8_t> zeros(64, 0);
  std::string error;
  // The leading 0x00000000 selects big endian, which is rejected.
  EXPECT_FALSE(
    rmw_someip_cpp::deserialize_message(ts, zeros.data(), zeros.size(), &message, error));
}

/// Random bytes must not crash it.
TEST(Serialization, survives_arbitrary_payloads)
{
  const auto ts = resolve_cpp<test_msgs::msg::MultiNested>();
  const auto someip_ts = resolve_cpp<test_msgs::msg::UnboundedSequences>();

  uint32_t seed = 12345;
  for (int iteration = 0; iteration < 200; ++iteration) {
    std::vector<uint8_t> payload(4 + (iteration % 40));
    payload[0] = 0x00;
    payload[1] = 0x01;  // little endian, so it gets as far as the body
    payload[2] = 0x00;
    payload[3] = 0x00;
    for (size_t i = 4; i < payload.size(); ++i) {
      seed = seed * 1103515245u + 12345u;
      payload[i] = static_cast<uint8_t>(seed >> 16);
    }

    test_msgs::msg::MultiNested message;
    std::string error;
    (void)rmw_someip_cpp::deserialize_message(
      ts, payload.data(), payload.size(), &message, error);

    test_msgs::msg::UnboundedSequences other;
    (void)rmw_someip_cpp::deserialize_message_someip(
      someip_ts, payload.data(), payload.size(), &other, error);
  }
  SUCCEED();  // not crashing is the requirement
}
