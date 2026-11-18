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
// CDR primitives.
//
// Two properties were established by measurement and break byte compatibility
// with DDS implementations if changed: alignment is measured from after the
// encapsulation header rather than from the start of the buffer, and a wstring
// character occupies 4 bytes (docs/design.md §16).

#include <cstdint>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "detail/cdr.hpp"

TEST(Cdr, writes_the_little_endian_encapsulation_header)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  ASSERT_EQ(4u, buffer.size());
  EXPECT_EQ(0x00, buffer[0]);
  EXPECT_EQ(0x01, buffer[1]);
  EXPECT_EQ(0x00, buffer[2]);
  EXPECT_EQ(0x00, buffer[3]);
}

/// Alignment starts after the header; counting from the buffer start is off by 4.
TEST(Cdr, aligns_relative_to_the_end_of_the_header)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint8_t>(0x11);      // offset 0 -> 1
  writer.write<uint32_t>(0x22334455);  // offset 1 aligns to 4, so 3 bytes of padding
  ASSERT_EQ(4u + 8u, buffer.size());
  EXPECT_EQ(0x11, buffer[4]);
  EXPECT_EQ(0u, buffer[5]);
  EXPECT_EQ(0u, buffer[6]);
  EXPECT_EQ(0u, buffer[7]);
  EXPECT_EQ(0x55, buffer[8]);  // little endian

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  uint8_t first = 0;
  uint32_t second = 0;
  ASSERT_TRUE(reader.read<uint8_t>(first));
  ASSERT_TRUE(reader.read<uint32_t>(second));
  EXPECT_EQ(0x11, first);
  EXPECT_EQ(0x22334455u, second);
}

TEST(Cdr, round_trips_every_primitive_width)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint8_t>(0x01);
  writer.write<uint16_t>(0x0203);
  writer.write<uint32_t>(0x04050607);
  writer.write<uint64_t>(0x08090A0B0C0D0E0FULL);
  writer.write<float>(1.5f);
  writer.write<double>(-2.25);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  uint8_t v8 = 0;
  uint16_t v16 = 0;
  uint32_t v32 = 0;
  uint64_t v64 = 0;
  float f = 0;
  double d = 0;
  ASSERT_TRUE(reader.read<uint8_t>(v8));
  ASSERT_TRUE(reader.read<uint16_t>(v16));
  ASSERT_TRUE(reader.read<uint32_t>(v32));
  ASSERT_TRUE(reader.read<uint64_t>(v64));
  ASSERT_TRUE(reader.read<float>(f));
  ASSERT_TRUE(reader.read<double>(d));
  EXPECT_EQ(0x01, v8);
  EXPECT_EQ(0x0203, v16);
  EXPECT_EQ(0x04050607u, v32);
  EXPECT_EQ(0x08090A0B0C0D0E0FULL, v64);
  EXPECT_FLOAT_EQ(1.5f, f);
  EXPECT_DOUBLE_EQ(-2.25, d);
}

/// A string length includes the NUL; even an empty string writes length 1 and a NUL.
TEST(Cdr, strings_carry_the_nul_in_their_length)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write_string("ab", 2);

  EXPECT_EQ(4u + 4u + 3u, buffer.size());
  EXPECT_EQ(3u, buffer[4]);  // length is 2 plus the NUL

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  const char * data = nullptr;
  size_t length = 0;
  ASSERT_TRUE(reader.read_string(data, length));
  EXPECT_EQ(2u, length);
  EXPECT_EQ(0, std::string(data, length).compare("ab"));
}

TEST(Cdr, empty_string_round_trips)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write_string("", 0);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  const char * data = nullptr;
  size_t length = 0;
  ASSERT_TRUE(reader.read_string(data, length));
  EXPECT_EQ(0u, length);
}

/// Each character is 4 bytes. ROS represents wstrings as UTF-16, but
/// rosidl_typesupport_fastrtps_cpp converts to std::wstring before encoding
/// and wchar_t is 4 bytes on Linux. Verified against its output.
TEST(Cdr, wstring_uses_four_bytes_per_character)
{
  const uint16_t text[] = {0x0061, 0x0062};  // "ab"
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write_wstring(text, 2);

  ASSERT_EQ(4u + 4u + 8u, buffer.size());
  EXPECT_EQ(2u, buffer[4]);   // characters, not bytes
  EXPECT_EQ(0x61, buffer[8]);
  EXPECT_EQ(0u, buffer[9]);
  EXPECT_EQ(0u, buffer[10]);
  EXPECT_EQ(0u, buffer[11]);
  EXPECT_EQ(0x62, buffer[12]);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  std::vector<uint16_t> restored;
  ASSERT_TRUE(reader.read_wstring(restored));
  ASSERT_EQ(2u, restored.size());
  EXPECT_EQ(0x0061, restored[0]);
  EXPECT_EQ(0x0062, restored[1]);
}

TEST(Cdr, empty_wstring_round_trips)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write_wstring(nullptr, 0);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  std::vector<uint16_t> restored;
  ASSERT_TRUE(reader.read_wstring(restored));
  EXPECT_TRUE(restored.empty());
}

TEST(Cdr, write_bytes_and_read_bytes_round_trip)
{
  const uint8_t payload[] = {1, 2, 3, 4, 5};
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write_bytes(payload, sizeof(payload));

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  uint8_t restored[sizeof(payload)] = {};
  ASSERT_TRUE(reader.read_bytes(restored, sizeof(restored)));
  EXPECT_EQ(0, std::memcmp(payload, restored, sizeof(payload)));
}

// ---------------------------------------------------------------------------
// Malformed input must not crash
// ---------------------------------------------------------------------------

/// Input shorter than the header.
///
/// With position_ before origin_, align()'s subtraction wraps as size_t, slips
/// past the bounds check and reads header bytes as payload.
TEST(Cdr, reader_rejects_a_buffer_shorter_than_the_header)
{
  for (size_t size = 0; size < rmw_someip_cpp::cdr::kEncapsulationSize; ++size) {
    const uint8_t data[] = {0x00, 0x01, 0x00, 0x00};
    rmw_someip_cpp::cdr::Reader reader(data, size);
    EXPECT_FALSE(reader.ok());
    uint8_t value = 0xFF;
    EXPECT_FALSE(reader.read<uint8_t>(value));
    EXPECT_EQ(0xFF, value) << "read payload at size=" << size;
  }
}

/// Reader only reports the endianness; rejecting it is deserialize_message()'s
/// job. This checks the report is right.
TEST(Cdr, reader_reports_big_endian_encapsulation)
{
  const uint8_t big_endian[] = {0x00, 0x00, 0x00, 0x00, 0x01};
  EXPECT_TRUE(rmw_someip_cpp::cdr::Reader(big_endian, sizeof(big_endian)).big_endian());

  const uint8_t little_endian[] = {0x00, 0x01, 0x00, 0x00, 0x01};
  EXPECT_FALSE(rmw_someip_cpp::cdr::Reader(little_endian, sizeof(little_endian)).big_endian());
}

/// Nothing can be read after fail().
TEST(Cdr, reader_stays_failed_once_marked)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint32_t>(1);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  EXPECT_TRUE(reader.ok());
  reader.fail();
  EXPECT_FALSE(reader.ok());
  uint32_t value = 0;
  EXPECT_FALSE(reader.read<uint32_t>(value));
  uint8_t bytes[1] = {};
  EXPECT_FALSE(reader.read_bytes(bytes, 1));
}

TEST(Cdr, reader_stops_at_the_end_of_the_buffer)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint32_t>(42);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  uint32_t value = 0;
  ASSERT_TRUE(reader.read<uint32_t>(value));
  EXPECT_EQ(42u, value);
  // There is no second one.
  EXPECT_FALSE(reader.read<uint32_t>(value));
}

TEST(Cdr, reader_rejects_a_truncated_string)
{
  // Declares length 100 with no body.
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint32_t>(100);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  const char * data = nullptr;
  size_t length = 0;
  EXPECT_FALSE(reader.read_string(data, length));
}

TEST(Cdr, reader_rejects_a_zero_length_string)
{
  // The length includes the NUL, so 0 is impossible.
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint32_t>(0);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  const char * data = nullptr;
  size_t length = 0;
  EXPECT_FALSE(reader.read_string(data, length));
}

TEST(Cdr, reader_rejects_a_truncated_wstring)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint32_t>(100);  // claims 100 characters, so 400 bytes

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  std::vector<uint16_t> restored;
  EXPECT_FALSE(reader.read_wstring(restored));
}

TEST(Cdr, reader_rejects_truncated_bytes)
{
  std::vector<uint8_t> buffer;
  rmw_someip_cpp::cdr::Writer writer(buffer);
  writer.write<uint8_t>(1);

  rmw_someip_cpp::cdr::Reader reader(buffer.data(), buffer.size());
  uint8_t out[16] = {};
  EXPECT_FALSE(reader.read_bytes(out, sizeof(out)));
}
