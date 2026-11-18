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
// AUTOSAR SOME/IP Transformer serializer, used in extern mode.
//
// The differences from CDR are the whole point of extern mode, so this is a
// separate implementation rather than a shared one:
//
//   | | CDR | SOME/IP |
//   |---|---|---|
//   | byte order | little endian | big endian |
//   | alignment | to the type width, padded | packed, no padding |
//   | encapsulation header | 4 bytes | none |
//   | dynamic array length | element count (uint32) | byte count (uint32) |
//   | string length | bytes including NUL | bytes including NUL |
//
// The byte-count length is what complicates reading: the element count is
// unknown, so elements of variable size (strings, nested structs) have to be
// skipped over and counted first.

#include "detail/someip_serializer.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "detail/introspection_traits.hpp"

namespace rmw_someip_cpp
{

namespace
{

// ---------------------------------------------------------------------------
// Big-endian reading and writing
// ---------------------------------------------------------------------------

class Writer
{
public:
  explicit Writer(std::vector<uint8_t> & buffer)
  : buffer_(buffer) {}

  /// Write a primitive big endian. No alignment.
  void write_primitive(const void * data, size_t width)
  {
    const auto * bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < width; ++i) {
      buffer_.push_back(bytes[width - 1 - i]);
    }
  }

  void write_raw(const void * data, size_t size)
  {
    const auto * bytes = static_cast<const uint8_t *>(data);
    buffer_.insert(buffer_.end(), bytes, bytes + size);
  }

  /// Reserve a length field. \return the position to fill in later
  size_t reserve_length()
  {
    const size_t position = buffer_.size();
    buffer_.insert(buffer_.end(), 4, 0);
    return position;
  }

  /// Write the byte count from a reserved position to the current one.
  void fill_length(size_t position)
  {
    const auto length = static_cast<uint32_t>(buffer_.size() - position - 4);
    buffer_[position] = static_cast<uint8_t>((length >> 24) & 0xFF);
    buffer_[position + 1] = static_cast<uint8_t>((length >> 16) & 0xFF);
    buffer_[position + 2] = static_cast<uint8_t>((length >> 8) & 0xFF);
    buffer_[position + 3] = static_cast<uint8_t>(length & 0xFF);
  }

  /// A string: length in bytes including NUL, then the body, then NUL.
  void write_string(const char * data, size_t length)
  {
    const auto encoded = static_cast<uint32_t>(length + 1);
    write_primitive(&encoded, sizeof(encoded));
    if (length > 0) {
      write_raw(data, length);
    }
    buffer_.push_back(0);
  }

private:
  std::vector<uint8_t> & buffer_;
};

class Reader
{
public:
  Reader(const uint8_t * data, size_t size)
  : data_(data), size_(size) {}

  size_t position() const {return position_;}
  void seek(size_t position) {position_ = position;}
  size_t remaining() const {return size_ - position_;}

  bool read_primitive(void * out, size_t width)
  {
    if (position_ + width > size_) {
      return false;
    }
    auto * bytes = static_cast<uint8_t *>(out);
    for (size_t i = 0; i < width; ++i) {
      bytes[i] = data_[position_ + width - 1 - i];
    }
    position_ += width;
    return true;
  }

  bool read_uint32(uint32_t & value)
  {
    return read_primitive(&value, sizeof(value));
  }

  /// Point at the string body and return its length, excluding the NUL.
  bool read_string(const char *& out, size_t & length)
  {
    uint32_t encoded = 0;
    if (!read_uint32(encoded) || encoded == 0 || position_ + encoded > size_) {
      return false;
    }
    out = reinterpret_cast<const char *>(data_ + position_);
    length = encoded - 1;   // drop the terminating NUL
    position_ += encoded;
    return true;
  }

  /// Copy bytes verbatim, for arrays of 1-byte elements where order is moot.
  bool read_raw(void * out, size_t size)
  {
    if (position_ + size > size_) {
      return false;
    }
    std::memcpy(out, data_ + position_, size);
    position_ += size;
    return true;
  }

  bool skip(size_t bytes)
  {
    if (position_ + bytes > size_) {
      return false;
    }
    position_ += bytes;
    return true;
  }

private:
  const uint8_t * data_;
  size_t size_;
  size_t position_ {0};
};

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

template<typename Traits>
bool write_members(
  const typename Traits::Members * members,
  const void * ros_message,
  Writer & writer,
  std::string & error);

template<typename Traits>
bool write_single(
  const typename Traits::Member & member,
  const void * field,
  Writer & writer,
  std::string & error)
{
  const size_t width = primitive_size(member.type_id_);
  if (width != 0) {
    writer.write_primitive(field, width);
    return true;
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_STRING) {
    writer.write_string(Traits::string_data(field), Traits::string_size(field));
    return true;
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE) {
    const auto * nested = static_cast<const typename Traits::Members *>(member.members_->data);
    return write_members<Traits>(nested, field, writer, error);
  }
  error = std::string("member '") + member.name_ +
    "' has a type that the SOME/IP serializer does not support";
  return false;
}

template<typename Traits>
bool write_members(
  const typename Traits::Members * members,
  const void * ros_message,
  Writer & writer,
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

    const bool fixed = is_fixed_array(member);
    size_t count = member.array_size_;
    size_t length_position = 0;
    if (!fixed) {
      if (member.size_function == nullptr) {
        error = std::string("member '") + member.name_ + "' has no size_function";
        return false;
      }
      count = member.size_function(field);
      // The length is a byte count, not an element count, so fill it in
      // after writing the elements.
      length_position = writer.reserve_length();
    }

    const size_t width = primitive_size(member.type_id_);
    // A dynamic sequence's field points at the container, so reading its raw
    // bytes would emit internal pointers — and std::vector<bool> is bit-packed,
    // where the elements are not even contiguous bytes. Dynamic sequences must
    // go through the accessors.
    if (!fixed && member.get_const_function == nullptr && member.fetch_function == nullptr) {
      error = std::string("member '") + member.name_ +
        "' is a sequence but provides neither get_const_function nor fetch_function";
      return false;
    }
    // Copy arrays of 1-byte elements in one go, for the reason given at
    // the same place in type_support.cpp. Only width 1: SOME/IP is big endian,
    // and anything wider needs write_primitive to reverse each element.
    if (width == 1 && count > 0) {
      const void * elements = (member.get_const_function != nullptr) ?
        member.get_const_function(field, 0) :
        (fixed && member.fetch_function == nullptr ? field : nullptr);
      if (elements != nullptr) {
        writer.write_raw(elements, count);
        if (!fixed) {
          writer.fill_length(length_position);
        }
        continue;
      }
    }

    // Fixed-size array without accessors: step by the element's real size.
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

    if (!fixed) {
      writer.fill_length(length_position);
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Skipping, needed to count the elements of a dynamic array
// ---------------------------------------------------------------------------

template<typename Traits>
bool skip_members(
  const typename Traits::Members * members,
  Reader & reader);

template<typename Traits>
bool skip_single(const typename Traits::Member & member, Reader & reader)
{
  const size_t width = primitive_size(member.type_id_);
  if (width != 0) {
    return reader.skip(width);
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_STRING) {
    uint32_t encoded = 0;
    return reader.read_uint32(encoded) && reader.skip(encoded);
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE) {
    const auto * nested = static_cast<const typename Traits::Members *>(member.members_->data);
    return skip_members<Traits>(nested, reader);
  }
  return false;
}

template<typename Traits>
bool skip_members(
  const typename Traits::Members * members,
  Reader & reader)
{
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    if (!member.is_array_) {
      if (!skip_single<Traits>(member, reader)) {
        return false;
      }
      continue;
    }
    if (is_fixed_array(member)) {
      for (size_t index = 0; index < member.array_size_; ++index) {
        if (!skip_single<Traits>(member, reader)) {
          return false;
        }
      }
    } else {
      uint32_t byte_length = 0;
      if (!reader.read_uint32(byte_length) || !reader.skip(byte_length)) {
        return false;
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------

template<typename Traits>
bool read_members(
  const typename Traits::Members * members,
  void * ros_message,
  Reader & reader,
  std::string & error);

template<typename Traits>
bool read_single(
  const typename Traits::Member & member,
  void * field,
  Reader & reader,
  std::string & error)
{
  const size_t width = primitive_size(member.type_id_);
  if (width != 0) {
    if (!reader.read_primitive(field, width)) {
      error = "truncated SOME/IP payload while reading a primitive";
      return false;
    }
    return true;
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_STRING) {
    const char * data = nullptr;
    size_t length = 0;
    if (!reader.read_string(data, length)) {
      error = "truncated SOME/IP payload while reading a string";
      return false;
    }
    if (!Traits::string_assign(field, data, length)) {
      error = "failed to assign a string field";
      return false;
    }
    return true;
  }
  if (member.type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE) {
    const auto * nested = static_cast<const typename Traits::Members *>(member.members_->data);
    return read_members<Traits>(nested, field, reader, error);
  }
  error = std::string("member '") + member.name_ +
    "' has a type that the SOME/IP serializer does not support";
  return false;
}

template<typename Traits>
bool read_members(
  const typename Traits::Members * members,
  void * ros_message,
  Reader & reader,
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

    if (is_fixed_array(member)) {
      size_t stride = 0;
      if (member.get_function == nullptr) {
        stride = element_stride<Traits>(member);
        if (stride == 0 && member.array_size_ > 0) {
          error = std::string("member '") + member.name_ +
            "' is an array without accessors and its element size is unknown";
          return false;
        }
      }
      for (size_t index = 0; index < member.array_size_; ++index) {
        void * element = (member.get_function != nullptr) ?
          member.get_function(field, index) :
          static_cast<uint8_t *>(field) + stride * index;
        if (!read_single<Traits>(member, element, reader, error)) {
          return false;
        }
      }
      continue;
    }

    // Dynamic array. The length is a byte count, so count the elements first.
    uint32_t byte_length = 0;
    if (!reader.read_uint32(byte_length) || byte_length > reader.remaining()) {
      error = std::string("truncated SOME/IP payload at the length field of '") +
        member.name_ + "'";
      return false;
    }
    const size_t body_start = reader.position();
    const size_t body_end = body_start + byte_length;

    size_t count = 0;
    const size_t width = primitive_size(member.type_id_);
    if (width != 0) {
      // Fixed-size elements: divide.
      if (byte_length % width != 0) {
        error = std::string("the length of '") + member.name_ +
          "' is not a multiple of the element size";
        return false;
      }
      count = byte_length / width;
    } else {
      // Strings and nested structs vary in size: skip over them and count.
      while (reader.position() < body_end) {
        if (!skip_single<Traits>(member, reader) || reader.position() > body_end) {
          error = std::string("failed to determine the element count of '") +
            member.name_ + "'";
          return false;
        }
        ++count;
      }
      reader.seek(body_start);
    }

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
    // A dynamic sequence's field points at the container, so without an
    // accessor the elements are unreachable, as on the write side.
    if (member.get_function == nullptr && member.assign_function == nullptr) {
      error = std::string("member '") + member.name_ +
        "' is a sequence but provides neither get_function nor assign_function";
      return false;
    }

    // Bulk read of 1-byte elements, as on the write side.
    if (width == 1 && count > 0 && member.get_function != nullptr) {
      if (!reader.read_raw(member.get_function(field, 0), count)) {
        error = std::string("truncated SOME/IP payload while reading '") + member.name_ + "'";
        return false;
      }
    } else {
      for (size_t index = 0; index < count; ++index) {
        if (member.get_function != nullptr) {
          if (!read_single<Traits>(member, member.get_function(field, index), reader, error)) {
            return false;
          }
        } else {
          alignas(16) uint8_t scratch[16] {};
          if (!read_single<Traits>(member, scratch, reader, error)) {
            return false;
          }
          member.assign_function(field, index, scratch);
        }
      }
    }
    if (reader.position() != body_end) {
      error = std::string("the encoded length of '") + member.name_ +
        "' does not match its content";
      return false;
    }
  }
  return true;
}

}  // namespace

bool serialize_message_someip(
  const TypeSupportInfo & type_support,
  const void * ros_message,
  std::vector<uint8_t> & out,
  std::string & error)
{
  if (type_support.members == nullptr || ros_message == nullptr) {
    error = "null type support or message";
    return false;
  }
  Writer writer(out);
  if (type_support.is_c) {
    return write_members<CTraits>(
      static_cast<const CTraits::Members *>(type_support.members), ros_message, writer, error);
  }
  return write_members<CppTraits>(
    static_cast<const CppTraits::Members *>(type_support.members), ros_message, writer, error);
}

bool deserialize_message_someip(
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
  Reader reader(data, size);
  if (type_support.is_c) {
    return read_members<CTraits>(
      static_cast<const CTraits::Members *>(type_support.members), ros_message, reader, error);
  }
  return read_members<CppTraits>(
    static_cast<const CppTraits::Members *>(type_support.members), ros_message, reader, error);
}

}  // namespace rmw_someip_cpp
