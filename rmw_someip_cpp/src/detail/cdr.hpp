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

#ifndef DETAIL__CDR_HPP_
#define DETAIL__CDR_HPP_

#include <cstdint>
#include <cstring>
#include <vector>

namespace rmw_someip_cpp
{

/// CDR (Common Data Representation) primitives.
///
/// Since rmw_get_serialization_format() returns "cdr", the bytes have to match
/// what DDS implementations produce — that is what lets a rosbag2 recording be
/// read back under another RMW, and a DDS bridge work at all.
///
/// The rules that matter:
///   - The first 4 bytes are the encapsulation header. Alignment is measured
///     from after the header, not from the start of the buffer.
///   - Primitives align to their own size (1/2/4/8).
///   - A string is a uint32 length including the NUL, then the bytes, then NUL.
///   - A sequence is a uint32 element count then the elements; a fixed-size
///     array is just the elements.
namespace cdr
{

/// Length of the encapsulation header.
constexpr size_t kEncapsulationSize = 4;

/// Encapsulation header for little-endian plain CDR.
constexpr uint8_t kEncapsulationLE[kEncapsulationSize] = {0x00, 0x01, 0x00, 0x00};

class Writer
{
public:
  explicit Writer(std::vector<uint8_t> & buffer)
  : buffer_(buffer)
  {
    buffer_.insert(buffer_.end(), kEncapsulationLE, kEncapsulationLE + kEncapsulationSize);
    origin_ = buffer_.size();
  }

  void align(size_t alignment)
  {
    const size_t offset = buffer_.size() - origin_;
    const size_t padding = (alignment - (offset % alignment)) % alignment;
    buffer_.insert(buffer_.end(), padding, 0);
  }

  template<typename T>
  void write(T value)
  {
    align(sizeof(T));
    const auto * bytes = reinterpret_cast<const uint8_t *>(&value);
    buffer_.insert(buffer_.end(), bytes, bytes + sizeof(T));
  }

  void write_bytes(const void * data, size_t size)
  {
    const auto * bytes = static_cast<const uint8_t *>(data);
    buffer_.insert(buffer_.end(), bytes, bytes + size);
  }

  /// Write a string. The length includes the terminating NUL.
  void write_string(const char * data, size_t length)
  {
    write<uint32_t>(static_cast<uint32_t>(length + 1));
    if (length > 0) {
      write_bytes(data, length);
    }
    buffer_.push_back(0);
  }

  /// Write a wstring. The length counts characters and excludes the NUL.
  ///
  /// Each character occupies 4 bytes. ROS represents wstrings as UTF-16
  /// (u16string), but rosidl_typesupport_fastrtps_cpp converts to std::wstring
  /// before encoding, and wchar_t is 4 bytes on Linux. Byte compatibility
  /// requires matching that. Verified: u"ab" becomes
  /// 02 00 00 00 | 61 00 00 00 | 62 00 00 00.
  void write_wstring(const uint16_t * data, size_t length)
  {
    write<uint32_t>(static_cast<uint32_t>(length));
    for (size_t i = 0; i < length; ++i) {
      write<uint32_t>(static_cast<uint32_t>(data[i]));
    }
  }

private:
  std::vector<uint8_t> & buffer_;
  size_t origin_ {0};
};

class Reader
{
public:
  Reader(const uint8_t * data, size_t size)
  : data_(data), size_(size)
  {
    // Skip the encapsulation header and treat its end as the alignment origin.
    // data_[1] == 0x00 would mean big endian; the ROS 2 toolchain always emits
    // little endian, so only LE is handled.
    origin_ = kEncapsulationSize;
    position_ = kEncapsulationSize;
    if (size_ < kEncapsulationSize) {
      // Shorter than the header. position_ must never be before origin_:
      // align() computes position_ - origin_ as size_t, which would wrap and
      // pass the bounds check, reading header bytes as payload. Fail up front.
      failed_ = true;
      return;
    }
    big_endian_ = (data_[1] == 0x00 || data_[1] == 0x02);
  }

  bool big_endian() const {return big_endian_;}
  bool ok() const {return !failed_;}
  size_t position() const {return position_;}

  void align(size_t alignment)
  {
    const size_t offset = position_ - origin_;
    position_ += (alignment - (offset % alignment)) % alignment;
  }

  template<typename T>
  bool read(T & value)
  {
    if (failed_) {
      return false;
    }
    align(sizeof(T));
    if (position_ + sizeof(T) > size_) {
      failed_ = true;
      return false;
    }
    std::memcpy(&value, data_ + position_, sizeof(T));
    position_ += sizeof(T);
    return true;
  }

  bool read_bytes(void * out, size_t size)
  {
    if (failed_ || position_ + size > size_) {
      failed_ = true;
      return false;
    }
    std::memcpy(out, data_ + position_, size);
    position_ += size;
    return true;
  }

  /// Point at the string body and return its length, excluding the NUL.
  bool read_string(const char *& out, size_t & length)
  {
    uint32_t encoded_length = 0;
    if (!read<uint32_t>(encoded_length)) {
      return false;
    }
    if (encoded_length == 0 || position_ + encoded_length > size_) {
      failed_ = true;
      return false;
    }
    out = reinterpret_cast<const char *>(data_ + position_);
    length = encoded_length - 1;  // drop the terminating NUL
    position_ += encoded_length;
    return true;
  }

  /// Read a wstring: a character count, then 4 bytes each (see write_wstring).
  bool read_wstring(std::vector<uint16_t> & out)
  {
    uint32_t length = 0;
    if (!read<uint32_t>(length)) {
      return false;
    }
    if (position_ + static_cast<size_t>(length) * 4u > size_) {
      failed_ = true;
      return false;
    }
    out.resize(length);
    for (uint32_t i = 0; i < length; ++i) {
      uint32_t character = 0;
      if (!read<uint32_t>(character)) {
        return false;
      }
      out[i] = static_cast<uint16_t>(character);
    }
    return true;
  }

  void fail() {failed_ = true;}

private:
  const uint8_t * data_;
  size_t size_;
  size_t position_ {0};
  size_t origin_ {0};
  bool big_endian_ {false};
  bool failed_ {false};
};

}  // namespace cdr
}  // namespace rmw_someip_cpp

#endif  // DETAIL__CDR_HPP_
