// Checks that this implementation's CDR output interoperates with a DDS one
// (rmw_fastrtps_cpp).
//
// Returning "cdr" from rmw_get_serialization_format() commits to producing what
// other implementations produce; without that neither rosbag2 interoperability
// nor a DDS bridge works. Both .so files are dlopened in one process and the
// same message is put through each rmw_serialize().
//
// Values are also round-tripped through this implementation's own
// rmw_deserialize() to confirm they survive.

#include <dlfcn.h>

#include <cstdio>
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/types/uint8_array.h"
#include "rmw/serialized_message.h"
#include "rmw/types.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

// The same test_msgs the official tests use, so the types that fail
// test_rmw_implementation's round trip can be reproduced here.
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/bounded_sequences.hpp"
#include "test_msgs/msg/multi_nested.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"
#include "test_msgs/msg/w_strings.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace
{

using SerializeFn = rmw_ret_t (*)(
  const void *, const rosidl_message_type_support_t *, rmw_serialized_message_t *);
using DeserializeFn = rmw_ret_t (*)(
  const rmw_serialized_message_t *, const rosidl_message_type_support_t *, void *);

struct Rmw
{
  void * handle;
  SerializeFn serialize;
  DeserializeFn deserialize;
};

Rmw load(const char * path)
{
  Rmw rmw{};
  rmw.handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (rmw.handle == nullptr) {
    std::printf("  dlopen failed: %s\n", dlerror());
    return rmw;
  }
  rmw.serialize = reinterpret_cast<SerializeFn>(dlsym(rmw.handle, "rmw_serialize"));
  rmw.deserialize = reinterpret_cast<DeserializeFn>(dlsym(rmw.handle, "rmw_deserialize"));
  return rmw;
}

int g_pass = 0;
int g_fail = 0;

/// Verify interoperability by round-tripping across implementations.
///
/// A plain byte comparison does not work: Fast DDS leaves uninitialized bytes
/// in struct padding, whose contents CDR leaves unspecified, and puts an
/// over-estimated size in buffer_length rather than what it actually wrote.
/// Asking whether the other side can read it is the meaningful check.
template<typename MsgT>
void compare(const char * label, const MsgT & msg, const Rmw & ours, const Rmw & theirs)
{
  const auto * ts = rosidl_typesupport_cpp::get_message_type_support_handle<MsgT>();
  auto allocator = rcutils_get_default_allocator();

  rmw_serialized_message_t a = rmw_get_zero_initialized_serialized_message();
  rmw_serialized_message_t b = rmw_get_zero_initialized_serialized_message();
  rcutils_uint8_array_init(&a, 0, &allocator);
  rcutils_uint8_array_init(&b, 0, &allocator);

  bool ok = true;
  const char * detail = "";

  // (1) Can Fast DDS read what this implementation wrote?
  if (ours.serialize(&msg, ts, &a) != RMW_RET_OK) {
    ok = false;
    detail = "someip serialize failed";
  } else {
    MsgT decoded_by_them;
    if (theirs.deserialize(&a, ts, &decoded_by_them) != RMW_RET_OK) {
      ok = false;
      detail = "fastrtps could not deserialize our bytes";
    } else if (!(decoded_by_them == msg)) {
      ok = false;
      detail = "fastrtps decoded our bytes to a different value";
    }
  }

  // (2) Can this implementation read what Fast DDS wrote?
  if (ok) {
    if (theirs.serialize(&msg, ts, &b) != RMW_RET_OK) {
      ok = false;
      detail = "fastrtps serialize failed";
    } else {
      MsgT decoded_by_us;
      if (ours.deserialize(&b, ts, &decoded_by_us) != RMW_RET_OK) {
        ok = false;
        detail = "we could not deserialize fastrtps bytes";
      } else if (!(decoded_by_us == msg)) {
        ok = false;
        detail = "we decoded fastrtps bytes to a different value";
      }
    }
  }

  // (3) A round trip through this implementation alone.
  if (ok) {
    MsgT round_trip;
    if (ours.deserialize(&a, ts, &round_trip) != RMW_RET_OK || !(round_trip == msg)) {
      ok = false;
      detail = "our own round trip lost data";
    }
  }

  std::printf(
    "%-30s someip=%zuB fastrtps=%zuB  %s%s%s\n",
    label, a.buffer_length, b.buffer_length,
    ok ? "OK" : "*** FAIL: ", ok ? "" : detail, "");
  std::fflush(stdout);

  ok ? ++g_pass : ++g_fail;
  rcutils_uint8_array_fini(&a);
  rcutils_uint8_array_fini(&b);
}

}  // namespace

int main()
{
  Rmw ours = load("librmw_someip_cpp.so");
  Rmw theirs = load("librmw_fastrtps_cpp.so");
  if (ours.serialize == nullptr || theirs.serialize == nullptr) {
    std::printf("failed to resolve rmw_serialize from both implementations\n");
    return 1;
  }

  // Empty, short and long strings; the long one crosses alignment boundaries.
  {
    std_msgs::msg::String m;
    compare("String(empty)", m, ours, theirs);
    m.data = "hi";
    compare("String(short)", m, ours, theirs);
    m.data = std::string(300, 'x');
    compare("String(300 chars)", m, ours, theirs);
  }

  // A nested message mixing int32 and string, which exercises padding.
  {
    std_msgs::msg::Header m;
    m.stamp.sec = -12345;
    m.stamp.nanosec = 999999999u;
    m.frame_id = "base_link";
    compare("Header(nested+string)", m, ours, theirs);
  }

  // A dynamic sequence, and nested structs inside one.
  {
    std_msgs::msg::Int32MultiArray m;
    m.layout.data_offset = 7;
    m.layout.dim.resize(2);
    m.layout.dim[0].label = "rows";
    m.layout.dim[0].size = 3;
    m.layout.dim[0].stride = 9;
    m.layout.dim[1].label = "cols";
    m.layout.dim[1].size = 3;
    m.layout.dim[1].stride = 3;
    m.data = {1, -2, 3, -4, 5};
    compare("Int32MultiArray(seq)", m, ours, theirs);
  }

  // A composite with doubles, so 8-byte alignment.
  {
    geometry_msgs::msg::PoseStamped m;
    m.header.frame_id = "map";
    m.header.stamp.sec = 42;
    m.pose.position.x = 1.5;
    m.pose.position.y = -2.25;
    m.pose.position.z = 3.125;
    m.pose.orientation.w = 1.0;
    compare("PoseStamped(doubles)", m, ours, theirs);
  }

  // A large uint8 sequence, a bool, and a nested sequence.
  {
    sensor_msgs::msg::PointCloud2 m;
    m.header.frame_id = "lidar";
    m.height = 1;
    m.width = 10;
    m.fields.resize(2);
    m.fields[0].name = "x";
    m.fields[0].offset = 0;
    m.fields[0].datatype = 7;
    m.fields[0].count = 1;
    m.fields[1].name = "y";
    m.fields[1].offset = 4;
    m.fields[1].datatype = 7;
    m.fields[1].count = 1;
    m.is_bigendian = false;
    m.point_step = 8;
    m.row_step = 80;
    m.data.resize(80);
    for (size_t i = 0; i < m.data.size(); ++i) {
      m.data[i] = static_cast<uint8_t>(i);
    }
    m.is_dense = true;
    compare("PointCloud2(bytes+bool)", m, ours, theirs);
  }

  // test_msgs, what the official suites use; the awkward cases live here.
  {
    test_msgs::msg::BasicTypes m;
    m.bool_value = true;
    m.int64_value = -1234567890123LL;
    m.float64_value = 3.14159;
    compare("test_msgs/BasicTypes", m, ours, theirs);
  }
  {
    test_msgs::msg::Arrays m;   // fixed-size arrays
    m.string_values[0] = "a";
    m.string_values[1] = "bb";
    m.int32_values[2] = -7;
    m.bool_values[1] = true;
    compare("test_msgs/Arrays(fixed)", m, ours, theirs);
  }
  {
    test_msgs::msg::UnboundedSequences empty;   // every field default
    compare("UnboundedSeq(empty)", empty, ours, theirs);
  }
  {
    test_msgs::msg::UnboundedSequences only_nested;
    only_nested.basic_types_values.resize(1);   // only the nested message sequence
    compare("UnboundedSeq(nested only)", only_nested, ours, theirs);
  }
  {
    test_msgs::msg::UnboundedSequences only_str;
    only_str.string_values = {"x", "yy"};
    compare("UnboundedSeq(strings only)", only_str, ours, theirs);
  }
  {
    test_msgs::msg::UnboundedSequences m;
    m.int32_values = {1, -2, 3};
    m.string_values = {"x", "yy"};
    m.basic_types_values.resize(2);
    m.basic_types_values[0].int32_value = 11;
    compare("test_msgs/UnboundedSeq", m, ours, theirs);
  }
  {
    test_msgs::msg::BoundedSequences m;   // the type the official test failed on
    m.int32_values = {1, -2, 3};
    m.string_values = {"x", "yy"};
    m.basic_types_values.resize(2);
    m.basic_types_values[0].int32_value = 11;
    compare("test_msgs/BoundedSeq", m, ours, theirs);
  }
  {
    test_msgs::msg::WStrings m;   // a wstring length counts characters, no NUL
    m.wstring_value = u"こんにちは";   // non-ASCII, to catch a UTF-16 mistake
    m.array_of_wstrings[0] = u"ab";
    m.bounded_sequence_of_wstrings = {u"x", u"yz"};
    compare("test_msgs/WStrings", m, ours, theirs);
  }
  {
    test_msgs::msg::MultiNested m;
    m.array_of_arrays[0].int32_values = {1, 2};
    compare("test_msgs/MultiNested", m, ours, theirs);
  }

  std::printf("\n=== pass=%d fail=%d ===\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
