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

#include "detail/graph_message.hpp"

#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace rmw_someip_cpp
{

namespace
{

constexpr uint32_t kMagic = 0x49475352u;  // "RSGI" little endian
constexpr uint16_t kVersion = 3u;  // v3 added the RIHS type hash and participant guid

// Per-message cap, so malformed or hostile input cannot force a huge allocation.
constexpr uint32_t kMaxCount = 100000u;
constexpr uint32_t kMaxStringLength = 65535u;

template<typename T>
void put(std::vector<uint8_t> & buffer, T value)
{
  const auto * bytes = reinterpret_cast<const uint8_t *>(&value);
  buffer.insert(buffer.end(), bytes, bytes + sizeof(T));
}

void put_string(std::vector<uint8_t> & buffer, const std::string & value)
{
  put<uint32_t>(buffer, static_cast<uint32_t>(value.size()));
  buffer.insert(buffer.end(), value.begin(), value.end());
}

class Cursor
{
public:
  Cursor(const uint8_t * data, size_t size)
  : data_(data), size_(size) {}

  template<typename T>
  bool get(T & value)
  {
    if (position_ + sizeof(T) > size_) {
      return false;
    }
    std::memcpy(&value, data_ + position_, sizeof(T));
    position_ += sizeof(T);
    return true;
  }

  bool get_string(std::string & value)
  {
    uint32_t length = 0;
    if (!get<uint32_t>(length) || length > kMaxStringLength) {
      return false;
    }
    if (position_ + length > size_) {
      return false;
    }
    value.assign(reinterpret_cast<const char *>(data_ + position_), length);
    position_ += length;
    return true;
  }

  bool get_bytes(void * out, size_t length)
  {
    if (position_ + length > size_) {
      return false;
    }
    std::memcpy(out, data_ + position_, length);
    position_ += length;
    return true;
  }

private:
  const uint8_t * data_;
  size_t size_;
  size_t position_ {0};
};

}  // namespace

std::vector<uint8_t> encode_graph_message(const ParticipantEntitiesInfo & info)
{
  std::vector<uint8_t> buffer;
  buffer.reserve(256 + info.endpoints.size() * 128);

  put<uint32_t>(buffer, kMagic);
  put<uint16_t>(buffer, kVersion);
  put<uint16_t>(buffer, info.instance_id);
  put<uint64_t>(buffer, info.guid);

  put<uint32_t>(buffer, static_cast<uint32_t>(info.nodes.size()));
  for (const auto & node : info.nodes) {
    put_string(buffer, node.node_namespace);
    put_string(buffer, node.node_name);
    put_string(buffer, node.enclave);
  }

  put<uint32_t>(buffer, static_cast<uint32_t>(info.endpoints.size()));
  for (const auto & endpoint : info.endpoints) {
    put<uint8_t>(buffer, static_cast<uint8_t>(endpoint.kind));
    put_string(buffer, endpoint.node_namespace);
    put_string(buffer, endpoint.node_name);
    put_string(buffer, endpoint.topic_name);
    put_string(buffer, endpoint.type_name);
    put<uint8_t>(buffer, endpoint.type_hash.version);
    buffer.insert(
      buffer.end(), endpoint.type_hash.value,
      endpoint.type_hash.value + ROSIDL_TYPE_HASH_SIZE);
    put<uint16_t>(buffer, endpoint.event_id);
    put<uint16_t>(buffer, endpoint.eventgroup_id);
    put<uint16_t>(buffer, endpoint.method_id);
    put<uint8_t>(buffer, endpoint.reliability);
    put<uint8_t>(buffer, endpoint.durability);
    put<uint8_t>(buffer, endpoint.history);
    put<uint32_t>(buffer, endpoint.depth);
    put<uint8_t>(buffer, endpoint.liveliness);
    put<int64_t>(buffer, endpoint.deadline_sec);
    put<uint64_t>(buffer, endpoint.deadline_nsec);
    put<int64_t>(buffer, endpoint.lifespan_sec);
    put<uint64_t>(buffer, endpoint.lifespan_nsec);
    put<int64_t>(buffer, endpoint.liveliness_lease_sec);
    put<uint64_t>(buffer, endpoint.liveliness_lease_nsec);
    buffer.insert(buffer.end(), endpoint.gid, endpoint.gid + RMW_GID_STORAGE_SIZE);
  }
  return buffer;
}

bool decode_graph_message(
  const uint8_t * data,
  size_t size,
  ParticipantEntitiesInfo & out,
  std::string & error)
{
  Cursor cursor(data, size);

  uint32_t magic = 0;
  uint16_t version = 0;
  if (!cursor.get<uint32_t>(magic) || magic != kMagic) {
    error = "graph message has a bad magic number";
    return false;
  }
  if (!cursor.get<uint16_t>(version)) {
    error = "graph message is truncated at the version field";
    return false;
  }
  if (version != kVersion) {
    error =
      "graph message version " + std::to_string(version) +
      " is not supported (expected " + std::to_string(kVersion) + ")";
    return false;
  }
  if (!cursor.get<uint16_t>(out.instance_id)) {
    error = "graph message is truncated at the participant instance id";
    return false;
  }
  if (!cursor.get<uint64_t>(out.guid)) {
    error = "graph message is truncated at the participant guid";
    return false;
  }

  uint32_t node_count = 0;
  if (!cursor.get<uint32_t>(node_count) || node_count > kMaxCount) {
    error = "graph message has an invalid node count";
    return false;
  }
  out.nodes.clear();
  out.nodes.reserve(node_count);
  for (uint32_t i = 0; i < node_count; ++i) {
    ParticipantEntitiesInfo::NodeInfo node;
    if (!cursor.get_string(node.node_namespace) ||
      !cursor.get_string(node.node_name) ||
      !cursor.get_string(node.enclave))
    {
      error = "graph message is truncated inside the node list";
      return false;
    }
    out.nodes.push_back(std::move(node));
  }

  uint32_t endpoint_count = 0;
  if (!cursor.get<uint32_t>(endpoint_count) || endpoint_count > kMaxCount) {
    error = "graph message has an invalid endpoint count";
    return false;
  }
  out.endpoints.clear();
  out.endpoints.reserve(endpoint_count);
  for (uint32_t i = 0; i < endpoint_count; ++i) {
    EndpointInfo endpoint;
    uint8_t kind = 0;
    if (!cursor.get<uint8_t>(kind) ||
      !cursor.get_string(endpoint.node_namespace) ||
      !cursor.get_string(endpoint.node_name) ||
      !cursor.get_string(endpoint.topic_name) ||
      !cursor.get_string(endpoint.type_name) ||
      !cursor.get<uint8_t>(endpoint.type_hash.version) ||
      !cursor.get_bytes(endpoint.type_hash.value, ROSIDL_TYPE_HASH_SIZE) ||
      !cursor.get<uint16_t>(endpoint.event_id) ||
      !cursor.get<uint16_t>(endpoint.eventgroup_id) ||
      !cursor.get<uint16_t>(endpoint.method_id) ||
      !cursor.get<uint8_t>(endpoint.reliability) ||
      !cursor.get<uint8_t>(endpoint.durability) ||
      !cursor.get<uint8_t>(endpoint.history) ||
      !cursor.get<uint32_t>(endpoint.depth) ||
      !cursor.get<uint8_t>(endpoint.liveliness) ||
      !cursor.get<int64_t>(endpoint.deadline_sec) ||
      !cursor.get<uint64_t>(endpoint.deadline_nsec) ||
      !cursor.get<int64_t>(endpoint.lifespan_sec) ||
      !cursor.get<uint64_t>(endpoint.lifespan_nsec) ||
      !cursor.get<int64_t>(endpoint.liveliness_lease_sec) ||
      !cursor.get<uint64_t>(endpoint.liveliness_lease_nsec) ||
      !cursor.get_bytes(endpoint.gid, RMW_GID_STORAGE_SIZE))
    {
      error = "graph message is truncated inside the endpoint list";
      return false;
    }
    if (kind > static_cast<uint8_t>(EndpointKind::Client)) {
      error = "graph message contains an unknown endpoint kind";
      return false;
    }
    endpoint.kind = static_cast<EndpointKind>(kind);
    out.endpoints.push_back(std::move(endpoint));
  }

  return true;
}

}  // namespace rmw_someip_cpp
