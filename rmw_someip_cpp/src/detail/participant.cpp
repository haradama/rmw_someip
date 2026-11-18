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

#include "detail/participant.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace rmw_someip_cpp
{

namespace
{

constexpr uint16_t kDefaultServiceIdBase = 0xF000;
constexpr uint16_t kDefaultSdPortBase = 30490;
constexpr uint16_t kUnreliablePortBase = 31000;
constexpr uint16_t kReliablePortBase = 32000;
constexpr uint16_t kPortSpan = 1000;

uint64_t fnv1a(const std::string & s, uint64_t seed = 1469598103934665603ULL)
{
  uint64_t h = seed;
  for (unsigned char c : s) {
    h ^= c;
    h *= 1099511628211ULL;
  }
  return h;
}

uint16_t env_u16(const char * name, uint16_t fallback)
{
  const char * v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return fallback;
  }
  return static_cast<uint16_t>(std::strtoul(v, nullptr, 0));
}

/// Test whether a port can actually be bound.
///
/// Deliberately without SO_REUSEADDR: with it, binding can succeed on a port
/// already in use, which would not answer the question.
bool port_is_free(uint16_t port, int type)
{
  const int fd = ::socket(AF_INET, type, 0);
  if (fd < 0) {
    return false;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  const bool ok =
    ::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) == 0;
  ::close(fd);
  return ok;
}

/// Lock descriptors held for the life of the process.
///
/// Deliberately never closed. The OS releases them at exit, which is
/// exactly the intended meaning: reserved while this participant lives.
std::vector<int> & held_port_locks()
{
  static std::vector<int> locks;
  return locks;
}

/// Reserve a port.
///
/// Probing with bind and closing is not enough: between the close and
/// vsomeip's real bind, another process can see the same port as free.
///
/// So a lock file under /tmp is used as well. vsomeip already uses /tmp for
/// local routing, so what /tmp is shared with matches what a port can collide
/// with (the network namespace), and this excludes other rmw_someip processes
/// on the host reliably. The bind probe stays to catch ports held by something
/// that is not rmw_someip.
///
/// The lock files stay in /tmp, empty, one per port and reused.
bool reserve_port(uint16_t port, int type)
{
  const std::string path = "/tmp/rmw_someip_port_" + std::to_string(port) + ".lock";
  const int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
  if (fd < 0) {
    // Where /tmp is not writable, fall back to the bind probe alone.
    return port_is_free(port, type);
  }
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd);  // another rmw_someip process holds it
    return false;
  }
  if (!port_is_free(port, type)) {
    ::close(fd);  // in use by something else
    return false;
  }
  held_port_locks().push_back(fd);
  return true;
}

/// Scan for a free port from base + (offset % span).
///
/// The derived value alone collides once several processes share a network
/// namespace, because the span is small; one vsomeip then fails to bind and
/// cannot communicate. test_rclcpp's test_n_nodes starts enough processes to
/// hit it.
uint16_t pick_port(uint16_t base, uint16_t span, uint16_t offset, int type)
{
  for (uint16_t k = 0; k < span; ++k) {
    const auto port = static_cast<uint16_t>(base + ((offset + k) % span));
    if (reserve_port(port, type)) {
      return port;
    }
  }
  // All taken. Fall back to the derived value; vsomeip reports the bind failure.
  return static_cast<uint16_t>(base + (offset % span));
}

}  // namespace

ParticipantIdentity ParticipantIdentity::create(size_t domain_id)
{
  std::array<char, 256> host{};
  if (gethostname(host.data(), host.size() - 1) != 0) {
    host[0] = '\0';
  }

  const auto pid = static_cast<uint64_t>(getpid());
  const uint64_t h = fnv1a(std::string(host.data()) + "#" + std::to_string(pid));

  // 0x0000 and 0xFFFF (ANY_INSTANCE) are reserved.
  auto instance = static_cast<uint16_t>((h % 0xFFFE) + 1);

  ParticipantIdentity id;
  id.service_id = static_cast<uint16_t>(
    env_u16("RMW_SOMEIP_SERVICE_ID_BASE", kDefaultServiceIdBase) + (domain_id & 0xFF));
  id.instance_id = instance;
  id.guid = h;
  // Separate discovery per domain, as DDS separates ports per domain. The SD
  // port is multicast and shared by every participant, so it is not probed.
  id.sd_port = static_cast<uint16_t>(
    env_u16("RMW_SOMEIP_SD_PORT_BASE", kDefaultSdPortBase) + domain_id * 2);
  id.unreliable_port =
    pick_port(kUnreliablePortBase, kPortSpan, static_cast<uint16_t>(instance), SOCK_DGRAM);
  id.reliable_port =
    pick_port(kReliablePortBase, kPortSpan, static_cast<uint16_t>(instance), SOCK_STREAM);
  id.application_name = "rmw_someip_" + std::to_string(pid);
  return id;
}

}  // namespace rmw_someip_cpp
