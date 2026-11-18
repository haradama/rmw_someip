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

#ifndef DETAIL__PARTICIPANT_HPP_
#define DETAIL__PARTICIPANT_HPP_

#include <cstdint>
#include <cstring>
#include <string>

#include "rmw/types.h"

#include "rmw_someip_cpp/identifier.hpp"

namespace rmw_someip_cpp
{

/// Build an entity gid.
///
/// Layout: [0..1] instance_id (LE), [2..3] entity_id (LE), rest zero. The
/// entity_id is the event_id for a publisher, or a sequential number for a client.
///
/// The receiver rebuilds the sender's gid to compare it. SOME/IP messages
/// carry no gid, so on receipt it is reconstructed from (instance_id, event_id)
/// by this same rule to make rmw_compare_gids_equal() work
/// (rmw_context_impl_s::on_someip_message). A second copy of the rule that
/// drifts would silently break publisher identity, so keep it only here.
inline rmw_gid_t make_entity_gid(uint16_t instance_id, uint16_t entity_id)
{
  rmw_gid_t gid {};
  gid.implementation_identifier = kIdentifier;
  std::memset(gid.data, 0, RMW_GID_STORAGE_SIZE);
  gid.data[0] = static_cast<uint8_t>(instance_id & 0xFF);
  gid.data[1] = static_cast<uint8_t>((instance_id >> 8) & 0xFF);
  gid.data[2] = static_cast<uint8_t>(entity_id & 0xFF);
  gid.data[3] = static_cast<uint8_t>((entity_id >> 8) & 0xFF);
  return gid;
}

/// This ROS process's identity on SOME/IP.
///
/// The ID model: one SOME/IP service for the whole ROS
/// domain, participants told apart by instance_id. Only the values held here
/// need declaring in the vsomeip JSON, and all of them are known by the time
/// rmw_init() returns. Topics get events and eventgroups at runtime, so adding
/// one never requires a configuration change.
struct ParticipantIdentity
{
  /// Well-known service ID, shared by the whole domain.
  uint16_t service_id;
  /// Instance ID identifying this participant.
  uint16_t instance_id;
  /// 64-bit fingerprint of the participant, derived from hostname and pid.
  ///
  /// instance_id is only 16 bits, so it must collide once a domain holds 65534
  /// participants. On a collision a peer's graph looks like this process's own
  /// echo and gets dropped, which shows up as discovery mysteriously failing.
  /// The graph channel carries this wider value so the collision is detectable.
  uint64_t guid;
  /// SOME/IP-SD port, offset per domain.
  uint16_t sd_port;
  /// Port for BEST_EFFORT (UDP).
  uint16_t unreliable_port;
  /// Port for RELIABLE (TCP).
  uint16_t reliable_port;
  /// vsomeip application name; must match the JSON's applications entry.
  std::string application_name;

  /// Derive from the domain id and per-process information.
  ///
  /// Ports are reserved before use. Using the derived value directly
  /// collides once several processes share a network namespace, because the
  /// port range is small. A collision makes one vsomeip fail to bind, and it
  /// cannot communicate at all.
  ///
  /// instance_id is hashed from hostname and pid, so it essentially never
  /// collides on one host. Collisions across hosts are not
  /// prevented, but vsomeip catches them during discovery and reports
  /// "Rejecting routing info ... offered previously on this node".
  static ParticipantIdentity create(size_t domain_id);
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__PARTICIPANT_HPP_
