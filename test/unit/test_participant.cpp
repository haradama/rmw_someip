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
// Participant identity: IDs and ports.

#include <cstdlib>
#include <set>
#include <string>

#include "gtest/gtest.h"

#include "detail/participant.hpp"

TEST(MakeEntityGid, encodes_instance_and_entity_little_endian)
{
  const auto gid = rmw_someip_cpp::make_entity_gid(0x1234, 0x8009);
  EXPECT_STREQ(rmw_someip_cpp::kIdentifier, gid.implementation_identifier);
  EXPECT_EQ(0x34, gid.data[0]);
  EXPECT_EQ(0x12, gid.data[1]);
  EXPECT_EQ(0x09, gid.data[2]);
  EXPECT_EQ(0x80, gid.data[3]);
  for (size_t i = 4; i < RMW_GID_STORAGE_SIZE; ++i) {
    EXPECT_EQ(0, gid.data[i]) << "i=" << i;
  }
}

/// A receiver rebuilds the sender's gid to compare it, so the same input must
/// always give the same result for rmw_compare_gids_equal() to work.
TEST(MakeEntityGid, is_deterministic_and_distinct)
{
  EXPECT_EQ(0, std::memcmp(
      rmw_someip_cpp::make_entity_gid(1, 2).data,
      rmw_someip_cpp::make_entity_gid(1, 2).data, RMW_GID_STORAGE_SIZE));
  EXPECT_NE(0, std::memcmp(
      rmw_someip_cpp::make_entity_gid(1, 2).data,
      rmw_someip_cpp::make_entity_gid(1, 3).data, RMW_GID_STORAGE_SIZE));
  EXPECT_NE(0, std::memcmp(
      rmw_someip_cpp::make_entity_gid(1, 2).data,
      rmw_someip_cpp::make_entity_gid(2, 2).data, RMW_GID_STORAGE_SIZE));
}

TEST(ParticipantIdentity, derives_ids_from_the_domain)
{
  ::unsetenv("RMW_SOMEIP_SERVICE_ID_BASE");
  ::unsetenv("RMW_SOMEIP_SD_PORT_BASE");

  const auto zero = rmw_someip_cpp::ParticipantIdentity::create(0);
  const auto seven = rmw_someip_cpp::ParticipantIdentity::create(7);

  // Service IDs are offset per domain.
  EXPECT_EQ(0xF000, zero.service_id);
  EXPECT_EQ(0xF007, seven.service_id);
  // SD ports are separated per domain, as DDS separates its ports.
  EXPECT_EQ(30490, zero.sd_port);
  EXPECT_EQ(30490 + 14, seven.sd_port);

  // 0x0000 and 0xFFFF (ANY_INSTANCE) are reserved.
  EXPECT_NE(0x0000, zero.instance_id);
  EXPECT_NE(0xFFFF, zero.instance_id);
  EXPECT_NE(0u, zero.guid);
  EXPECT_FALSE(zero.application_name.empty());
}

TEST(ParticipantIdentity, honours_the_environment_overrides)
{
  ::setenv("RMW_SOMEIP_SERVICE_ID_BASE", "0x9000", 1);
  ::setenv("RMW_SOMEIP_SD_PORT_BASE", "40000", 1);
  const auto id = rmw_someip_cpp::ParticipantIdentity::create(1);
  EXPECT_EQ(0x9001, id.service_id);
  EXPECT_EQ(40002, id.sd_port);
  ::unsetenv("RMW_SOMEIP_SERVICE_ID_BASE");
  ::unsetenv("RMW_SOMEIP_SD_PORT_BASE");
}

/// Ports are reserved before use, so repeated calls in one process differ.
///
/// The derived value alone collides once several processes share a network
/// namespace.
TEST(ParticipantIdentity, reserves_distinct_ports)
{
  std::set<uint16_t> unreliable;
  std::set<uint16_t> reliable;
  for (int i = 0; i < 8; ++i) {
    const auto id = rmw_someip_cpp::ParticipantIdentity::create(0);
    EXPECT_GE(id.unreliable_port, 31000);
    EXPECT_LT(id.unreliable_port, 32000);
    EXPECT_GE(id.reliable_port, 32000);
    EXPECT_LT(id.reliable_port, 33000);
    EXPECT_TRUE(unreliable.insert(id.unreliable_port).second)
      << "returned the same unreliable port twice";
    EXPECT_TRUE(reliable.insert(id.reliable_port).second)
      << "returned the same reliable port twice";
  }
}
