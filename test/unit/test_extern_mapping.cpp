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
// Loading someip_mapping.yaml.
//
// A misconfiguration shows up as "starts fine but silently never communicates",
// so which mistake produces which message is worth pinning down.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

#include "detail/extern_mapping.hpp"

namespace
{

/// Write YAML to a temporary file and load it.
class TempYaml
{
public:
  explicit TempYaml(const std::string & content)
  : path_("/tmp/rmw_someip_test_" + std::to_string(++counter_) + ".yaml")
  {
    std::ofstream out(path_);
    out << content;
  }
  ~TempYaml() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  static int counter_;
  std::string path_;
};
int TempYaml::counter_ = 0;

constexpr const char * kValid = R"(
externs:
  /from_ecu:
    direction: subscribe
    service_id: "0x1234"
    instance_id: "0x0001"
    event_id: "0x8001"
    eventgroup_id: "0x0001"
    type: demo_msgs/msg/Status
    serialization: someip_native
    reliable: false
  /to_ecu:
    direction: publish
    service_id: 4661
    instance_id: 1
    event_id: 32769
    eventgroup_id: 1
    serialization: cdr
    reliable: true
    port: "30510"
)";

}  // namespace

TEST(ExternMapping, loads_a_valid_file)
{
  TempYaml yaml(kValid);
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::ExternMapping::load(yaml.path(), mapping, error)) << error;
  EXPECT_FALSE(mapping.empty());

  const auto * from_ecu = mapping.find("/from_ecu");
  ASSERT_NE(nullptr, from_ecu);
  EXPECT_EQ(0x1234, from_ecu->service_id);
  EXPECT_EQ(0x0001, from_ecu->instance_id);
  EXPECT_EQ(0x8001, from_ecu->event_id);
  EXPECT_EQ("demo_msgs/msg/Status", from_ecu->type_name);
  EXPECT_FALSE(from_ecu->ros_is_publisher);
  EXPECT_FALSE(from_ecu->reliable);
  EXPECT_EQ(rmw_someip_cpp::Serialization::SomeipNative, from_ecu->serialization);

  // Decimal gives the same value. Hex reads better in the file, but the value
  // may also be written as a plain YAML number.
  const auto * to_ecu = mapping.find("/to_ecu");
  ASSERT_NE(nullptr, to_ecu);
  EXPECT_EQ(0x1235, to_ecu->service_id);
  EXPECT_EQ(0x8001, to_ecu->event_id);
  EXPECT_TRUE(to_ecu->ros_is_publisher);
  EXPECT_TRUE(to_ecu->reliable);
  EXPECT_EQ(30510, to_ecu->port);
  EXPECT_EQ(rmw_someip_cpp::Serialization::Cdr, to_ecu->serialization);

  EXPECT_EQ(nullptr, mapping.find("/not_in_the_file"));
  EXPECT_EQ(1u, mapping.offered_topics().size());
}

TEST(ExternMapping, direction_defaults_to_subscribe)
{
  TempYaml yaml(R"(
externs:
  /t:
    service_id: 1
    instance_id: 1
    event_id: 1
    eventgroup_id: 1
)");
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::ExternMapping::load(yaml.path(), mapping, error)) << error;
  const auto * topic = mapping.find("/t");
  ASSERT_NE(nullptr, topic);
  EXPECT_FALSE(topic->ros_is_publisher);
  // serialization also defaults to someip_native.
  EXPECT_EQ(rmw_someip_cpp::Serialization::SomeipNative, topic->serialization);
}

TEST(ExternMapping, rejects_a_missing_file)
{
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  EXPECT_FALSE(
    rmw_someip_cpp::ExternMapping::load("/tmp/definitely_not_here.yaml", mapping, error));
  EXPECT_NE(std::string::npos, error.find("failed to parse"));
}

TEST(ExternMapping, rejects_a_file_without_externs)
{
  TempYaml yaml("something_else: 1\n");
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::ExternMapping::load(yaml.path(), mapping, error));
  EXPECT_NE(std::string::npos, error.find("no 'externs' section"));
}

/// A missing ID is rejected in a way that names which one.
class ExternMappingMissingId : public ::testing::TestWithParam<const char *> {};

TEST_P(ExternMappingMissingId, is_rejected)
{
  const std::string missing = GetParam();
  std::string content = "externs:\n  /t:\n";
  for (const char * key : {"service_id", "instance_id", "event_id", "eventgroup_id"}) {
    if (missing != key) {
      content += std::string("    ") + key + ": 1\n";
    }
  }
  TempYaml yaml(content);
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::ExternMapping::load(yaml.path(), mapping, error));
  EXPECT_NE(std::string::npos, error.find("has no " + missing)) << error;
}

INSTANTIATE_TEST_SUITE_P(
  AllIds, ExternMappingMissingId,
  ::testing::Values("service_id", "instance_id", "event_id", "eventgroup_id"));

TEST(ExternMapping, rejects_an_unknown_direction)
{
  TempYaml yaml(R"(
externs:
  /t:
    direction: sideways
    service_id: 1
    instance_id: 1
    event_id: 1
    eventgroup_id: 1
)");
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::ExternMapping::load(yaml.path(), mapping, error));
  EXPECT_NE(std::string::npos, error.find("invalid direction"));
}

TEST(ExternMapping, rejects_an_unknown_serialization)
{
  TempYaml yaml(R"(
externs:
  /t:
    serialization: protobuf
    service_id: 1
    instance_id: 1
    event_id: 1
    eventgroup_id: 1
)");
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::ExternMapping::load(yaml.path(), mapping, error));
  EXPECT_NE(std::string::npos, error.find("invalid serialization"));
}

/// An offered topic without a port is always rejected. Letting it through
/// makes vsomeip return success from offer_service() while sending no SD
/// offer: the process starts and is never discovered (docs/design.md §3).
TEST(ExternMapping, rejects_publish_without_a_port)
{
  TempYaml yaml(R"(
externs:
  /t:
    direction: publish
    service_id: 1
    instance_id: 1
    event_id: 1
    eventgroup_id: 1
)");
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  EXPECT_FALSE(rmw_someip_cpp::ExternMapping::load(yaml.path(), mapping, error));
  EXPECT_NE(std::string::npos, error.find("no port"));
}

TEST(ExternMapping, env_without_the_variable_is_an_empty_mapping)
{
  ::unsetenv("RMW_SOMEIP_MAPPING_FILE");
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  EXPECT_TRUE(rmw_someip_cpp::ExternMapping::load_from_env(mapping, error));
  EXPECT_TRUE(mapping.empty());

  // An empty value also means unused.
  ::setenv("RMW_SOMEIP_MAPPING_FILE", "", 1);
  EXPECT_TRUE(rmw_someip_cpp::ExternMapping::load_from_env(mapping, error));
  EXPECT_TRUE(mapping.empty());
  ::unsetenv("RMW_SOMEIP_MAPPING_FILE");
}

TEST(ExternMapping, env_pointing_at_a_file_loads_it)
{
  TempYaml yaml(kValid);
  ::setenv("RMW_SOMEIP_MAPPING_FILE", yaml.path().c_str(), 1);
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::ExternMapping::load_from_env(mapping, error)) << error;
  EXPECT_NE(nullptr, mapping.find("/from_ecu"));
  ::unsetenv("RMW_SOMEIP_MAPPING_FILE");
}
