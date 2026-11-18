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
// Generating the vsomeip JSON configuration.
//
// vsomeip starts regardless of what is wrong here; the only symptom is that no
// SD offer goes out (docs/design.md §3). So the output is inspected directly.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

#include "detail/config_generator.hpp"
#include "detail/extern_mapping.hpp"
#include "detail/participant.hpp"

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

rmw_someip_cpp::ParticipantIdentity make_identity()
{
  auto id = rmw_someip_cpp::ParticipantIdentity::create(0);
  id.instance_id = 0x0042;
  return id;
}

/// Read the generated configuration and clean it up.
class GeneratedConfig
{
public:
  GeneratedConfig(
    const rmw_someip_cpp::ParticipantIdentity & id,
    bool localhost_only,
    const rmw_someip_cpp::ExternMapping & mapping)
  {
    config_ = rmw_someip_cpp::ConfigGenerator::generate(id, localhost_only, mapping, error_);
    if (!config_.path.empty()) {
      content_ = read_file(config_.path);
    }
  }
  ~GeneratedConfig() {rmw_someip_cpp::ConfigGenerator::cleanup(config_);}
  const std::string & path() const {return config_.path;}
  bool generated() const {return config_.generated;}
  const std::string & content() const {return content_;}
  const std::string & error() const {return error_;}

private:
  rmw_someip_cpp::ConfigGenerator::Config config_;
  std::string content_;
  std::string error_;
};

}  // namespace

TEST(ConfigGenerator, writes_the_participant_service_with_ports)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");
  const auto id = make_identity();
  rmw_someip_cpp::ExternMapping mapping;
  GeneratedConfig config(id, true, mapping);

  ASSERT_FALSE(config.path().empty()) << config.error();
  const auto & json = config.content();

  // Without a port on the service declaration, no SD offer is sent.
  EXPECT_NE(std::string::npos, json.find("\"unicast\""));
  EXPECT_NE(std::string::npos, json.find("\"services\""));
  EXPECT_NE(std::string::npos, json.find(std::to_string(id.unreliable_port)));
  EXPECT_NE(std::string::npos, json.find(std::to_string(id.reliable_port)));
  EXPECT_NE(std::string::npos, json.find(std::to_string(id.sd_port)));
  EXPECT_NE(std::string::npos, json.find(id.application_name));
  // Nothing is discovered unless service discovery is enabled.
  EXPECT_NE(std::string::npos, json.find("\"service-discovery\""));
  EXPECT_NE(std::string::npos, json.find("\"enable\": \"true\""));

  // localhost_only confines it to loopback.
  EXPECT_NE(std::string::npos, json.find("127.0.0.1"));
}

/// "routing" must not be written.
///
/// With it, the second process fails init() complaining that another routing
/// manager is present. Omitted, vsomeip treats the first application to start
/// as the routing manager (docs/design.md §16(j)).
TEST(ConfigGenerator, omits_routing_unless_asked)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");
  rmw_someip_cpp::ExternMapping mapping;
  GeneratedConfig config(make_identity(), true, mapping);
  ASSERT_FALSE(config.path().empty()) << config.error();
  EXPECT_EQ(std::string::npos, config.content().find("\"routing\""));
}

TEST(ConfigGenerator, writes_routing_when_the_host_is_given)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::setenv("RMW_SOMEIP_ROUTING_HOST", "someip_routingd", 1);
  rmw_someip_cpp::ExternMapping mapping;
  GeneratedConfig config(make_identity(), true, mapping);
  ASSERT_FALSE(config.path().empty()) << config.error();
  EXPECT_NE(std::string::npos, config.content().find("\"routing\""));
  EXPECT_NE(std::string::npos, config.content().find("someip_routingd"));
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");
}

/// SOME/IP-TP declarations.
///
/// A topic larger than UDP's maximum message length (1416 bytes by default) is
/// silently dropped unless TP is declared. The declaration lists event IDs
/// individually, so a fixed number are declared up front (docs/design.md §3).
TEST(ConfigGenerator, declares_someip_tp_for_topic_events)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");
  ::setenv("RMW_SOMEIP_TP_EVENT_COUNT", "4", 1);
  rmw_someip_cpp::ExternMapping mapping;
  GeneratedConfig config(make_identity(), true, mapping);
  ASSERT_FALSE(config.path().empty()) << config.error();
  EXPECT_NE(std::string::npos, config.content().find("someip-tp"));
  EXPECT_NE(std::string::npos, config.content().find("max-segment-length"));
  ::unsetenv("RMW_SOMEIP_TP_EVENT_COUNT");
}

/// A topic the ROS side offers in extern mode needs its own service declared
/// with a port, separate from the participant's.
TEST(ConfigGenerator, declares_extern_services_that_ros_offers)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");

  const std::string yaml_path = "/tmp/rmw_someip_cfg_test.yaml";
  {
    std::ofstream out(yaml_path);
    out <<
      "externs:\n"
      "  /to_ecu:\n"
      "    direction: publish\n"
      "    service_id: \"0x4321\"\n"
      "    instance_id: \"0x0001\"\n"
      "    event_id: \"0x8001\"\n"
      "    eventgroup_id: \"0x0001\"\n"
      "    port: \"30599\"\n"
      "  /to_ecu_tcp:\n"
      "    direction: publish\n"
      "    service_id: \"0x4322\"\n"
      "    instance_id: \"0x0001\"\n"
      "    event_id: \"0x8001\"\n"
      "    eventgroup_id: \"0x0001\"\n"
      "    reliable: true\n"
      "    port: \"30598\"\n"
      "  /from_ecu:\n"
      "    direction: subscribe\n"
      "    service_id: \"0x5678\"\n"
      "    instance_id: \"0x0001\"\n"
      "    event_id: \"0x8001\"\n"
      "    eventgroup_id: \"0x0001\"\n";
  }
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  ASSERT_TRUE(rmw_someip_cpp::ExternMapping::load(yaml_path, mapping, error)) << error;

  GeneratedConfig config(make_identity(), true, mapping);
  ASSERT_FALSE(config.path().empty()) << config.error();
  // The offered ones are declared with a port: UDP as "unreliable", TCP as
  // "reliable" with magic cookies disabled.
  EXPECT_NE(std::string::npos, config.content().find("\"unreliable\": \"30599\""));
  EXPECT_NE(std::string::npos, config.content().find("\"port\": \"30598\""));
  EXPECT_NE(std::string::npos, config.content().find("enable-magic-cookies"));
  // A service that is only subscribed to needs no declaration; its endpoint
  // is learned from SD.
  std::remove(yaml_path.c_str());
}

/// The escape hatch for a hand-written configuration.
///
/// Ownership must not be guessed from the path: a user's file that looks like
/// a generated one has to survive shutdown.
TEST(ConfigGenerator, never_deletes_a_user_supplied_config)
{
  // Deliberately the same prefix the generated file uses.
  const std::string user_path = "/tmp/rmw_someip_user_config.json";
  {
    std::ofstream out(user_path);
    out << "{}";
  }
  ::setenv("RMW_SOMEIP_CONFIG_FILE", user_path.c_str(), 1);

  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  const auto config =
    rmw_someip_cpp::ConfigGenerator::generate(make_identity(), true, mapping, error);
  EXPECT_EQ(user_path, config.path);
  EXPECT_FALSE(config.generated);

  rmw_someip_cpp::ConfigGenerator::cleanup(config);
  std::ifstream check(user_path);
  EXPECT_TRUE(check.good()) << "deleted the user's own configuration file";

  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  std::remove(user_path.c_str());
}

TEST(ConfigGenerator, cleanup_removes_the_generated_file)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");
  rmw_someip_cpp::ExternMapping mapping;
  std::string error;
  const auto config =
    rmw_someip_cpp::ConfigGenerator::generate(make_identity(), true, mapping, error);
  ASSERT_FALSE(config.path.empty()) << error;
  EXPECT_TRUE(config.generated);
  {
    std::ifstream check(config.path);
    ASSERT_TRUE(check.good());
  }
  rmw_someip_cpp::ConfigGenerator::cleanup(config);
  std::ifstream check(config.path);
  EXPECT_FALSE(check.good());

  // An empty or nonexistent path must not crash.
  rmw_someip_cpp::ConfigGenerator::cleanup({});
  rmw_someip_cpp::ConfigGenerator::cleanup({"/tmp/definitely_not_here.json", true});
}

/// Without localhost_only, a real interface address is picked up.
TEST(ConfigGenerator, detects_a_non_loopback_address)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");
  ::unsetenv("RMW_SOMEIP_UNICAST_ADDRESS");
  rmw_someip_cpp::ExternMapping mapping;
  GeneratedConfig config(make_identity(), false, mapping);
  ASSERT_FALSE(config.path().empty()) << config.error();
  EXPECT_NE(std::string::npos, config.content().find("\"unicast\""));
}

/// The address can be pinned, which a container needs to match its real IP.
TEST(ConfigGenerator, honours_an_explicit_unicast_address)
{
  ::unsetenv("RMW_SOMEIP_CONFIG_FILE");
  ::unsetenv("RMW_SOMEIP_ROUTING_HOST");
  ::setenv("RMW_SOMEIP_UNICAST_ADDRESS", "172.30.0.9", 1);
  rmw_someip_cpp::ExternMapping mapping;
  GeneratedConfig config(make_identity(), false, mapping);
  ASSERT_FALSE(config.path().empty()) << config.error();
  EXPECT_NE(std::string::npos, config.content().find("172.30.0.9"));
  ::unsetenv("RMW_SOMEIP_UNICAST_ADDRESS");
}
