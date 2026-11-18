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

#include "detail/config_generator.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "detail/context_impl.hpp"

namespace rmw_someip_cpp
{

namespace
{

std::string hex16(uint16_t v)
{
  std::ostringstream os;
  os << "0x" << std::hex << std::setw(4) << std::setfill('0') << v;
  return os.str();
}

/// How many event IDs get SOME/IP-TP (segmented transport) enabled.
///
/// SOME/IP's UDP has a maximum message length (1416 bytes by default) and
/// drops anything larger on send with "Dropping to big message". Large
/// BEST_EFFORT topics such as PointCloud2 hit this.
///
/// vsomeip's someip-tp configuration lists event IDs individually. Topic event
/// IDs are numbered sequentially from kFirstTopicEventId, so declaring a fixed
/// number from the start covers them. A process with more topics than that
/// raises the count through RMW_SOMEIP_TP_EVENT_COUNT.
size_t tp_event_count()
{
  const char * v = std::getenv("RMW_SOMEIP_TP_EVENT_COUNT");
  if (v != nullptr && *v != '\0') {
    return std::strtoul(v, nullptr, 0);
  }
  return 256;
}

/// Maximum segment length: the MTU less the IPv4, UDP and SOME/IP headers,
/// and a multiple of 16 as AUTOSAR SOME/IP-TP requires.
constexpr uint16_t kMaxSegmentLength = 1392;

/// First topic event ID; must match kFirstTopicEventId in context_impl.hpp.
constexpr uint16_t kFirstTopicEventIdForTp = 0x8001;

/// vsomeip log level, defaulted to warning so it does not bury the ROS log.
std::string log_level()
{
  const char * v = std::getenv("RMW_SOMEIP_LOG_LEVEL");
  return (v != nullptr && *v != '\0') ? v : "warning";
}

}  // namespace

std::string ConfigGenerator::detect_unicast_address(bool localhost_only, std::string & error)
{
  if (const char * forced = std::getenv("RMW_SOMEIP_UNICAST_ADDRESS");
    forced != nullptr && *forced != '\0')
  {
    return forced;
  }
  if (localhost_only) {
    return "127.0.0.1";
  }

  // Take the first non-loopback IPv4 address. With several NICs, the user
  // pins one through RMW_SOMEIP_UNICAST_ADDRESS.
  ifaddrs * ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) {
    error = "getifaddrs() failed while detecting the local IPv4 address";
    return {};
  }

  std::string result;
  for (ifaddrs * ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    if ((ifa->ifa_flags & IFF_LOOPBACK) != 0 || (ifa->ifa_flags & IFF_UP) == 0) {
      continue;
    }
    char buf[INET_ADDRSTRLEN]{};
    auto * sin = reinterpret_cast<sockaddr_in *>(ifa->ifa_addr);
    if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) != nullptr) {
      result = buf;
      break;
    }
  }
  freeifaddrs(ifaddr);

  if (result.empty()) {
    error =
      "no non-loopback IPv4 interface found; set RMW_SOMEIP_UNICAST_ADDRESS "
      "or enable ROS_LOCALHOST_ONLY";
  }
  return result;
}

ConfigGenerator::Config ConfigGenerator::generate(
  const ParticipantIdentity & id,
  bool localhost_only,
  const ExternMapping & extern_mapping,
  std::string & error)
{
  if (const char * user_config = std::getenv("RMW_SOMEIP_CONFIG_FILE");
    user_config != nullptr && *user_config != '\0')
  {
    // Check it is readable here. vsomeip silently ignores a configuration
    // file it cannot read and starts with defaults, so a mistyped path
    // produces no diagnostic and the process runs with settings nobody chose.
    if (std::ifstream(user_config).good()) {
      // The user's file; not owned here, so never deleted.
      return Config{user_config, false};
    }
    error =
      std::string("RMW_SOMEIP_CONFIG_FILE points at a file that cannot be read: ") + user_config;
    return {};
  }

  const std::string unicast = detect_unicast_address(localhost_only, error);
  if (unicast.empty()) {
    return {};
  }

  // Where the routing manager is.
  //
  // vsomeip allows exactly one routing manager per host. If every process
  // declares itself the routing host, every one after the first fails init with
  //   "configured as routing but other routing manager present"
  // and two ROS processes on one host cannot communicate.
  //
  // Omitting "routing" makes vsomeip treat the first application to start as
  // the routing manager and the rest as proxies, so it is left out by default.
  // A setup with a dedicated routing daemon names it in RMW_SOMEIP_ROUTING_HOST.
  const char * routing_env = std::getenv("RMW_SOMEIP_ROUTING_HOST");
  const std::string routing_host =
    (routing_env != nullptr && *routing_env != '\0') ? routing_env : std::string();

  std::ostringstream os;
  os << "{\n"
     << "  \"_generated_by\": \"rmw_someip_cpp\",\n"
     << "  \"unicast\": \"" << unicast << "\",\n"
     << "  \"logging\": {\n"
     << "    \"level\": \"" << log_level() << "\",\n"
     << "    \"console\": \"true\",\n"
     << "    \"file\": { \"enable\": \"false\" },\n"
     << "    \"dlt\": \"false\"\n"
     << "  },\n"
     << "  \"applications\": [\n"
     << "    { \"name\": \"" << id.application_name << "\", \"id\": \""
     << hex16(id.instance_id) << "\" }\n"
     << "  ],\n"
     // This participant's service. Without a port declared here,
     // offer_service() reports success but no SD packet goes out.
     << "  \"services\": [\n"
     << "    {\n"
     << "      \"service\": \"" << hex16(id.service_id) << "\",\n"
     << "      \"instance\": \"" << hex16(id.instance_id) << "\",\n"
     << "      \"unreliable\": \"" << id.unreliable_port << "\",\n"
     << "      \"reliable\": { \"port\": \"" << id.reliable_port
     << "\", \"enable-magic-cookies\": \"false\" },\n"
     // SOME/IP-TP, for topics larger than UDP's maximum message length.
     << "      \"someip-tp\": {\n"
     << "        \"service-to-client\": [\n";

  const size_t tp_count = tp_event_count();
  for (size_t i = 0; i < tp_count; ++i) {
    const auto event_id = static_cast<uint16_t>(kFirstTopicEventIdForTp + i);
    os << "          { \"method\": \"" << hex16(event_id)
       << "\", \"max-segment-length\": \"" << kMaxSegmentLength
       << "\", \"separation-time\": \"0\" }"
       << ((i + 1 < tp_count) ? ",\n" : "\n");
  }

  os << "        ]\n"
     << "      }\n"
     << "    }";

  // Services the ROS side offers in extern mode. These are not the
  // participant's service, so each needs its own declaration with a port.
  for (const auto & topic : extern_mapping.offered_topics()) {
    os << ",\n    {\n"
       << "      \"service\": \"" << hex16(topic.service_id) << "\",\n"
       << "      \"instance\": \"" << hex16(topic.instance_id) << "\",\n";
    if (topic.reliable) {
      os << "      \"reliable\": { \"port\": \"" << topic.port
         << "\", \"enable-magic-cookies\": \"false\" }\n";
    } else {
      os << "      \"unreliable\": \"" << topic.port << "\"\n";
    }
    os << "    }";
  }

  os << "\n  ],\n";
  if (!routing_host.empty()) {
    os << "  \"routing\": \"" << routing_host << "\",\n";
  }
  os
     << "  \"service-discovery\": {\n"
     << "    \"enable\": \"true\",\n"
     << "    \"multicast\": \"224.244.224.245\",\n"
     << "    \"port\": \"" << id.sd_port << "\",\n"
     << "    \"protocol\": \"udp\",\n"
     << "    \"initial_delay_min\": \"10\",\n"
     << "    \"initial_delay_max\": \"100\",\n"
     << "    \"repetitions_base_delay\": \"200\",\n"
     << "    \"repetitions_max\": \"3\",\n"
     << "    \"ttl\": \"3\",\n"
     << "    \"cyclic_offer_delay\": \"2000\",\n"
     << "    \"request_response_delay\": \"1500\"\n"
     << "  }\n"
     << "}\n";

  const std::string path =
    "/tmp/rmw_someip_" + std::to_string(getpid()) + ".json";
  std::ofstream ofs(path, std::ios::trunc);
  if (!ofs) {
    error = "failed to write the generated vsomeip configuration to " + path;
    return {};
  }
  ofs << os.str();
  ofs.close();
  return Config{path, true};
}

void ConfigGenerator::cleanup(const Config & config)
{
  // Only delete what was generated here; never guess from the path.
  if (config.generated && !config.path.empty()) {
    std::remove(config.path.c_str());
  }
}

}  // namespace rmw_someip_cpp
