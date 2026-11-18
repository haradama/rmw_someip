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

#ifndef DETAIL__CONFIG_GENERATOR_HPP_
#define DETAIL__CONFIG_GENERATOR_HPP_

#include <string>

#include "detail/extern_mapping.hpp"
#include "detail/participant.hpp"

namespace rmw_someip_cpp
{

/// Generate vsomeip's JSON configuration at runtime.
///
/// vsomeip expects a static JSON configuration, but measurement
/// (docs/design.md §3) showed that only the participant's own service,
/// instance and ports have to be declared there — events and eventgroups can
/// be assigned at runtime. All of those are known by the time rmw_init() runs.
///
/// The generated file is handed to vsomeip through VSOMEIP_CONFIGURATION.
class ConfigGenerator
{
public:
  /// Where the configuration file is, and who created it.
  struct Config
  {
    /// Path handed to vsomeip; empty when generation failed.
    std::string path;

    /// Did this class create the file?
    ///
    /// Ownership is recorded here and never guessed from the path: a user may
    /// point RMW_SOMEIP_CONFIG_FILE at a file that looks like a generated one,
    /// and it must not be deleted at shutdown.
    bool generated {false};
  };

  /// Generate the configuration and write it to a temporary file.
  ///
  /// When RMW_SOMEIP_CONFIG_FILE is set, that path is returned as-is with
  /// `generated` false, which is the escape hatch for a hand-written config.
  ///
  /// \param[in] id this participant's identity
  /// \param[in] localhost_only the ROS_LOCALHOST_ONLY equivalent
  /// \param[in] extern_mapping extern-mode topics. Those the ROS side offers
  ///   need a service and port declared: without the port, offer_service()
  ///   reports success but no SD offer is ever sent (docs/design.md §3).
  /// \param[out] error message on failure
  static Config generate(
    const ParticipantIdentity & id,
    bool localhost_only,
    const ExternMapping & extern_mapping,
    std::string & error);

  /// Delete the file generate() created. A no-op when `generated` is false.
  static void cleanup(const Config & config);

private:
  /// Pick the address for vsomeip's "unicast". Discovery does not work unless
  /// it matches the container's or host's real IP.
  static std::string detect_unicast_address(bool localhost_only, std::string & error);
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__CONFIG_GENERATOR_HPP_
