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

#ifndef DETAIL__EXTERN_MAPPING_HPP_
#define DETAIL__EXTERN_MAPPING_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rmw_someip_cpp
{

/// Payload encoding.
enum class Serialization
{
  Cdr,           ///< ROS to ROS (native mode)
  SomeipNative,  ///< AUTOSAR SOME/IP Transformer (extern mode)
};

/// One topic's extern-mode configuration.
///
/// A non-ROS peer does not join the graph channel, so IDs cannot be negotiated
/// at runtime; they are pinned in the configuration file instead.
struct ExternTopic
{
  std::string topic_name;
  std::string type_name;      ///< empty means the type name is not checked

  uint16_t service_id {0};
  uint16_t instance_id {0};
  uint16_t event_id {0};
  uint16_t eventgroup_id {0};

  /// True when the ROS side offers (ROS to peer), false when it subscribes.
  bool ros_is_publisher {false};

  /// TCP (reliable) or UDP (best effort).
  bool reliable {false};

  Serialization serialization {Serialization::SomeipNative};

  /// Port to offer on, when ros_is_publisher.
  uint16_t port {0};
};

/// Contents of someip_mapping.yaml.
class ExternMapping
{
public:
  /// Load the file named by RMW_SOMEIP_MAPPING_FILE.
  /// An unset variable yields an empty mapping, which is not an error.
  /// \return false if the file could not be read
  static bool load_from_env(ExternMapping & out, std::string & error);

  /// Load from an explicit path.
  static bool load(const std::string & path, ExternMapping & out, std::string & error);

  /// \return the entry for this topic, or nullptr
  const ExternTopic * find(const std::string & topic_name) const;

  bool empty() const {return topics_.empty();}

  /// Topics the ROS side offers, which need a service and port in the
  /// generated vsomeip JSON.
  ///
  /// There is no counterpart for subscribed topics: endpoints of external
  /// services are learned from SOME/IP-SD, so nothing has to be declared.
  std::vector<ExternTopic> offered_topics() const;

private:
  std::map<std::string, ExternTopic> topics_;
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__EXTERN_MAPPING_HPP_
