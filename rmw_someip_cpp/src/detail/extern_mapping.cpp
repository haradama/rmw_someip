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

#include "detail/extern_mapping.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <string>
#include <vector>

namespace rmw_someip_cpp
{

namespace
{

/// Accept both "0x1234" and "4660". Hex reads better in the configuration
/// file, but the value may also be written as a plain YAML number.
uint16_t parse_id(const YAML::Node & node, const char * key, bool & ok)
{
  if (!node[key]) {
    ok = false;
    return 0;
  }
  const std::string text = node[key].as<std::string>();
  ok = true;
  return static_cast<uint16_t>(std::strtoul(text.c_str(), nullptr, 0));
}

}  // namespace

bool ExternMapping::load_from_env(ExternMapping & out, std::string & error)
{
  const char * path = std::getenv("RMW_SOMEIP_MAPPING_FILE");
  if (path == nullptr || *path == '\0') {
    return true;  // extern mode unused; an empty mapping is success
  }
  return load(path, out, error);
}

bool ExternMapping::load(const std::string & path, ExternMapping & out, std::string & error)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    error = "failed to parse '" + path + "': " + e.what();
    return false;
  }

  const YAML::Node externs = root["externs"];
  if (!externs) {
    error = "'" + path + "' has no 'externs' section";
    return false;
  }

  for (auto it = externs.begin(); it != externs.end(); ++it) {
    const std::string topic_name = it->first.as<std::string>();
    const YAML::Node entry = it->second;

    ExternTopic topic;
    topic.topic_name = topic_name;

    bool ok = true;
    topic.service_id = parse_id(entry, "service_id", ok);
    if (!ok) {
      error = "extern topic '" + topic_name + "' has no service_id";
      return false;
    }
    topic.instance_id = parse_id(entry, "instance_id", ok);
    if (!ok) {
      error = "extern topic '" + topic_name + "' has no instance_id";
      return false;
    }
    topic.event_id = parse_id(entry, "event_id", ok);
    if (!ok) {
      error = "extern topic '" + topic_name + "' has no event_id";
      return false;
    }
    topic.eventgroup_id = parse_id(entry, "eventgroup_id", ok);
    if (!ok) {
      error = "extern topic '" + topic_name + "' has no eventgroup_id";
      return false;
    }

    if (entry["type"]) {
      topic.type_name = entry["type"].as<std::string>();
    }

    const std::string direction =
      entry["direction"] ? entry["direction"].as<std::string>() : "subscribe";
    if (direction == "publish") {
      topic.ros_is_publisher = true;
    } else if (direction == "subscribe") {
      topic.ros_is_publisher = false;
    } else {
      error = "extern topic '" + topic_name +
        "' has an invalid direction '" + direction + "' (expected publish or subscribe)";
      return false;
    }

    const std::string serialization =
      entry["serialization"] ? entry["serialization"].as<std::string>() : "someip_native";
    if (serialization == "someip_native") {
      topic.serialization = Serialization::SomeipNative;
    } else if (serialization == "cdr") {
      topic.serialization = Serialization::Cdr;
    } else {
      error = "extern topic '" + topic_name +
        "' has an invalid serialization '" + serialization + "'";
      return false;
    }

    if (entry["reliable"]) {
      topic.reliable = entry["reliable"].as<bool>();
    }
    if (entry["port"]) {
      bool port_ok = true;
      topic.port = parse_id(entry, "port", port_ok);
    }
    if (topic.ros_is_publisher && topic.port == 0) {
      error = "extern topic '" + topic_name +
        "' has direction: publish but no port. A service that is offered must declare a "
        "port, otherwise vsomeip accepts offer_service() but never sends the SD offer.";
      return false;
    }

    out.topics_[topic_name] = topic;
  }
  return true;
}

const ExternTopic * ExternMapping::find(const std::string & topic_name) const
{
  auto it = topics_.find(topic_name);
  return (it == topics_.end()) ? nullptr : &it->second;
}

std::vector<ExternTopic> ExternMapping::offered_topics() const
{
  std::vector<ExternTopic> result;
  for (const auto & [name, topic] : topics_) {
    (void)name;
    if (topic.ros_is_publisher) {
      result.push_back(topic);
    }
  }
  return result;
}

}  // namespace rmw_someip_cpp
