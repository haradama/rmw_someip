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

#ifndef DETAIL__NODE_DATA_HPP_
#define DETAIL__NODE_DATA_HPP_

#include <string>

#include "rmw/types.h"

#include "detail/guard_condition.hpp"

namespace rmw_someip_cpp
{

/// Backing object for rmw_node_t::data.
///
/// SOME/IP has no notion of a node. A node is only a logical unit announced on
/// the graph channel; communication happens per context (per participant).
struct NodeData
{
  std::string name;
  std::string name_space;
  std::string enclave;

  /// Wakes the executor when the ROS graph changes.
  /// Backs the handle returned by rmw_node_get_graph_guard_condition().
  GuardCondition graph_guard_condition;
  rmw_guard_condition_t graph_guard_condition_handle;
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__NODE_DATA_HPP_
