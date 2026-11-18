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

#ifndef DETAIL__GUARD_CONDITION_HPP_
#define DETAIL__GUARD_CONDITION_HPP_

#include <mutex>

#include "detail/wait_set.hpp"

namespace rmw_someip_cpp
{

/// Backing object for rmw_guard_condition_t.
///
/// Used by the executor to interrupt a wait, and to signal graph changes
/// (rmw_node_get_graph_guard_condition).
class GuardCondition final : public Waitable
{
public:
  /// Safe to call from any thread.
  void trigger()
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    triggered_ = true;
    notify_locked();
  }

  /// Read and clear the trigger.
  ///
  /// rmw_wait() keeps only the guard conditions that were triggered, so the
  /// flag has to be consumed at the same time it is tested.
  bool take()
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    const bool was_triggered = triggered_;
    triggered_ = false;
    return was_triggered;
  }

protected:
  bool has_data_locked() const override {return triggered_;}

private:
  bool triggered_ {false};
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__GUARD_CONDITION_HPP_
