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

#ifndef DETAIL__WAIT_SET_HPP_
#define DETAIL__WAIT_SET_HPP_

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace rmw_someip_cpp
{

/// What rmw_wait() blocks on.
///
/// vsomeip invokes its callbacks on its own threads, so arrival has to be
/// signalled across threads.
///
/// Lock order: entity mutex, then WaitSetImpl mutex. Never the reverse.
/// To keep that invariant, this class must not query an entity while holding
/// its own mutex, so the wait predicate looks only at `triggered_`. Entities
/// call trigger() both when data arrives and when they are attached while
/// already holding data, which covers both a missed wakeup and a lost sample.
class WaitSetImpl
{
public:
  /// Wake the waiting thread. Safe to call while holding an entity's mutex.
  void trigger()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      triggered_ = true;
    }
    condition_.notify_one();
  }

  /// Wait until triggered or timed out.
  /// \param[in] timeout a negative duration waits forever
  void wait(std::chrono::nanoseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (timeout < std::chrono::nanoseconds::zero()) {
      condition_.wait(lock, [this]() {return triggered_;});
    } else {
      condition_.wait_for(lock, timeout, [this]() {return triggered_;});
    }
    triggered_ = false;
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool triggered_ {false};
};

/// Common part of everything that can be attached to a wait set.
class Waitable
{
public:
  virtual ~Waitable() = default;

  /// Attach to a wait set, triggering right away if data is already held.
  void attach(WaitSetImpl * wait_set)
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    wait_set_ = wait_set;
    if (wait_set != nullptr && has_data_locked()) {
      wait_set->trigger();
    }
  }

  void detach()
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    wait_set_ = nullptr;
  }

  /// Is there anything to take? Used to prune rmw_wait()'s result arrays.
  bool has_data() const
  {
    std::lock_guard<std::mutex> lock(waitable_mutex_);
    return has_data_locked();
  }

protected:
  /// Implemented by the derived class; called with waitable_mutex_ held.
  virtual bool has_data_locked() const = 0;

  /// Tell the wait set that data arrived. Call with waitable_mutex_ held.
  void notify_locked()
  {
    if (wait_set_ != nullptr) {
      wait_set_->trigger();
    }
  }

  mutable std::mutex waitable_mutex_;

private:
  WaitSetImpl * wait_set_ {nullptr};
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__WAIT_SET_HPP_
