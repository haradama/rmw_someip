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

#ifndef DETAIL__MESSAGE_POOL_HPP_
#define DETAIL__MESSAGE_POOL_HPP_

#include <cstdlib>
#include <mutex>
#include <set>
#include <vector>

#include "rmw/allocators.h"
#include "rosidl_runtime_c/message_initialization.h"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "detail/type_support.hpp"

namespace rmw_someip_cpp
{

/// Pool of ROS messages behind the loaned-message API.
///
/// This is not shared memory. vsomeip's local path is a UNIX domain socket,
/// so CDR still goes over the wire; what the pool saves is the application
/// allocating and destroying the message itself. The rmw
/// contract is that the caller borrows memory from the middleware, and says
/// nothing about how it is transported, so that contract is met.
///
/// Messages are created and destroyed through the introspection init_function
/// and fini_function, which is what lets an RMW without per-type code
/// generation instantiate an arbitrary type.
class MessagePool
{
public:
  ~MessagePool()
  {
    // Destroy outstanding loans too, so a caller that forgot to return one
    // does not leak when the publisher or subscription goes away.
    std::lock_guard<std::mutex> lock(mutex_);
    for (void * message : idle_) {
      destroy(message);
    }
    for (void * message : loaned_) {
      destroy(message);
    }
    idle_.clear();
    loaned_.clear();
  }

  void configure(const TypeSupportInfo & type_support)
  {
    is_c_ = type_support.is_c;
    members_ = type_support.members;
  }

  bool available() const {return members_ != nullptr;}

  /// Lend one message, initialized and empty.
  /// \return nullptr on failure
  void * borrow()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    void * message = nullptr;
    if (!idle_.empty()) {
      message = idle_.back();
      idle_.pop_back();
    } else {
      message = create();
      if (message == nullptr) {
        return nullptr;
      }
    }
    loaned_.insert(message);
    return message;
  }

  /// Take a loan back.
  /// \return false if this pool did not lend it
  bool give_back(void * message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (loaned_.erase(message) == 0) {
      return false;
    }
    // Reset for reuse. fini frees the inner buffers of strings and sequences,
    // so reuse only saves the outer allocation.
    if (idle_.size() < kMaxIdle) {
      fini(message);
      init(message);
      idle_.push_back(message);
    } else {
      destroy(message);
    }
    return true;
  }

private:
  /// Cap on idle messages, so a burst of loans is not held onto forever.
  static constexpr size_t kMaxIdle = 8;

  void * create() const
  {
    const size_t size = size_of();
    if (size == 0) {
      return nullptr;
    }
    // rmw_allocate / rmw_free: this is rmw-owned memory lent to the caller,
    // so it should go through rmw's own entry points. It also lets tests
    // substitute them to provoke an allocation failure.
    void * memory = rmw_allocate(size);
    if (memory == nullptr) {
      return nullptr;
    }
    init(memory);
    return memory;
  }

  void destroy(void * message) const
  {
    fini(message);
    rmw_free(message);
  }

  size_t size_of() const
  {
    if (members_ == nullptr) {
      return 0;
    }
    if (is_c_) {
      return static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
        members_)->size_of_;
    }
    return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      members_)->size_of_;
  }

  /// init_function's argument type differs between C and C++ (a plain enum vs
  /// a scoped enum class). Same ABI, different types, so branch rather than cast.
  void init(void * memory) const
  {
    if (is_c_) {
      const auto * members =
        static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(members_);
      members->init_function(memory, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
    } else {
      const auto * members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(members_);
      members->init_function(memory, rosidl_runtime_cpp::MessageInitialization::ALL);
    }
  }

  void fini(void * memory) const
  {
    if (is_c_) {
      const auto * members =
        static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(members_);
      members->fini_function(memory);
    } else {
      const auto * members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(members_);
      members->fini_function(memory);
    }
  }

  mutable std::mutex mutex_;
  bool is_c_ {false};
  const void * members_ {nullptr};
  std::vector<void *> idle_;
  std::set<void *> loaned_;
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__MESSAGE_POOL_HPP_
