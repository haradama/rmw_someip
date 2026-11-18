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

#ifndef ALLOC_HOOKS_HPP_
#define ALLOC_HOOKS_HPP_

#include <cstdint>

namespace rmw_someip_cpp_test
{

/// Fail this thread's nth (0-based) allocation that is allowed to return null.
///
/// That means `operator new(nothrow)` and `rmw_allocate()`. Neither
/// `rmw_publisher_allocate()` nor `new (std::nothrow)` takes an allocator, so
/// substituting an rcutils_allocator_t cannot reach them.
///
/// malloc itself is not replaced; see the top of alloc_hooks.cpp.
///
/// Disarms itself after one failure. Failing indefinitely would take cleanup
/// and error-message construction down with it.
void fail_allocation_at(int64_t n);

/// Disarm, for the case where no failure occurred.
void stop_failing_allocations();

/// Did an allocation actually fail since fail_allocation_at()?
bool allocation_failure_fired();

}  // namespace rmw_someip_cpp_test

#endif  // ALLOC_HOOKS_HPP_
