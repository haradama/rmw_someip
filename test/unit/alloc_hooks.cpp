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
// Substitutions that let a test provoke an allocation failure.
//
// Symbols defined in the executable resolve ahead of a shared library's, so
// allocations inside librmw are caught without LD_PRELOAD.
//
// malloc itself is not replaced. That was tried: rosidl's
// get_message_typesupport_handle() throws from a noexcept context when
// allocation fails, which ends in std::terminate. Another library cannot be
// made OOM-tolerant from here.
//
// Instead only the two entry points rmw_someip_cpp treats as "null on failure"
// are replaced:
//
//   operator new(size_t, nothrow)  ->  `new (std::nothrow) PublisherData()`
//   rmw_allocate()                 ->  what rmw_publisher_allocate() calls
//
// Both are allowed to return null, so failing them throws nothing.

#include "alloc_hooks.hpp"

#include <cstddef>
#include <cstdlib>
#include <new>

namespace
{

// Constant-initialized PODs, so setting up the TLS block allocates nothing.
//
// Being thread-local is the point: vsomeip runs several dispatch threads that
// allocate constantly, and a process-wide count would make "the nth
// allocation" differ from run to run.
thread_local int64_t g_countdown = -1;
thread_local bool g_fired = false;

/// \return true to fail this allocation
inline bool take_failure()
{
  if (g_countdown < 0) {
    return false;
  }
  if (g_countdown == 0) {
    g_countdown = -1;   // disarm; failing on would take cleanup down too
    g_fired = true;
    return true;
  }
  --g_countdown;
  return false;
}

}  // namespace

namespace rmw_someip_cpp_test
{

void fail_allocation_at(int64_t n)
{
  g_fired = false;
  g_countdown = n;
}

void stop_failing_allocations()
{
  g_countdown = -1;
}

bool allocation_failure_fired()
{
  return g_fired;
}

}  // namespace rmw_someip_cpp_test

// -- The substitutions --------------------------------------------------------

void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  if (take_failure()) {
    return nullptr;
  }
  return std::malloc(size);
}

void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  if (take_failure()) {
    return nullptr;
  }
  return std::malloc(size);
}

// What the nothrow new allocates comes back to the nothrow delete. Without the
// matching definitions, memory from malloc would reach a different deallocator.
void operator delete(void * pointer, const std::nothrow_t &) noexcept
{
  std::free(pointer);
}

void operator delete[](void * pointer, const std::nothrow_t &) noexcept
{
  std::free(pointer);
}

extern "C" {

/// Replaces librmw's rmw_allocate(). rmw_publisher_allocate() and its siblings
/// all go through it, so one hook covers them.
void * rmw_allocate(size_t size)
{
  if (take_failure()) {
    return nullptr;
  }
  return std::malloc(size);
}

void rmw_free(void * pointer)
{
  std::free(pointer);
}

}  // extern "C"
