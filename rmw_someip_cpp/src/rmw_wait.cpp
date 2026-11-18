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
// Wait sets and guard conditions.
//
// vsomeip invokes its callbacks on its own threads, so waiting is done with a
// condition variable. The lock order is documented in detail/wait_set.hpp.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <new>
#include <vector>

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#include "detail/entity_data.hpp"
#include "detail/event_data.hpp"
#include "detail/guard_condition.hpp"
#include "detail/wait_set.hpp"
#include "rmw_someip_cpp/identifier.hpp"

namespace
{

/// The arrays rmw_wait() receives hold each handle's `data`, the
/// implementation's own type. Casting void* straight to the Waitable base
/// would depend on layout, so go through the concrete type first.
///
/// The slot is kept as well, because entries that are not ready have to be
/// set to nullptr on return.
template<typename T>
struct Attached
{
  T * entity;
  void ** slot;
};

template<typename T>
std::vector<Attached<T>> collect(void ** entries, size_t count)
{
  std::vector<Attached<T>> out;
  if (entries == nullptr) {
    return out;
  }
  out.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    if (entries[i] != nullptr) {
      out.push_back({static_cast<T *>(entries[i]), &entries[i]});
    }
  }
  return out;
}

}  // namespace

extern "C"
{
rmw_guard_condition_t *
rmw_create_guard_condition(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return nullptr);

  auto * guard_condition = new (std::nothrow) rmw_guard_condition_t();
  if (guard_condition == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the guard condition handle");
    return nullptr;
  }
  auto * impl = new (std::nothrow) rmw_someip_cpp::GuardCondition();
  if (impl == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the guard condition");
    delete guard_condition;
    return nullptr;
  }

  guard_condition->implementation_identifier = rmw_someip_cpp::kIdentifier;
  guard_condition->data = impl;
  guard_condition->context = context;
  return guard_condition;
}

rmw_ret_t
rmw_destroy_guard_condition(rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard_condition,
    guard_condition->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  delete static_cast<rmw_someip_cpp::GuardCondition *>(guard_condition->data);
  delete guard_condition;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_trigger_guard_condition(const rmw_guard_condition_t * guard_condition)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(guard_condition, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    guard_condition,
    guard_condition->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * impl = static_cast<rmw_someip_cpp::GuardCondition *>(guard_condition->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(impl, RMW_RET_INVALID_ARGUMENT);
  impl->trigger();
  return RMW_RET_OK;
}

rmw_wait_set_t *
rmw_create_wait_set(rmw_context_t * context, size_t max_conditions)
{
  // max_conditions is only a preallocation hint; storage is dynamic here.
  (void)max_conditions;
  RMW_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return nullptr);

  rmw_wait_set_t * wait_set = rmw_wait_set_allocate();
  if (wait_set == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the wait set handle");
    return nullptr;
  }
  auto * impl = new (std::nothrow) rmw_someip_cpp::WaitSetImpl();
  if (impl == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the wait set");
    rmw_wait_set_free(wait_set);
    return nullptr;
  }

  wait_set->implementation_identifier = rmw_someip_cpp::kIdentifier;
  wait_set->data = impl;
  return wait_set;
}

rmw_ret_t
rmw_destroy_wait_set(rmw_wait_set_t * wait_set)
{
  // rmw requires ERROR rather than INVALID_ARGUMENT here.
  if (wait_set == nullptr) {
    RMW_SET_ERROR_MSG("wait_set argument is null");
    return RMW_RET_ERROR;
  }
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set,
    wait_set->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  delete static_cast<rmw_someip_cpp::WaitSetImpl *>(wait_set->data);
  rmw_wait_set_free(wait_set);
  return RMW_RET_OK;
}

rmw_ret_t
rmw_wait(
  rmw_subscriptions_t * subscriptions,
  rmw_guard_conditions_t * guard_conditions,
  rmw_services_t * services,
  rmw_clients_t * clients,
  rmw_events_t * events,
  rmw_wait_set_t * wait_set,
  const rmw_time_t * wait_timeout)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    wait_set,
    wait_set->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto * impl = static_cast<rmw_someip_cpp::WaitSetImpl *>(wait_set->data);
  RMW_CHECK_ARGUMENT_FOR_NULL(impl, RMW_RET_INVALID_ARGUMENT);


  auto guards = (guard_conditions != nullptr) ?
    collect<rmw_someip_cpp::GuardCondition>(
    guard_conditions->guard_conditions, guard_conditions->guard_condition_count) :
    std::vector<Attached<rmw_someip_cpp::GuardCondition>>{};

  auto subs = (subscriptions != nullptr) ?
    collect<rmw_someip_cpp::SubscriptionData>(
    subscriptions->subscribers, subscriptions->subscriber_count) :
    std::vector<Attached<rmw_someip_cpp::SubscriptionData>>{};

  auto srvs = (services != nullptr) ?
    collect<rmw_someip_cpp::ServiceData>(services->services, services->service_count) :
    std::vector<Attached<rmw_someip_cpp::ServiceData>>{};

  auto clis = (clients != nullptr) ?
    collect<rmw_someip_cpp::ClientData>(clients->clients, clients->client_count) :
    std::vector<Attached<rmw_someip_cpp::ClientData>>{};

  // Events are passed differently. Subscriptions, services and clients arrive
  // as each handle's `data`, but an event arrives as the rmw_event_t handle
  // itself; casting that straight to EventData* yields garbage.
  std::vector<Attached<rmw_someip_cpp::EventData>> evts;
  if (events != nullptr && events->events != nullptr) {
    evts.reserve(events->event_count);
    for (size_t i = 0; i < events->event_count; ++i) {
      auto * handle = static_cast<rmw_event_t *>(events->events[i]);
      if (handle == nullptr || handle->data == nullptr) {
        continue;
      }
      evts.push_back(
        {static_cast<rmw_someip_cpp::EventData *>(handle->data), &events->events[i]});
    }
  }

  // Attach without holding the wait set's lock, per the order in
  // detail/wait_set.hpp. An entity that already holds data triggers during
  // attach, so nothing is missed.
  for (const auto & entry : guards) {
    entry.entity->attach(impl);
  }
  for (const auto & entry : subs) {
    entry.entity->attach(impl);
  }
  for (const auto & entry : srvs) {
    entry.entity->attach(impl);
  }
  for (const auto & entry : clis) {
    entry.entity->attach(impl);
  }
  for (const auto & entry : evts) {
    entry.entity->attach(impl);
  }

  // Being woken does not mean anything is ready. WaitSetImpl::triggered_ only
  // says somebody woke the set, and it can outlive its reason: a message that
  // arrives between wait() and detach() sets it again, is reported ready by the
  // scan below and taken, and the next rmw_wait() then wakes at once with
  // nothing to report. Returning RMW_RET_TIMEOUT there would make an infinite
  // wait time out, which rclcpp treats as fatal, so wait again on the time
  // that remains (docs/design.md §19).
  const bool wait_forever = (wait_timeout == nullptr);
  const auto deadline = std::chrono::steady_clock::now() +
    (wait_forever ?
    std::chrono::nanoseconds::zero() :
    std::chrono::nanoseconds(
      static_cast<int64_t>(wait_timeout->sec) * 1000000000LL +
      static_cast<int64_t>(wait_timeout->nsec)));

  // A guard condition's take() consumes the trigger, so the result is
  // remembered. Testing with has_data() instead would leave the trigger set
  // and spin the executor. Subscriptions stay queued until rmw_take(), so
  // has_data() is fine for them.
  std::vector<uint8_t> guards_ready(guards.size(), 0);
  std::vector<uint8_t> subs_ready(subs.size(), 0);
  std::vector<uint8_t> srvs_ready(srvs.size(), 0);
  std::vector<uint8_t> clis_ready(clis.size(), 0);
  std::vector<uint8_t> evts_ready(evts.size(), 0);

  bool any_ready = false;
  while (true) {
    if (wait_forever) {
      impl->wait(std::chrono::nanoseconds(-1));
    } else {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      impl->wait(
        (remaining > std::chrono::nanoseconds::zero()) ?
        remaining : std::chrono::nanoseconds::zero());
    }

    for (size_t i = 0; i < guards.size(); ++i) {
      if (guards[i].entity->take()) {
        guards_ready[i] = 1;
        any_ready = true;
      }
    }
    const auto scan = [&any_ready](const auto & entries, std::vector<uint8_t> & ready) {
        for (size_t i = 0; i < entries.size(); ++i) {
          if (entries[i].entity->has_data()) {
            ready[i] = 1;
            any_ready = true;
          }
        }
      };
    scan(subs, subs_ready);
    scan(srvs, srvs_ready);
    scan(clis, clis_ready);
    scan(evts, evts_ready);

    if (any_ready) {
      break;
    }
    // Only an infinite wait retries until something is ready.
    if (!wait_forever && std::chrono::steady_clock::now() >= deadline) {
      break;
    }
  }

  for (const auto & entry : guards) {
    entry.entity->detach();
  }
  for (const auto & entry : subs) {
    entry.entity->detach();
  }
  for (const auto & entry : srvs) {
    entry.entity->detach();
  }
  for (const auto & entry : clis) {
    entry.entity->detach();
  }
  for (const auto & entry : evts) {
    entry.entity->detach();
  }

  // rmw_wait() requires entries that are not ready to be set to nullptr.
  const auto clear_not_ready = [](const auto & entries, const std::vector<uint8_t> & ready) {
      for (size_t i = 0; i < entries.size(); ++i) {
        if (!ready[i]) {
          *entries[i].slot = nullptr;
        }
      }
    };
  clear_not_ready(guards, guards_ready);
  clear_not_ready(subs, subs_ready);
  clear_not_ready(srvs, srvs_ready);
  clear_not_ready(clis, clis_ready);
  clear_not_ready(evts, evts_ready);

  return any_ready ? RMW_RET_OK : RMW_RET_TIMEOUT;
}
}  // extern "C"
