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
// Context lifecycle: where the vsomeip application is created and destroyed.

#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <utility>

#include "rcutils/strdup.h"
#include "rcutils/types.h"

#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw/rmw.h"

#include "detail/config_generator.hpp"
#include "detail/context_impl.hpp"
#include "detail/extern_mapping.hpp"
#include "detail/node_data.hpp"
#include "detail/participant.hpp"
#include "detail/someip_session.hpp"
#include "rmw_someip_cpp/identifier.hpp"

extern "C"
{
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ALLOCATOR(&allocator, return RMW_RET_INVALID_ARGUMENT);
  if (init_options->implementation_identifier != nullptr) {
    RMW_SET_ERROR_MSG("expected zero-initialized init_options");
    return RMW_RET_INVALID_ARGUMENT;
  }

  init_options->instance_id = 0;
  init_options->implementation_identifier = rmw_someip_cpp::kIdentifier;
  init_options->allocator = allocator;
  init_options->impl = nullptr;
  init_options->enclave = nullptr;
  init_options->domain_id = RMW_DEFAULT_DOMAIN_ID;
  init_options->security_options = rmw_get_zero_initialized_security_options();
  init_options->localhost_only = RMW_LOCALHOST_ONLY_DEFAULT;
  init_options->discovery_options = rmw_get_zero_initialized_discovery_options();
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(src, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(dst, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(src->implementation_identifier, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    src,
    src->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (dst->implementation_identifier != nullptr) {
    RMW_SET_ERROR_MSG("expected zero-initialized dst");
    return RMW_RET_INVALID_ARGUMENT;
  }

  rmw_init_options_t tmp = *src;
  // rmw_discovery_options_copy() wants a non-const allocator, hence the copy.
  rcutils_allocator_t allocator = src->allocator;

  tmp.enclave = nullptr;
  if (src->enclave != nullptr) {
    tmp.enclave = rcutils_strdup(src->enclave, allocator);
    if (tmp.enclave == nullptr) {
      RMW_SET_ERROR_MSG("failed to copy the enclave");
      return RMW_RET_BAD_ALLOC;
    }
  }

  tmp.security_options = rmw_get_zero_initialized_security_options();
  rmw_ret_t ret =
    rmw_security_options_copy(&src->security_options, &allocator, &tmp.security_options);
  if (ret != RMW_RET_OK) {
    allocator.deallocate(tmp.enclave, allocator.state);
    return ret;
  }

  tmp.discovery_options = rmw_get_zero_initialized_discovery_options();
  ret = rmw_discovery_options_copy(&src->discovery_options, &allocator, &tmp.discovery_options);
  if (ret != RMW_RET_OK) {
    const rmw_ret_t fini_ret = rmw_security_options_fini(&tmp.security_options, &allocator);
    (void)fini_ret;
    allocator.deallocate(tmp.enclave, allocator.state);
    return ret;
  }

  *dst = tmp;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(init_options->implementation_identifier, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    init_options,
    init_options->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  rcutils_allocator_t * allocator = &init_options->allocator;
  RCUTILS_CHECK_ALLOCATOR(allocator, return RMW_RET_INVALID_ARGUMENT);

  allocator->deallocate(init_options->enclave, allocator->state);
  rmw_ret_t ret = rmw_security_options_fini(&init_options->security_options, allocator);
  if (ret != RMW_RET_OK) {
    return ret;
  }
  ret = rmw_discovery_options_fini(&init_options->discovery_options);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  *init_options = rmw_get_zero_initialized_init_options();
  return RMW_RET_OK;
}

rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(options, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(options->implementation_identifier, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    options,
    options->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // The enclave is required; a null one breaks the security setup downstream.
  RMW_CHECK_ARGUMENT_FOR_NULL(options->enclave, RMW_RET_INVALID_ARGUMENT);

  // Re-initializing an initialized context is invalid.
  if (context->impl != nullptr) {
    RMW_SET_ERROR_MSG("the given context has already been initialized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  const size_t domain_id =
    (options->domain_id == RMW_DEFAULT_DOMAIN_ID) ? 0u : options->domain_id;

  context->instance_id = options->instance_id;
  context->implementation_identifier = rmw_someip_cpp::kIdentifier;
  context->actual_domain_id = domain_id;
  context->options = rmw_get_zero_initialized_init_options();
  context->impl = nullptr;

  rmw_ret_t ret = rmw_init_options_copy(options, &context->options);
  if (ret != RMW_RET_OK) {
    return ret;
  }

  const auto identity = rmw_someip_cpp::ParticipantIdentity::create(domain_id);

  std::string error;
  // Read the extern mapping first: generating the vsomeip config needs it.
  rmw_someip_cpp::ExternMapping extern_mapping;
  if (!rmw_someip_cpp::ExternMapping::load_from_env(extern_mapping, error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to load RMW_SOMEIP_MAPPING_FILE: %s", error.c_str());
    const rmw_ret_t fini_ret = rmw_init_options_fini(&context->options);
    (void)fini_ret;
    return RMW_RET_ERROR;
  }

  const auto config = rmw_someip_cpp::ConfigGenerator::generate(
    identity,
    options->localhost_only == RMW_LOCALHOST_ONLY_ENABLED,
    extern_mapping,
    error);
  if (config.path.empty()) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to generate the vsomeip configuration: %s", error.c_str());
    const rmw_ret_t fini_ret = rmw_init_options_fini(&context->options);
    (void)fini_ret;
    return RMW_RET_ERROR;
  }

  auto session = rmw_someip_cpp::SomeipSession::create(identity, config, error);
  if (!session) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to start the vsomeip application: %s", error.c_str());
    rmw_someip_cpp::ConfigGenerator::cleanup(config);
    const rmw_ret_t fini_ret = rmw_init_options_fini(&context->options);
    (void)fini_ret;
    return RMW_RET_ERROR;
  }

  context->impl = new (std::nothrow) rmw_context_impl_s(
    domain_id, identity, std::move(session));
  if (context->impl != nullptr) {
    context->impl->extern_mapping = std::move(extern_mapping);
  }
  if (context->impl == nullptr) {
    RMW_SET_ERROR_MSG("failed to allocate the context implementation");
    const rmw_ret_t fini_ret = rmw_init_options_fini(&context->options);
    (void)fini_ret;
    return RMW_RET_BAD_ALLOC;
  }

  // Start the graph channel; other participants' graphs become visible.
  if (!context->impl->start_graph_channel(error)) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "failed to start the graph channel: %s", error.c_str());
    delete context->impl;
    context->impl = nullptr;
    const rmw_ret_t fini_ret = rmw_init_options_fini(&context->options);
    (void)fini_ret;
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // Do not touch vsomeip here; all teardown happens in rmw_context_fini().
  //
  // rcl keeps destroying entities after shutdown, and the application loop may
  // still publish, so the vsomeip application has to outlive this call, as a
  // DDS participant does. It also has to return quickly: rclpy's
  // Context.try_shutdown() races the rcl_shutdown in the signal handler, and a
  // slow call here widens that window.
  //
  // rmw requires a second shutdown to succeed.
  context->impl->is_shutdown = true;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(context, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(context->impl, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    context,
    context->implementation_identifier,
    rmw_someip_cpp::kIdentifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // Finalizing before shutdown is invalid.
  if (!context->impl->is_shutdown) {
    RMW_SET_ERROR_MSG("the context must be shut down before it is finalized");
    return RMW_RET_INVALID_ARGUMENT;
  }

  {
    std::lock_guard<std::mutex> lock(context->impl->entities_mutex);
    if (!context->impl->nodes.empty()) {
      RMW_SET_ERROR_MSG("cannot finalize the context while nodes still exist");
      return RMW_RET_ERROR;
    }
  }

  delete context->impl;
  context->impl = nullptr;

  const rmw_ret_t ret = rmw_init_options_fini(&context->options);
  *context = rmw_get_zero_initialized_context();
  return ret;
}

// -- Unsupported --------------------------------------------------------------
// The allocation API, through which a caller would specify an allocation
// strategy. Loans here are served entirely from a pool built on the
// introspection init/fini functions (detail/message_pool.hpp), leaving nothing
// for a caller-supplied allocation to do. Unsupported, as in most RMW
// implementations; rcl accepts UNSUPPORTED from these. Loaned messages
// themselves (rmw_borrow_loaned_message and friends) are supported, and
// can_loan_messages is true (docs/design.md §22).

rmw_ret_t
rmw_init_publisher_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_publisher_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_init_publisher_allocation() is not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}

rmw_ret_t
rmw_init_subscription_allocation(
  const rosidl_message_type_support_t * type_support,
  const rosidl_runtime_c__Sequence__bound * message_bounds,
  rmw_subscription_allocation_t * allocation)
{
  (void)type_support;
  (void)message_bounds;
  (void)allocation;
  RMW_SET_ERROR_MSG("rmw_init_subscription_allocation() is not supported by rmw_someip_cpp");
  return RMW_RET_UNSUPPORTED;
}
}  // extern "C"
