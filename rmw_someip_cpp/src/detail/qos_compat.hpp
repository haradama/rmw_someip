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

#ifndef DETAIL__QOS_COMPAT_HPP_
#define DETAIL__QOS_COMPAT_HPP_

#include "rmw/qos_policy_kind.h"
#include "rmw/types.h"

namespace rmw_someip_cpp
{

/// Outcome of a QoS compatibility check.
struct QosIncompatibility
{
  /// The offending policy, or RMW_QOS_POLICY_INVALID when compatible.
  rmw_qos_policy_kind_t policy_kind {RMW_QOS_POLICY_INVALID};
  /// Text for rmw_qos_profile_check_compatible()'s reason. Null when compatible.
  const char * reason {nullptr};

  bool incompatible() const {return reason != nullptr;}
};

/// Should these two profiles be kept from matching?
///
/// Unlike DDS, SOME/IP does not refuse to connect on mismatched QoS, so this
/// is entirely the RMW's job. Without it a BEST_EFFORT publisher and a
/// RELIABLE subscription would silently connect (docs/design.md §7).
///
/// Three callers share this rule. Letting them drift would produce
/// contradictions such as rmw_qos_profile_check_compatible() reporting ERROR
/// while count_matched_subscriptions() still counts the pair:
///   - rmw_qos_profile_check_compatible() (public API)
///   - GraphCache::count_matched_* (match counts)
///   - the *_QOS_INCOMPATIBLE events (which report the offending policy)
inline QosIncompatibility check_incompatible(
  const rmw_qos_profile_t & publisher_profile,
  const rmw_qos_profile_t & subscription_profile)
{
  if (subscription_profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
    publisher_profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
  {
    return {RMW_QOS_POLICY_RELIABILITY,
      "ERROR: best effort publisher and reliable subscription;"};
  }
  if (subscription_profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL &&
    publisher_profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE)
  {
    return {RMW_QOS_POLICY_DURABILITY,
      "ERROR: volatile publisher and transient local subscription;"};
  }
  if (subscription_profile.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC &&
    publisher_profile.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
  {
    return {RMW_QOS_POLICY_LIVELINESS,
      "ERROR: automatic liveliness publisher and manual by topic subscription;"};
  }
  return {};
}

/// Are the effective values unresolved, so compatibility cannot be decided?
///
/// Liveliness has to be included. Leaving it out makes two default
/// profiles (liveliness SYSTEM_DEFAULT) compare as OK, which disagrees with
/// other implementations that return WARNING.
///
/// \return the reason to warn about, or nullptr
inline const char * check_warning(
  const rmw_qos_profile_t & publisher_profile,
  const rmw_qos_profile_t & subscription_profile)
{
  if (publisher_profile.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
    publisher_profile.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN ||
    subscription_profile.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
    subscription_profile.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN)
  {
    return "WARNING: Publisher or subscription reliability is system_default or unknown;";
  }
  if (publisher_profile.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
    publisher_profile.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN ||
    subscription_profile.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
    subscription_profile.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN)
  {
    return "WARNING: Publisher or subscription durability is system_default or unknown;";
  }
  if (publisher_profile.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
    publisher_profile.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN ||
    subscription_profile.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
    subscription_profile.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN)
  {
    return "WARNING: Publisher liveliness is system_default and subscription "
           "liveliness is system_default;";
  }
  return nullptr;
}

}  // namespace rmw_someip_cpp

#endif  // DETAIL__QOS_COMPAT_HPP_
