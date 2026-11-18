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

#ifndef DETAIL__SOMEIP_ENDPOINT_HPP_
#define DETAIL__SOMEIP_ENDPOINT_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include <vsomeip/vsomeip.hpp>

namespace rmw_someip_cpp
{

/// Everything the RMW asks of SOME/IP.
///
/// The seam between RMW logic and vsomeip.
///
/// What `rmw_context_impl_s` does — interpreting the graph, matching
/// subscriptions, maintaining routing tables, deriving QoS events — needs no
/// actual SOME/IP traffic. Calling `vsomeip::application` directly would still
/// force tests to run a routing manager and discovery, and would leave events
/// like "a peer appeared" or "a subscription arrived" impossible to provoke on
/// demand. Behind this interface a fake can be substituted.
///
/// Arguments are plain values rather than vsomeip types: event_type_e and
/// reliability_type_e become bools, and payloads become byte spans, which
/// removes a ternary and a `create_payload()` boilerplate from every call site.
///
/// vsomeip::message is the one exception. A service response has to inherit the
/// request ID from the original message (create_response), which is tied to
/// vsomeip's representation. Building a message needs no routing manager, so
/// tests with a fake can still construct one.
class SomeipEndpoint
{
public:
  /// Message arrival, called on a vsomeip thread.
  using MessageHandler = std::function<void (const std::shared_ptr<vsomeip::message> &)>;
  /// A participant appeared or went away.
  using AvailabilityHandler = std::function<void (uint16_t instance_id, bool available)>;
  /// Someone subscribed.
  using SubscriptionHandler = std::function<void (uint16_t client)>;

  virtual ~SomeipEndpoint() = default;

  // -- Offering side --------------------------------------------------------

  virtual void offer_service(uint16_t service, uint16_t instance) = 0;

  /// \param[in] is_field ET_FIELD, which delivers the latest value on subscribe
  /// \param[in] reliable TCP when true, UDP when false
  virtual void offer_event(
    uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
    bool is_field, bool reliable) = 0;

  virtual void stop_offer_event(uint16_t service, uint16_t instance, uint16_t event) = 0;

  virtual void notify(
    uint16_t service, uint16_t instance, uint16_t event,
    const uint8_t * data, size_t size) = 0;

  /// Send to one subscriber only, for TRANSIENT_LOCAL history replay.
  virtual void notify_one(
    uint16_t service, uint16_t instance, uint16_t event,
    const uint8_t * data, size_t size, uint16_t client) = 0;

  // -- Consuming side -------------------------------------------------------

  virtual void request_service(uint16_t service, uint16_t instance) = 0;

  virtual void request_event(
    uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
    bool is_field, bool reliable) = 0;

  virtual void subscribe(uint16_t service, uint16_t instance, uint16_t eventgroup) = 0;
  virtual void unsubscribe(uint16_t service, uint16_t instance, uint16_t eventgroup) = 0;
  virtual void release_event(uint16_t service, uint16_t instance, uint16_t event) = 0;

  // -- service / client -----------------------------------------------------

  virtual void send_request(
    uint16_t service, uint16_t instance, uint16_t method,
    const uint8_t * data, size_t size) = 0;

  /// Reply, inheriting the request ID from the original message.
  virtual void send_response(
    const std::shared_ptr<vsomeip::message> & request,
    const uint8_t * data, size_t size) = 0;

  // -- Handlers -------------------------------------------------------------

  /// One handler for every service, instance and method.
  virtual void on_message(MessageHandler handler) = 0;

  /// Availability of every instance of one service.
  virtual void on_availability(uint16_t service, AvailabilityHandler handler) = 0;

  virtual void on_subscription(
    uint16_t service, uint16_t instance, uint16_t eventgroup,
    SubscriptionHandler handler) = 0;

  virtual void clear_subscription_handler(
    uint16_t service, uint16_t instance, uint16_t eventgroup) = 0;
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__SOMEIP_ENDPOINT_HPP_
