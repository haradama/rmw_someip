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

#ifndef DETAIL__SOMEIP_SESSION_HPP_
#define DETAIL__SOMEIP_SESSION_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <vsomeip/vsomeip.hpp>

#include "detail/config_generator.hpp"
#include "detail/participant.hpp"
#include "detail/someip_endpoint.hpp"

namespace rmw_someip_cpp
{

/// The production SomeipEndpoint: a vsomeip::application wrapped in RAII.
///
/// application::start() blocks, so it runs on its own thread. Calls to
/// offer_service() or request_service() made before registration with the
/// routing manager completes (ST_REGISTERED) are ignored, so create() waits
/// for it.
class SomeipSession : public SomeipEndpoint
{
public:
  /// Create the vsomeip application and wait for routing manager registration.
  /// \return nullptr on failure, with a message in `error`
  static std::unique_ptr<SomeipSession> create(
    const ParticipantIdentity & id,
    const ConfigGenerator::Config & config,
    std::string & error);

  ~SomeipSession() override;

  SomeipSession(const SomeipSession &) = delete;
  SomeipSession & operator=(const SomeipSession &) = delete;

  const ParticipantIdentity & identity() const {return identity_;}

  // -- SomeipEndpoint ------------------------------------------------------
  void offer_service(uint16_t service, uint16_t instance) override;
  void offer_event(
    uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
    bool is_field, bool reliable) override;
  void stop_offer_event(uint16_t service, uint16_t instance, uint16_t event) override;
  void notify(
    uint16_t service, uint16_t instance, uint16_t event,
    const uint8_t * data, size_t size) override;
  void notify_one(
    uint16_t service, uint16_t instance, uint16_t event,
    const uint8_t * data, size_t size, uint16_t client) override;

  void request_service(uint16_t service, uint16_t instance) override;
  void request_event(
    uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
    bool is_field, bool reliable) override;
  void subscribe(uint16_t service, uint16_t instance, uint16_t eventgroup) override;
  void unsubscribe(uint16_t service, uint16_t instance, uint16_t eventgroup) override;
  void release_event(uint16_t service, uint16_t instance, uint16_t event) override;

  void send_request(
    uint16_t service, uint16_t instance, uint16_t method,
    const uint8_t * data, size_t size) override;
  void send_response(
    const std::shared_ptr<vsomeip::message> & request,
    const uint8_t * data, size_t size) override;

  void on_message(MessageHandler handler) override;
  void on_availability(uint16_t service, AvailabilityHandler handler) override;
  void on_subscription(
    uint16_t service, uint16_t instance, uint16_t eventgroup,
    SubscriptionHandler handler) override;
  void clear_subscription_handler(
    uint16_t service, uint16_t instance, uint16_t eventgroup) override;

private:
  SomeipSession() = default;

  std::shared_ptr<vsomeip::application> app_;
  std::thread thread_;
  ParticipantIdentity identity_;
  ConfigGenerator::Config config_;
};

}  // namespace rmw_someip_cpp

#endif  // DETAIL__SOMEIP_SESSION_HPP_
