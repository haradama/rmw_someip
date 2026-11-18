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

#include "detail/someip_session.hpp"

#include <cstdlib>
#include <future>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "detail/config_generator.hpp"

namespace rmw_someip_cpp
{

namespace
{
constexpr auto kRegistrationTimeout = std::chrono::seconds(5);
}

std::unique_ptr<SomeipSession> SomeipSession::create(
  const ParticipantIdentity & id,
  const ConfigGenerator::Config & config,
  std::string & error)
{
  // vsomeip reads both of these in create_application().
  ::setenv("VSOMEIP_CONFIGURATION", config.path.c_str(), 1);
  ::setenv("VSOMEIP_APPLICATION_NAME", id.application_name.c_str(), 1);

  auto runtime = vsomeip::runtime::get();
  if (!runtime) {
    error = "vsomeip::runtime::get() returned null";
    return nullptr;
  }

  auto app = runtime->create_application(id.application_name);
  if (!app) {
    error = "vsomeip create_application(\"" + id.application_name + "\") failed";
    return nullptr;
  }

  // Synchronization for the registration wait. The state handler has to be
  // registered before init().
  auto registered = std::make_shared<std::promise<void>>();
  auto registered_future = registered->get_future();
  auto fulfilled = std::make_shared<std::atomic<bool>>(false);

  app->register_state_handler(
    [registered, fulfilled](vsomeip::state_type_e state) {
      if (state == vsomeip::state_type_e::ST_REGISTERED) {
        // Can fire repeatedly on reconnect, so set the promise once.
        if (!fulfilled->exchange(true)) {
          registered->set_value();
        }
      }
    });

  if (!app->init()) {
    error =
      "vsomeip application::init() failed. Check the generated configuration at " + config.path;
    return nullptr;
  }

  auto session = std::unique_ptr<SomeipSession>(new SomeipSession());
  session->app_ = app;
  session->identity_ = id;
  session->config_ = config;
  // start() blocks, so it gets its own thread.
  session->thread_ = std::thread([app]() {app->start();});

  if (registered_future.wait_for(kRegistrationTimeout) != std::future_status::ready) {
    error =
      "timed out waiting for the vsomeip routing manager registration. "
      "If another process on this host hosts the routing manager, set "
      "RMW_SOMEIP_ROUTING_HOST to its application name.";
    // The session destructor does the stop and join.
    return nullptr;
  }

  return session;
}

SomeipSession::~SomeipSession()
{
  if (app_) {
    app_->clear_all_handler();
    app_->stop();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  ConfigGenerator::cleanup(config_);
}

// ---------------------------------------------------------------------------
// SomeipEndpoint implementation
//
// All of it delegates to vsomeip. Converting plain values (bools, byte spans)
// into vsomeip types happens only here, so the call sites stay free of them.
// ---------------------------------------------------------------------------

namespace
{

vsomeip::event_type_e event_type_of(bool is_field)
{
  return is_field ? vsomeip::event_type_e::ET_FIELD : vsomeip::event_type_e::ET_EVENT;
}

vsomeip::reliability_type_e reliability_of(bool reliable)
{
  return reliable ?
         vsomeip::reliability_type_e::RT_RELIABLE :
         vsomeip::reliability_type_e::RT_UNRELIABLE;
}

std::shared_ptr<vsomeip::payload> make_payload(const uint8_t * data, size_t size)
{
  auto payload = vsomeip::runtime::get()->create_payload();
  if (size > 0) {
    payload->set_data(data, static_cast<uint32_t>(size));
  }
  return payload;
}

}  // namespace

void SomeipSession::offer_service(uint16_t service, uint16_t instance)
{
  app_->offer_service(service, instance);
}

void SomeipSession::offer_event(
  uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
  bool is_field, bool reliable)
{
  app_->offer_event(
    service, instance, event, std::set<vsomeip::eventgroup_t>{eventgroup},
    event_type_of(is_field), std::chrono::milliseconds::zero(), false, true, nullptr,
    reliability_of(reliable));
}

void SomeipSession::stop_offer_event(uint16_t service, uint16_t instance, uint16_t event)
{
  app_->stop_offer_event(service, instance, event);
}

void SomeipSession::notify(
  uint16_t service, uint16_t instance, uint16_t event, const uint8_t * data, size_t size)
{
  app_->notify(service, instance, event, make_payload(data, size));
}

void SomeipSession::notify_one(
  uint16_t service, uint16_t instance, uint16_t event,
  const uint8_t * data, size_t size, uint16_t client)
{
  app_->notify_one(service, instance, event, make_payload(data, size), client);
}

void SomeipSession::request_service(uint16_t service, uint16_t instance)
{
  app_->request_service(service, instance);
}

void SomeipSession::request_event(
  uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
  bool is_field, bool reliable)
{
  app_->request_event(
    service, instance, event, std::set<vsomeip::eventgroup_t>{eventgroup},
    event_type_of(is_field), reliability_of(reliable));
}

void SomeipSession::subscribe(uint16_t service, uint16_t instance, uint16_t eventgroup)
{
  app_->subscribe(service, instance, eventgroup);
}

void SomeipSession::unsubscribe(uint16_t service, uint16_t instance, uint16_t eventgroup)
{
  app_->unsubscribe(service, instance, eventgroup);
}

void SomeipSession::release_event(uint16_t service, uint16_t instance, uint16_t event)
{
  app_->release_event(service, instance, event);
}

void SomeipSession::send_request(
  uint16_t service, uint16_t instance, uint16_t method, const uint8_t * data, size_t size)
{
  auto request = vsomeip::runtime::get()->create_request();
  request->set_service(service);
  request->set_instance(instance);
  request->set_method(method);
  request->set_payload(make_payload(data, size));
  app_->send(request);
}

void SomeipSession::send_response(
  const std::shared_ptr<vsomeip::message> & request, const uint8_t * data, size_t size)
{
  // create_response() inherits the request ID from the original message.
  auto response = vsomeip::runtime::get()->create_response(request);
  response->set_payload(make_payload(data, size));
  app_->send(response);
}

void SomeipSession::on_message(MessageHandler handler)
{
  app_->register_message_handler(
    vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE, vsomeip::ANY_METHOD, std::move(handler));
}

void SomeipSession::on_availability(uint16_t service, AvailabilityHandler handler)
{
  app_->register_availability_handler(
    service, vsomeip::ANY_INSTANCE,
    [handler](vsomeip::service_t, vsomeip::instance_t instance, bool available) {
      // An ANY_INSTANCE handler fires once with instance=0xFFFF.
      // That is not a real participant, so drop it.
      if (instance == vsomeip::ANY_INSTANCE) {
        return;
      }
      handler(static_cast<uint16_t>(instance), available);
    });
}

void SomeipSession::on_subscription(
  uint16_t service, uint16_t instance, uint16_t eventgroup, SubscriptionHandler handler)
{
  app_->register_subscription_handler(
    service, instance, eventgroup,
    [handler](
      vsomeip::client_t client, const vsomeip_sec_client_t *,
      const std::string &, bool subscribed) {
      if (subscribed) {
        handler(static_cast<uint16_t>(client));
      }
      // The return value accepts or rejects the subscription; always accept.
      return true;
    });
}

void SomeipSession::clear_subscription_handler(
  uint16_t service, uint16_t instance, uint16_t eventgroup)
{
  app_->unregister_subscription_handler(service, instance, eventgroup);
}

}  // namespace rmw_someip_cpp
