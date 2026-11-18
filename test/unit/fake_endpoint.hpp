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

#ifndef FAKE_ENDPOINT_HPP_
#define FAKE_ENDPOINT_HPP_

#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "detail/someip_endpoint.hpp"

namespace rmw_someip_cpp_test
{

/// A fake SOME/IP endpoint.
///
/// Records the calls made to it and touches neither the network nor a routing
/// manager, which is what lets `rmw_context_impl_s` — graph interpretation,
/// subscription matching, routing tables, QoS event derivation — run without
/// SOME/IP at all.
///
/// Incoming events (a peer appeared, a graph arrived, someone subscribed) are
/// provoked by invoking the stored handlers directly. None of them can be
/// produced on demand against the real thing, which was the main obstacle to
/// testing this at all.
class FakeEndpoint : public rmw_someip_cpp::SomeipEndpoint
{
public:
  struct Offered
  {
    uint16_t service, instance, event, eventgroup;
    bool is_field, reliable;
  };
  struct Subscribed
  {
    uint16_t service, instance, eventgroup;
  };
  struct Notified
  {
    uint16_t service, instance, event;
    std::vector<uint8_t> payload;
    uint16_t client;  ///< only meaningful for notify_one
  };

  // -- What was called -------------------------------------------------------
  std::vector<std::pair<uint16_t, uint16_t>> offered_services;
  std::vector<Offered> offered_events;
  std::vector<std::tuple<uint16_t, uint16_t, uint16_t>> stopped_events;
  std::vector<std::pair<uint16_t, uint16_t>> requested_services;
  std::vector<Offered> requested_events;
  std::vector<Subscribed> subscribed;
  std::vector<Subscribed> unsubscribed;
  std::vector<std::tuple<uint16_t, uint16_t, uint16_t>> released_events;
  std::vector<Notified> notified;
  std::vector<Notified> notified_one;
  std::vector<Notified> requests_sent;
  std::vector<std::vector<uint8_t>> responses_sent;
  std::vector<Subscribed> cleared_subscription_handlers;

  // -- Raise events upwards --------------------------------------------------

  void fire_availability(uint16_t instance, bool available)
  {
    if (availability_handler_) {
      availability_handler_(instance, available);
    }
  }

  void fire_message(const std::shared_ptr<vsomeip::message> & message)
  {
    if (message_handler_) {
      message_handler_(message);
    }
  }

  /// Invoke the subscription handler for this eventgroup, if one is registered.
  bool fire_subscription(uint16_t service, uint16_t instance, uint16_t eventgroup,
    uint16_t client)
  {
    for (const auto & entry : subscription_handlers_) {
      if (std::get<0>(entry.first) == service && std::get<1>(entry.first) == instance &&
        std::get<2>(entry.first) == eventgroup)
      {
        entry.second(client);
        return true;
      }
    }
    return false;
  }

  bool has_message_handler() const {return static_cast<bool>(message_handler_);}
  bool has_availability_handler() const {return static_cast<bool>(availability_handler_);}
  size_t subscription_handler_count() const {return subscription_handlers_.size();}

  // -- SomeipEndpoint --------------------------------------------------------

  void offer_service(uint16_t service, uint16_t instance) override
  {
    offered_services.push_back({service, instance});
  }
  void offer_event(
    uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
    bool is_field, bool reliable) override
  {
    offered_events.push_back({service, instance, event, eventgroup, is_field, reliable});
  }
  void stop_offer_event(uint16_t service, uint16_t instance, uint16_t event) override
  {
    stopped_events.push_back({service, instance, event});
  }
  void notify(
    uint16_t service, uint16_t instance, uint16_t event,
    const uint8_t * data, size_t size) override
  {
    notified.push_back({service, instance, event, {data, data + size}, 0});
  }
  void notify_one(
    uint16_t service, uint16_t instance, uint16_t event,
    const uint8_t * data, size_t size, uint16_t client) override
  {
    notified_one.push_back({service, instance, event, {data, data + size}, client});
  }

  void request_service(uint16_t service, uint16_t instance) override
  {
    requested_services.push_back({service, instance});
  }
  void request_event(
    uint16_t service, uint16_t instance, uint16_t event, uint16_t eventgroup,
    bool is_field, bool reliable) override
  {
    requested_events.push_back({service, instance, event, eventgroup, is_field, reliable});
  }
  void subscribe(uint16_t service, uint16_t instance, uint16_t eventgroup) override
  {
    subscribed.push_back({service, instance, eventgroup});
  }
  void unsubscribe(uint16_t service, uint16_t instance, uint16_t eventgroup) override
  {
    unsubscribed.push_back({service, instance, eventgroup});
  }
  void release_event(uint16_t service, uint16_t instance, uint16_t event) override
  {
    released_events.push_back({service, instance, event});
  }

  void send_request(
    uint16_t service, uint16_t instance, uint16_t method,
    const uint8_t * data, size_t size) override
  {
    requests_sent.push_back({service, instance, method, {data, data + size}, 0});
  }
  void send_response(
    const std::shared_ptr<vsomeip::message> &, const uint8_t * data, size_t size) override
  {
    responses_sent.push_back({data, data + size});
  }

  void on_message(MessageHandler handler) override {message_handler_ = std::move(handler);}
  void on_availability(uint16_t, AvailabilityHandler handler) override
  {
    availability_handler_ = std::move(handler);
  }
  void on_subscription(
    uint16_t service, uint16_t instance, uint16_t eventgroup,
    SubscriptionHandler handler) override
  {
    subscription_handlers_.emplace_back(
      std::make_tuple(service, instance, eventgroup), std::move(handler));
  }
  void clear_subscription_handler(
    uint16_t service, uint16_t instance, uint16_t eventgroup) override
  {
    cleared_subscription_handlers.push_back({service, instance, eventgroup});
    for (auto it = subscription_handlers_.begin(); it != subscription_handlers_.end(); ++it) {
      if (it->first == std::make_tuple(service, instance, eventgroup)) {
        subscription_handlers_.erase(it);
        return;
      }
    }
  }

private:
  MessageHandler message_handler_;
  AvailabilityHandler availability_handler_;
  std::vector<std::pair<std::tuple<uint16_t, uint16_t, uint16_t>, SubscriptionHandler>>
  subscription_handlers_;
};

}  // namespace rmw_someip_cpp_test

#endif  // FAKE_ENDPOINT_HPP_
