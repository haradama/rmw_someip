// Phase 0 spike, provider side.
//
// Two questions:
//   S1. Can offer_service() and offer_event() work from the runtime API alone,
//       with no "services" section in vsomeip.json? An RMW cannot know at
//       rmw_init() which topics will be created, so if this is impossible the
//       design has to pre-generate a pool of IDs into the JSON instead.
//   S2. Does SOME/IP-SD (UDP multicast) cross a Docker bridge network?
//
// Expected: the consumer sees availability=true and receives the notifications.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include <vsomeip/vsomeip.hpp>

namespace {

// Not in the JSON at all. Using IDs defined only here is the point.
constexpr vsomeip::service_t SERVICE_ID = 0x1234;
constexpr vsomeip::instance_t INSTANCE_ID = 0x0001;
constexpr vsomeip::eventgroup_t EVENTGROUP_ID = 0x0001;
constexpr vsomeip::event_t EVENT_ID = 0x8001;  // events start at 0x8000

std::shared_ptr<vsomeip::application> g_app;
std::atomic<bool> g_running{true};

void log(const std::string & msg)
{
  std::cout << "[provider] " << msg << std::endl;
}

}  // namespace

int main()
{
  auto rt = vsomeip::runtime::get();
  g_app = rt->create_application("provider");

  if (!g_app->init()) {
    std::cerr << "[provider] init() failed" << std::endl;
    return 1;
  }

  g_app->register_state_handler(
    [](vsomeip::state_type_e state) {
      log(state == vsomeip::state_type_e::ST_REGISTERED
        ? "routing manager: REGISTERED"
        : "routing manager: DEREGISTERED");
    });

  // Make subscribe and unsubscribe visible. TRANSIENT_LOCAL replay is meant to
  // hang off this hook, so it is worth confirming it works this early.
  g_app->register_subscription_status_handler(
    SERVICE_ID, INSTANCE_ID, EVENTGROUP_ID, EVENT_ID,
    [](vsomeip::service_t, vsomeip::instance_t, vsomeip::eventgroup_t,
      vsomeip::event_t, uint16_t error) {
      log(error == 0 ? "subscription ACCEPTED by us" : "subscription error");
    });

  std::thread sender(
    [rt]() {
      // Let registration and the first SD offer settle.
      std::this_thread::sleep_for(std::chrono::seconds(2));

      log("offer_service(0x1234, 0x0001) - IDs not declared in the JSON");
      g_app->offer_service(SERVICE_ID, INSTANCE_ID);

      std::set<vsomeip::eventgroup_t> groups{EVENTGROUP_ID};
      g_app->offer_event(
        SERVICE_ID, INSTANCE_ID, EVENT_ID, groups,
        vsomeip::event_type_e::ET_FIELD,          // delivers the latest value on subscribe
        std::chrono::milliseconds::zero(),
        false, true, nullptr,
        vsomeip::reliability_type_e::RT_UNRELIABLE);  // UDP
      log("offer_event(0x8001) as ET_FIELD / UDP");

      uint32_t counter = 0;
      while (g_running) {
        // A plain big-endian uint32 counter is enough for the spike.
        std::vector<vsomeip::byte_t> data{
          static_cast<vsomeip::byte_t>((counter >> 24) & 0xFF),
          static_cast<vsomeip::byte_t>((counter >> 16) & 0xFF),
          static_cast<vsomeip::byte_t>((counter >> 8) & 0xFF),
          static_cast<vsomeip::byte_t>(counter & 0xFF)};

        auto payload = vsomeip::runtime::get()->create_payload(data);
        g_app->notify(SERVICE_ID, INSTANCE_ID, EVENT_ID, payload);
        log("notify counter=" + std::to_string(counter));

        ++counter;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });

  g_app->start();  // blocks

  g_running = false;
  if (sender.joinable()) {
    sender.join();
  }
  return 0;
}
