// Phase 0 spike, consumer side; see provider.cpp for what is being tested.
//
// Passes when "AVAILABLE" is printed and counters keep arriving after it.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include <vsomeip/vsomeip.hpp>

namespace {

constexpr vsomeip::service_t SERVICE_ID = 0x1234;
constexpr vsomeip::instance_t INSTANCE_ID = 0x0001;
constexpr vsomeip::eventgroup_t EVENTGROUP_ID = 0x0001;
constexpr vsomeip::event_t EVENT_ID = 0x8001;

std::shared_ptr<vsomeip::application> g_app;
std::atomic<int> g_received{0};

void log(const std::string & msg)
{
  std::cout << "[consumer] " << msg << std::endl;
}

}  // namespace

int main()
{
  auto rt = vsomeip::runtime::get();
  g_app = rt->create_application("consumer");

  if (!g_app->init()) {
    std::cerr << "[consumer] init() failed" << std::endl;
    return 1;
  }

  g_app->register_state_handler(
    [](vsomeip::state_type_e state) {
      if (state == vsomeip::state_type_e::ST_REGISTERED) {
        log("routing manager: REGISTERED - issuing request_service");
        // Requesting IDs that appear nowhere in the JSON.
        g_app->request_service(SERVICE_ID, INSTANCE_ID);
      }
    });

  g_app->register_availability_handler(
    SERVICE_ID, INSTANCE_ID,
    [](vsomeip::service_t s, vsomeip::instance_t i, bool available) {
      char buf[128];
      snprintf(
        buf, sizeof(buf), "service [%04x.%04x] %s", s, i,
        available ? "AVAILABLE" : "NOT AVAILABLE");
      log(buf);

      if (available) {
        std::set<vsomeip::eventgroup_t> groups{EVENTGROUP_ID};
        g_app->request_event(
          SERVICE_ID, INSTANCE_ID, EVENT_ID, groups,
          vsomeip::event_type_e::ET_FIELD,
          vsomeip::reliability_type_e::RT_UNRELIABLE);
        g_app->subscribe(SERVICE_ID, INSTANCE_ID, EVENTGROUP_ID);
        log("request_event and subscribe done");
      }
    });

  g_app->register_message_handler(
    SERVICE_ID, INSTANCE_ID, EVENT_ID,
    [](const std::shared_ptr<vsomeip::message> & msg) {
      auto payload = msg->get_payload();
      uint32_t value = 0;
      if (payload->get_length() >= 4) {
        const auto * d = payload->get_data();
        value = (static_cast<uint32_t>(d[0]) << 24) |
          (static_cast<uint32_t>(d[1]) << 16) |
          (static_cast<uint32_t>(d[2]) << 8) |
          static_cast<uint32_t>(d[3]);
      }
      int n = ++g_received;
      log("RECEIVED counter=" + std::to_string(value) + " (total=" + std::to_string(n) + ")");

      if (n == 5) {
        log("=== SPIKE RESULT: SUCCESS - runtime IDs and SD across Docker both work ===");
      }
    });

  g_app->start();  // blocks
  return 0;
}
