// Phase 0 spike 2, consumer side; see provider.cpp for what is being tested.
//
// Subscribes only to eventgroups 0x0001 (/chatter) and 0x0003 (/status).
// Receiving nothing from 0x0002 (/sensor) is what makes selective subscription,
// and therefore ROS topic subscription, viable.

#include <atomic>
#include <cstdint>
#include <iostream>
#include <map>
#include <set>
#include <string>

#include <vsomeip/vsomeip.hpp>

namespace {

constexpr vsomeip::service_t SERVICE_ID = 0x2000;
constexpr vsomeip::instance_t INSTANCE_ID = 0x0001;

// Only the subscribed topics; /sensor (0x8002/0x0002) is left out deliberately.
const std::map<vsomeip::event_t, vsomeip::eventgroup_t> kSubscribe{
  {0x8001, 0x0001},   // /chatter
  {0x8003, 0x0003},   // /status
};

std::shared_ptr<vsomeip::application> g_app;
std::atomic<int> g_total{0};
std::atomic<bool> g_saw_unsubscribed{false};

void log(const std::string & msg)
{
  std::cout << "[consumer2] " << msg << std::endl;
}

}  // namespace

int main()
{
  auto rt = vsomeip::runtime::get();
  g_app = rt->create_application("consumer");

  if (!g_app->init()) {
    std::cerr << "[consumer2] init() failed" << std::endl;
    return 1;
  }

  g_app->register_state_handler(
    [](vsomeip::state_type_e state) {
      if (state == vsomeip::state_type_e::ST_REGISTERED) {
        g_app->request_service(SERVICE_ID, INSTANCE_ID);
        log("issuing request_service(0x2000)");
      }
    });

  g_app->register_availability_handler(
    SERVICE_ID, INSTANCE_ID,
    [](vsomeip::service_t, vsomeip::instance_t, bool available) {
      if (!available) {
        return;
      }
      log("service AVAILABLE - subscribing to the chosen eventgroups only");
      for (const auto & [event_id, eg_id] : kSubscribe) {
        std::set<vsomeip::eventgroup_t> groups{eg_id};
        g_app->request_event(
          SERVICE_ID, INSTANCE_ID, event_id, groups,
          vsomeip::event_type_e::ET_FIELD,
          vsomeip::reliability_type_e::RT_UNRELIABLE);
        g_app->subscribe(SERVICE_ID, INSTANCE_ID, eg_id);
        char buf[96];
        snprintf(buf, sizeof(buf), "  subscribe eventgroup=0x%04x (event=0x%04x)", eg_id, event_id);
        log(buf);
      }
    });

  // Receive with ANY_METHOD, so an unsubscribed event arriving is detectable.
  g_app->register_message_handler(
    SERVICE_ID, INSTANCE_ID, vsomeip::ANY_METHOD,
    [](const std::shared_ptr<vsomeip::message> & msg) {
      auto payload = msg->get_payload();
      std::string s(
        reinterpret_cast<const char *>(payload->get_data()), payload->get_length());
      const auto event_id = msg->get_method();

      if (kSubscribe.find(event_id) == kSubscribe.end()) {
        g_saw_unsubscribed = true;
        char buf[128];
        snprintf(
          buf, sizeof(buf), "!!! received unsubscribed event=0x%04x: %s",
          event_id, s.c_str());
        log(buf);
        return;
      }

      char buf[160];
      snprintf(
        buf, sizeof(buf), "RECEIVED event=0x%04x payload='%s'", event_id, s.c_str());
      log(buf);

      if (++g_total == 8) {
        log(
          g_saw_unsubscribed
          ? "=== SPIKE2 RESULT: FAIL - an unsubscribed topic leaked through ==="
          : "=== SPIKE2 RESULT: SUCCESS - runtime IDs and selective subscription work ===");
      }
    });

  g_app->start();
  return 0;
}
