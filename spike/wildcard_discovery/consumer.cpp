// Phase 0 spike 3, consumer side; see provider.cpp for what is being tested.
// Passes when instances 0x0001 and 0x0002 are both discovered by a single
// request_service.

#include <atomic>
#include <iostream>
#include <set>
#include <mutex>
#include <vsomeip/vsomeip.hpp>

namespace {
constexpr vsomeip::service_t GRAPH_SERVICE_ID = 0xFF00;
constexpr vsomeip::event_t GRAPH_EVENT_ID = 0x8000;
constexpr vsomeip::eventgroup_t GRAPH_EVENTGROUP_ID = 0x0001;
std::shared_ptr<vsomeip::application> g_app;
std::mutex g_mtx;
std::set<vsomeip::instance_t> g_found;
std::set<vsomeip::instance_t> g_received_from;
}

int main()
{
  auto rt = vsomeip::runtime::get();
  g_app = rt->create_application("consumer");
  if (!g_app->init()) { std::cerr << "[consumer3] init failed\n"; return 1; }

  g_app->register_state_handler([](vsomeip::state_type_e s) {
    if (s == vsomeip::state_type_e::ST_REGISTERED) {
      // The point of the spike: one request, with a wildcard instance.
      g_app->request_service(GRAPH_SERVICE_ID, vsomeip::ANY_INSTANCE);
      std::cout << "[consumer3] issuing one request_service(0xFF00, ANY_INSTANCE)" << std::endl;
    }
  });

  g_app->register_availability_handler(
    GRAPH_SERVICE_ID, vsomeip::ANY_INSTANCE,
    [](vsomeip::service_t, vsomeip::instance_t inst, bool available) {
      if (!available) return;
      std::lock_guard<std::mutex> lk(g_mtx);
      if (!g_found.insert(inst).second) return;
      std::cout << "[consumer3] DISCOVERED instance=" << inst << std::endl;
      std::set<vsomeip::eventgroup_t> groups{GRAPH_EVENTGROUP_ID};
      g_app->request_event(GRAPH_SERVICE_ID, inst, GRAPH_EVENT_ID, groups,
        vsomeip::event_type_e::ET_FIELD, vsomeip::reliability_type_e::RT_UNRELIABLE);
      g_app->subscribe(GRAPH_SERVICE_ID, inst, GRAPH_EVENTGROUP_ID);
    });

  g_app->register_message_handler(
    GRAPH_SERVICE_ID, vsomeip::ANY_INSTANCE, vsomeip::ANY_METHOD,
    [](const std::shared_ptr<vsomeip::message> & m) {
      auto p = m->get_payload();
      std::string s(reinterpret_cast<const char *>(p->get_data()), p->get_length());
      std::lock_guard<std::mutex> lk(g_mtx);
      bool is_new = g_received_from.insert(m->get_instance()).second;
      std::cout << "[consumer3] RECV inst=" << m->get_instance() << " '" << s << "'" << std::endl;
      if (is_new && g_received_from.size() == 2) {
        std::cout << "[consumer3] === SPIKE3 RESULT: SUCCESS - ANY_INSTANCE found every participant ===" << std::endl;
      }
    });

  g_app->start();
  return 0;
}
