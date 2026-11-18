// Phase 0 spike 3: wildcard discovery through ANY_INSTANCE.
//
// Every participant offers the same well-known service_id under its own
// instance_id. If request_service(SERVICE, ANY_INSTANCE) discovers all of them
// at once, a whole ROS domain can be represented by a single service.
//
// PARTICIPANT_INSTANCE supplies the instance_id.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vsomeip/vsomeip.hpp>

namespace {
constexpr vsomeip::service_t GRAPH_SERVICE_ID = 0xFF00;   // well-known, shared by the domain
constexpr vsomeip::event_t GRAPH_EVENT_ID = 0x8000;
constexpr vsomeip::eventgroup_t GRAPH_EVENTGROUP_ID = 0x0001;
}

int main()
{
  const auto instance_id = static_cast<vsomeip::instance_t>(
    std::stoul(std::getenv("PARTICIPANT_INSTANCE"), nullptr, 0));
  const std::string tag = "[provider-" + std::to_string(instance_id) + "] ";

  auto rt = vsomeip::runtime::get();
  auto app = rt->create_application("provider");
  if (!app->init()) { std::cerr << tag << "init failed\n"; return 1; }

  std::thread sender([&]() {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    app->offer_service(GRAPH_SERVICE_ID, instance_id);
    std::set<vsomeip::eventgroup_t> groups{GRAPH_EVENTGROUP_ID};
    app->offer_event(GRAPH_SERVICE_ID, instance_id, GRAPH_EVENT_ID, groups,
      vsomeip::event_type_e::ET_FIELD, std::chrono::milliseconds::zero(),
      false, true, nullptr, vsomeip::reliability_type_e::RT_UNRELIABLE);
    std::cout << tag << "offered service=0xFF00 instance=" << instance_id << std::endl;

    int n = 0;
    while (true) {
      std::string s = "graph-from-instance-" + std::to_string(instance_id) + "#" + std::to_string(n++);
      std::vector<vsomeip::byte_t> d(s.begin(), s.end());
      app->notify(GRAPH_SERVICE_ID, instance_id, GRAPH_EVENT_ID, rt->create_payload(d));
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });
  app->start();
  return 0;
}
