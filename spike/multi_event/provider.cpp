// Phase 0 spike 2: one service per process, one event and eventgroup per topic.
//
// spike/dynamic_id established that:
//   - a service and instance must be declared with a port or no SD offer is sent
//   - events and eventgroups need no JSON and work from the runtime API alone
//   - the consumer needs no services section at all
//
// So an RMW that declares one service per process and assigns ROS topics to
// events and eventgroups within it can handle any number of topics created at
// runtime with no static configuration. This demonstrates that.
//
// Passes when the consumer receives eventgroups 0x0001 and 0x0003 but not 0x0002.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <vsomeip/vsomeip.hpp>

namespace {

// The only thing declared in the JSON, with a port.
constexpr vsomeip::service_t SERVICE_ID = 0x2000;
constexpr vsomeip::instance_t INSTANCE_ID = 0x0001;

// None of the following appear in the JSON; they stand in for ROS topics.
struct TopicMapping
{
  const char * ros_topic;
  vsomeip::event_t event_id;
  vsomeip::eventgroup_t eventgroup_id;
};

const std::vector<TopicMapping> kTopics{
  {"/chatter", 0x8001, 0x0001},
  {"/sensor", 0x8002, 0x0002},
  {"/status", 0x8003, 0x0003},
};

std::shared_ptr<vsomeip::application> g_app;
std::atomic<bool> g_running{true};

void log(const std::string & msg)
{
  std::cout << "[provider2] " << msg << std::endl;
}

}  // namespace

int main()
{
  auto rt = vsomeip::runtime::get();
  g_app = rt->create_application("provider");

  if (!g_app->init()) {
    std::cerr << "[provider2] init() failed" << std::endl;
    return 1;
  }

  std::thread sender(
    [rt]() {
      std::this_thread::sleep_for(std::chrono::seconds(2));

      g_app->offer_service(SERVICE_ID, INSTANCE_ID);
      log("offer_service(0x2000) - the one service the JSON declares");

      // The point of the spike: create three topics' worth at runtime.
      for (const auto & t : kTopics) {
        std::set<vsomeip::eventgroup_t> groups{t.eventgroup_id};
        g_app->offer_event(
          SERVICE_ID, INSTANCE_ID, t.event_id, groups,
          vsomeip::event_type_e::ET_FIELD,
          std::chrono::milliseconds::zero(), false, true, nullptr,
          vsomeip::reliability_type_e::RT_UNRELIABLE);
        char buf[128];
        snprintf(
          buf, sizeof(buf), "offer_event %s -> event=0x%04x eventgroup=0x%04x (not in the JSON)",
          t.ros_topic, t.event_id, t.eventgroup_id);
        log(buf);
      }

      uint32_t counter = 0;
      while (g_running) {
        for (const auto & t : kTopics) {
          // Payload is "<topic>#<counter>" as plain UTF-8.
          std::string s = std::string(t.ros_topic) + "#" + std::to_string(counter);
          std::vector<vsomeip::byte_t> data(s.begin(), s.end());
          g_app->notify(SERVICE_ID, INSTANCE_ID, t.event_id, rt->create_payload(data));
        }
        log("notify all topics, counter=" + std::to_string(counter));
        ++counter;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });

  g_app->start();

  g_running = false;
  if (sender.joinable()) {
    sender.join();
  }
  return 0;
}
