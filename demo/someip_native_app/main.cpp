// A native SOME/IP application, standing in for an ECU.
//
// Nothing here depends on ROS. It uses vsomeip alone and assembles payloads
// by hand according to the AUTOSAR SOME/IP Transformer — by hand here, from
// FIDL- or ARXML-generated code on a real ECU.
//
//   /vehicle/status  : offered here, subscribed to by the ROS 2 side
//   /vehicle/command : offered by the ROS 2 side, subscribed to here
//
// The byte layout, per the AUTOSAR SOME/IP Transformer:
//   - big endian
//   - struct members packed, with none of CDR's alignment padding
//   - variable-length data prefixed with a 4-byte length, counting bytes
//     rather than elements; for a string the length includes the NUL

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <vsomeip/vsomeip.hpp>

namespace
{

// Must match someip_mapping.yaml, which is where the ROS side reads them from.
constexpr vsomeip::service_t kStatusService = 0x1234;
constexpr vsomeip::instance_t kStatusInstance = 0x0001;
constexpr vsomeip::event_t kStatusEvent = 0x8001;
constexpr vsomeip::eventgroup_t kStatusEventgroup = 0x0001;

constexpr vsomeip::service_t kCommandService = 0x1235;
constexpr vsomeip::instance_t kCommandInstance = 0x0001;
constexpr vsomeip::event_t kCommandEvent = 0x8001;
constexpr vsomeip::eventgroup_t kCommandEventgroup = 0x0001;

std::shared_ptr<vsomeip::application> g_app;
std::atomic<bool> g_running{true};

void log(const std::string & message)
{
  std::cout << "[someip_ecu] " << message << std::endl;
}

// --- Big-endian writing ------------------------------------------------------

void put_u8(std::vector<vsomeip::byte_t> & buffer, uint8_t value)
{
  buffer.push_back(value);
}

void put_u32(std::vector<vsomeip::byte_t> & buffer, uint32_t value)
{
  buffer.push_back(static_cast<vsomeip::byte_t>((value >> 24) & 0xFF));
  buffer.push_back(static_cast<vsomeip::byte_t>((value >> 16) & 0xFF));
  buffer.push_back(static_cast<vsomeip::byte_t>((value >> 8) & 0xFF));
  buffer.push_back(static_cast<vsomeip::byte_t>(value & 0xFF));
}

void put_f32(std::vector<vsomeip::byte_t> & buffer, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  put_u32(buffer, bits);
}

/// A variable-length string: 4-byte length including the NUL, body, then NUL.
void put_string(std::vector<vsomeip::byte_t> & buffer, const std::string & value)
{
  put_u32(buffer, static_cast<uint32_t>(value.size() + 1));
  buffer.insert(buffer.end(), value.begin(), value.end());
  buffer.push_back(0);
}

// --- Big-endian reading ------------------------------------------------------

class Reader
{
public:
  Reader(const vsomeip::byte_t * data, size_t size)
  : data_(data), size_(size) {}

  bool read_u8(uint8_t & value)
  {
    if (position_ + 1 > size_) {
      return false;
    }
    value = data_[position_++];
    return true;
  }

  bool read_u32(uint32_t & value)
  {
    if (position_ + 4 > size_) {
      return false;
    }
    value = (static_cast<uint32_t>(data_[position_]) << 24) |
      (static_cast<uint32_t>(data_[position_ + 1]) << 16) |
      (static_cast<uint32_t>(data_[position_ + 2]) << 8) |
      static_cast<uint32_t>(data_[position_ + 3]);
    position_ += 4;
    return true;
  }

  bool read_f32(float & value)
  {
    uint32_t bits = 0;
    if (!read_u32(bits)) {
      return false;
    }
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

private:
  const vsomeip::byte_t * data_;
  size_t size_;
  size_t position_ {0};
};

/// Decode a VehicleCommand that arrived from ROS 2.
void on_command(const std::shared_ptr<vsomeip::message> & message)
{
  auto payload = message->get_payload();
  if (!payload) {
    return;
  }
  Reader reader(payload->get_data(), payload->get_length());

  float target_speed = 0.0F;
  uint8_t gear_request = 0;
  if (!reader.read_f32(target_speed) || !reader.read_u8(gear_request)) {
    log("!!! VehicleCommand payload is too short");
    return;
  }

  std::ostringstream os;
  os << "RECV /vehicle/command from ROS 2: target_speed=" << std::fixed
     << std::setprecision(1) << target_speed << " gear_request=" << static_cast<int>(gear_request);
  log(os.str());
}

}  // namespace

int main()
{
  auto runtime = vsomeip::runtime::get();
  g_app = runtime->create_application("someip_ecu");
  if (!g_app->init()) {
    std::cerr << "[someip_ecu] init() failed" << std::endl;
    return 1;
  }

  // Subscribe to /vehicle/command, which the ROS 2 side offers.
  g_app->register_availability_handler(
    kCommandService, kCommandInstance,
    [](vsomeip::service_t, vsomeip::instance_t, bool available) {
      if (!available) {
        return;
      }
      log("the ROS 2 side's /vehicle/command appeared; subscribing");
      g_app->request_event(
        kCommandService, kCommandInstance, kCommandEvent,
        std::set<vsomeip::eventgroup_t>{kCommandEventgroup},
        vsomeip::event_type_e::ET_EVENT,
        vsomeip::reliability_type_e::RT_UNRELIABLE);
      g_app->subscribe(kCommandService, kCommandInstance, kCommandEventgroup);
    });

  g_app->register_message_handler(
    kCommandService, kCommandInstance, kCommandEvent, on_command);

  std::thread worker(
    [runtime]() {
      std::this_thread::sleep_for(std::chrono::seconds(2));

      g_app->offer_service(kStatusService, kStatusInstance);
      g_app->offer_event(
        kStatusService, kStatusInstance, kStatusEvent,
        std::set<vsomeip::eventgroup_t>{kStatusEventgroup},
        vsomeip::event_type_e::ET_EVENT,
        std::chrono::milliseconds::zero(), false, true, nullptr,
        vsomeip::reliability_type_e::RT_UNRELIABLE);
      log("offer /vehicle/status (service 0x1234 / event 0x8001)");

      g_app->request_service(kCommandService, kCommandInstance);

      const char * modes[] = {"ECO", "NORMAL", "SPORT"};
      uint32_t tick = 0;
      while (g_running) {
        std::vector<vsomeip::byte_t> buffer;
        const auto speed = 40.0F + static_cast<float>(tick % 20);
        const auto steering = -10.0F + static_cast<float>(tick % 21);
        const auto gear = static_cast<uint8_t>(1 + (tick % 6));
        const std::string mode = modes[tick % 3];

        put_u32(buffer, tick * 100);      // timestamp_ms
        put_f32(buffer, speed);           // speed_kph
        put_f32(buffer, steering);        // steering_angle_deg
        put_u8(buffer, gear);             // gear
        put_string(buffer, mode);         // variable length

        g_app->notify(
          kStatusService, kStatusInstance, kStatusEvent,
          runtime->create_payload(buffer));

        std::ostringstream os;
        os << "SEND /vehicle/status: speed=" << std::fixed << std::setprecision(1) << speed
           << " steering=" << steering << " gear=" << static_cast<int>(gear)
           << " mode=" << mode << "  (" << buffer.size() << " bytes)";
        log(os.str());

        ++tick;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });

  g_app->start();

  g_running = false;
  if (worker.joinable()) {
    worker.join();
  }
  return 0;
}
