// Smoke test for the pub/sub path.
//
// An rclcpp Node creates six parameter services by default, so demo_nodes_cpp
// cannot start until services are implemented. start_parameter_services(false)
// removes them and leaves pub/sub on its own.
//
// Usage:  pubsub_smoke talker  /  pubsub_smoke listener
// The listener prints SMOKE RESULT and exits once it has received enough.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

/// NodeOptions that create no services at all.
///
/// An rclcpp Node creates six parameter services plus ~/get_type_description by
/// default. The latter is controlled by a node parameter rather than
/// NodeOptions, so a parameter override turns it off. What remains is only the
/// pub/sub path.
rclcpp::NodeOptions minimal_options()
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.append_parameter_override("start_type_description_service", false);
  return options;
}

/// Send a large message. SOME/IP's UDP has a maximum message length (1416
/// bytes by default), so this checks that a large BEST_EFFORT topic gets through.
int run_large_talker(int payload_bytes, bool best_effort)
{
  auto node = std::make_shared<rclcpp::Node>("large_talker", minimal_options());
  auto qos = best_effort ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);
  auto publisher = node->create_publisher<std_msgs::msg::ByteMultiArray>("large", qos);

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [publisher, payload_bytes, node]() {
      std_msgs::msg::ByteMultiArray message;
      message.data.assign(static_cast<size_t>(payload_bytes), 0xAB);
      RCLCPP_INFO(node->get_logger(), "Publishing %d bytes", payload_bytes);
      publisher->publish(message);
    });
  rclcpp::spin(node);
  return 0;
}

int run_large_listener(int payload_bytes, bool best_effort)
{
  auto node = std::make_shared<rclcpp::Node>("large_listener", minimal_options());
  auto qos = best_effort ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);
  auto subscription = node->create_subscription<std_msgs::msg::ByteMultiArray>(
    "large", qos,
    [payload_bytes, node](const std_msgs::msg::ByteMultiArray & message) {
      const bool ok = message.data.size() == static_cast<size_t>(payload_bytes);
      std::printf(
        "=== LARGE RESULT: %s - received %zu bytes, expected %d ===\n",
        ok ? "SUCCESS" : "SIZE MISMATCH", message.data.size(), payload_bytes);
      std::fflush(stdout);
      rclcpp::shutdown();
    });
  rclcpp::spin(node);
  return 0;
}

int run_talker()
{
  auto node = std::make_shared<rclcpp::Node>("smoke_talker", minimal_options());
  auto publisher = node->create_publisher<std_msgs::msg::String>("chatter", 10);

  int count = 0;
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [&publisher, &count, node]() {
      std_msgs::msg::String message;
      message.data = "Hello SOME/IP: " + std::to_string(count++);
      RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher->publish(message);
    });

  rclcpp::spin(node);
  return 0;
}

int run_listener(int expected)
{
  auto node = std::make_shared<rclcpp::Node>("smoke_listener", minimal_options());

  int received = 0;
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "chatter", 10,
    [&received, expected, node](const std_msgs::msg::String & message) {
      ++received;
      RCLCPP_INFO(node->get_logger(), "I heard: '%s'", message.data.c_str());
      if (received == expected) {
        std::printf(
          "=== SMOKE RESULT: SUCCESS - received %d over SOME/IP ===\n", expected);
        std::fflush(stdout);
        rclcpp::shutdown();
      }
    });

  rclcpp::spin(node);
  return received >= expected ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string role = (argc > 1) ? argv[1] : "";
  const int expected = (argc > 2) ? std::atoi(argv[2]) : 5;

  if (role == "talker") {
    return run_talker();
  }
  if (role == "listener") {
    return run_listener(expected);
  }
  // large_talker/large_listener <bytes> <reliable|best_effort>
  if (role == "large_talker" || role == "large_listener") {
    const int payload = (argc > 2) ? std::atoi(argv[2]) : 8192;
    const bool best_effort = (argc > 3) && std::string(argv[3]) == "best_effort";
    return (role == "large_talker")
      ? run_large_talker(payload, best_effort)
      : run_large_listener(payload, best_effort);
  }
  std::fprintf(stderr, "usage: %s <talker|listener> [expected_count]\n", argv[0]);
  return 2;
}
