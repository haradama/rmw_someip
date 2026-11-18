// The ROS 2 side of the demo.
//
// Nothing here depends on SOME/IP. It is ordinary rclcpp code, and the
// source gives no sign that the traffic runs over SOME/IP. The ID mapping and
// the serialization format are handled by someip_mapping.yaml and the RMW,
// which is the difference from a bridge node.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmw_someip_demo_msgs/msg/vehicle_command.hpp"
#include "rmw_someip_demo_msgs/msg/vehicle_status.hpp"

namespace
{

using rmw_someip_demo_msgs::msg::VehicleCommand;
using rmw_someip_demo_msgs::msg::VehicleStatus;

class VehicleBridgeNode : public rclcpp::Node
{
public:
  VehicleBridgeNode()
  : Node("vehicle_node", options())
  {
    // Receive status from the ECU, the native SOME/IP application.
    status_subscription_ = create_subscription<VehicleStatus>(
      "/vehicle/status", rclcpp::SensorDataQoS(),
      [this](const VehicleStatus & status) {
        ++received_;
        RCLCPP_INFO(
          get_logger(),
          "RECV /vehicle/status  t=%ums speed=%.1f steering=%.1f gear=%u mode='%s'",
          status.timestamp_ms, status.speed_kph, status.steering_angle_deg,
          status.gear, status.mode.c_str());
      });

    // Send commands to the ECU.
    command_publisher_ = create_publisher<VehicleCommand>(
      "/vehicle/command", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1500),
      [this]() {
        VehicleCommand command;
        command.target_speed_kph = 55.5F + static_cast<float>(sent_ % 5);
        command.gear_request = static_cast<uint8_t>(3 + (sent_ % 3));
        RCLCPP_INFO(
          get_logger(), "SEND /vehicle/command target_speed=%.1f gear_request=%u",
          command.target_speed_kph, command.gear_request);
        command_publisher_->publish(command);
        ++sent_;
      });
  }

private:
  /// Run without services. Extern mode covers pub/sub only, and parameter
  /// services need native mode's graph channel. The only peer here is one
  /// native SOME/IP application, so there is nothing for them to talk to.
  static rclcpp::NodeOptions options()
  {
    rclcpp::NodeOptions node_options;
    node_options.start_parameter_services(false);
    node_options.start_parameter_event_publisher(false);
    node_options.append_parameter_override("start_type_description_service", false);
    return node_options;
  }

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int received_ {0};
  int sent_ {0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
