# Copyright 2026 rmw_someip contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The ROS 2 side of the demo, in Python.

**Nothing here depends on SOME/IP.** It is ordinary rclpy code, exactly like the
C++ node in ../ros2_app, and talks to the same native SOME/IP application over
the same wire. The ID mapping and the serialization format are handled by
someip_mapping.yaml and the RMW.

Running this rather than the C++ node is what shows the client library does not
matter: rclpy reaches the RMW through the C introspection typesupport and rclcpp
through the C++ one, and both produce the same AUTOSAR bytes.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from rmw_someip_demo_msgs.msg import VehicleCommand, VehicleStatus


class VehicleBridgeNode(Node):

    def __init__(self):
        # Run without services, as the C++ node does. Extern mode covers pub/sub
        # only, and parameter services need native mode's graph channel; the only
        # peer here is one native SOME/IP application.
        super().__init__(
            'vehicle_node',
            start_parameter_services=False,
            parameter_overrides=[Parameter('start_type_description_service', value=False)],
        )

        self._sent = 0

        # Receive status from the ECU, the native SOME/IP application.
        self._status_subscription = self.create_subscription(
            VehicleStatus, '/vehicle/status', self._on_status, qos_profile_sensor_data)

        # Send commands to the ECU.
        self._command_publisher = self.create_publisher(
            VehicleCommand, '/vehicle/command', qos_profile_sensor_data)

        self._timer = self.create_timer(1.5, self._send_command)

    def _on_status(self, status):
        self.get_logger().info(
            f'RECV /vehicle/status  t={status.timestamp_ms}ms '
            f'speed={status.speed_kph:.1f} steering={status.steering_angle_deg:.1f} '
            f"gear={status.gear} mode='{status.mode}'")

    def _send_command(self):
        command = VehicleCommand()
        command.target_speed_kph = 55.5 + float(self._sent % 5)
        command.gear_request = 3 + (self._sent % 3)
        self.get_logger().info(
            f'SEND /vehicle/command target_speed={command.target_speed_kph:.1f} '
            f'gear_request={command.gear_request}')
        self._command_publisher.publish(command)
        self._sent += 1


def main():
    rclpy.init()
    node = VehicleBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
