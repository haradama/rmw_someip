cmake_minimum_required(VERSION 3.16)
project(rmw_someip_demo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleStatus.msg"
  "msg/VehicleCommand.msg"
)

ament_package()
