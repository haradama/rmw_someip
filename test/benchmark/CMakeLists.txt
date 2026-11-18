cmake_minimum_required(VERSION 3.16)
project(rmw_someip_benchmark)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Payload.msg")

foreach(target bench_pub bench_sub)
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} rclcpp::rclcpp)
  # The type is generated in this package, so link its typesupport directly.
  rosidl_get_typesupport_target(cpp_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(${target} "${cpp_typesupport}")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
