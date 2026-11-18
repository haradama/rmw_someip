cmake_minimum_required(VERSION 3.16)
project(cdr_compat_test CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rmw REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(test_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
add_executable(cdr_compat_test main.cpp)
target_link_libraries(cdr_compat_test
  rmw::rmw rcutils::rcutils
  ${std_msgs_TARGETS} ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${test_msgs_TARGETS}
  rosidl_typesupport_cpp::rosidl_typesupport_cpp)
install(TARGETS cdr_compat_test DESTINATION lib/${PROJECT_NAME})
ament_package()
