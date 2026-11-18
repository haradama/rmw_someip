cmake_minimum_required(VERSION 3.16)
# No ROS dependency at all; a plain vsomeip application.
project(someip_native_app CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(vsomeip3 REQUIRED)
find_package(Threads REQUIRED)
add_executable(someip_ecu main.cpp)
target_link_libraries(someip_ecu PRIVATE vsomeip3 Threads::Threads)
