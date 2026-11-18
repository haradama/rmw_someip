cmake_minimum_required(VERSION 3.16)
project(rmw_someip_spike_dynamic_id CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(vsomeip3 REQUIRED)
find_package(Threads REQUIRED)

foreach(target provider consumer)
  add_executable(${target} ${target}.cpp)
  target_link_libraries(${target} PRIVATE vsomeip3 Threads::Threads)
endforeach()
