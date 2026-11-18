cmake_minimum_required(VERSION 3.16)
project(rmw_someip_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Coverage instrumentation.
#
# The official suites dlopen the .so, so the library itself has to be built
# instrumented; adding the flags to a test binary achieves nothing. Optimization
# is off because inlining removes the lines that are being counted.
option(RMW_SOMEIP_COVERAGE "build with gcov instrumentation" OFF)
if(RMW_SOMEIP_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw REQUIRED)
# BEST_AVAILABLE QoS resolution; shared and not DDS-specific.
find_package(rmw_dds_common REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# vsomeip is not an ament package, so it is found as a plain CMake package.
find_package(vsomeip3 REQUIRED)
# Used to read the extern-mode mapping file (someip_mapping.yaml).
find_package(yaml-cpp REQUIRED)

add_library(rmw_someip_cpp SHARED
  src/identifier.cpp
  src/detail/config_generator.cpp
  src/detail/participant.cpp
  src/detail/context_impl.cpp
  src/detail/extern_mapping.cpp
  src/detail/graph_cache.cpp
  src/detail/graph_message.cpp
  src/detail/type_support.cpp
  src/detail/someip_serializer.cpp
  src/detail/someip_session.cpp
  src/rmw_client.cpp
  src/rmw_event.cpp
  src/rmw_graph.cpp
  src/rmw_init.cpp
  src/rmw_misc.cpp
  src/rmw_node.cpp
  src/rmw_publish.cpp
  src/rmw_serialize.cpp
  src/rmw_service.cpp
  src/rmw_subscribe.cpp
  src/rmw_wait.cpp
)

target_include_directories(rmw_someip_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Lets detail/*.hpp be included as "detail/..."; internal only.
target_include_directories(rmw_someip_cpp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(rmw_someip_cpp PUBLIC
  rmw::rmw
  rmw_dds_common::rmw_dds_common_library
  rcutils::rcutils
  rcpputils::rcpputils
  rosidl_runtime_c::rosidl_runtime_c
  rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
)
target_link_libraries(rmw_someip_cpp PRIVATE vsomeip3 yaml-cpp::yaml-cpp)

# Register as an RMW implementation.
#
# The arguments declare which typesupports this implementation handles, and
# rosidl resolves them in the order given. The CDR serializer is built on
# introspection, so the introspection typesupports are named, as
# rmw_cyclonedds_cpp does.
register_rmw_implementation(
  "c:rosidl_typesupport_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_introspection_cpp"
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rmw_someip_cpp
  EXPORT rmw_someip_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(rmw_someip_cpp)
ament_export_dependencies(
  rcutils
  rcpputils
  rmw
  rmw_dds_common
  rosidl_runtime_c
  rosidl_typesupport_introspection_c
  rosidl_typesupport_introspection_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
