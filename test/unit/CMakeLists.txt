cmake_minimum_required(VERSION 3.16)
project(rmw_someip_unit_tests CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_dds_common REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(test_msgs REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(vsomeip3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# The RMW sources are compiled straight into the test binary.
#
# Linking the installed .so instead would leave the internals of detail/ out of
# reach: anonymous namespaces, header-only classes. Most of the failure paths —
# malformed input, allocation failure, expiry — cannot be provoked from outside,
# and reaching them is what raises line coverage.
set(RMW_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../rmw_someip_cpp/src)

option(RMW_SOMEIP_COVERAGE "build with gcov instrumentation" OFF)
if(RMW_SOMEIP_COVERAGE)
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

if(BUILD_TESTING)
  ament_add_gtest(unit_tests
    main.cpp
    test_cdr.cpp
    test_graph_message.cpp
    test_graph_cache.cpp
    test_event_data.cpp
    test_wait_set.cpp
    test_message_pool.cpp
    test_rmw_checks.cpp
    test_participant.cpp
    test_config_generator.cpp
    test_context_impl.cpp
    test_entity_data.cpp
    test_qos_compat.cpp
    test_extern_mapping.cpp
    test_serialization.cpp
    test_synthetic_members.cpp
    ${RMW_SRC}/detail/config_generator.cpp
    ${RMW_SRC}/detail/context_impl.cpp
    ${RMW_SRC}/detail/extern_mapping.cpp
    ${RMW_SRC}/detail/graph_cache.cpp
    ${RMW_SRC}/detail/graph_message.cpp
    ${RMW_SRC}/detail/participant.cpp
    ${RMW_SRC}/detail/someip_serializer.cpp
    ${RMW_SRC}/detail/type_support.cpp
    ${RMW_SRC}/identifier.cpp
  )
  target_include_directories(unit_tests PRIVATE
    ${RMW_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../rmw_someip_cpp/include
  )
  target_link_libraries(unit_tests
    rmw::rmw
    rmw_dds_common::rmw_dds_common_library
    rcutils::rcutils
    rcpputils::rcpputils
    rosidl_runtime_c::rosidl_runtime_c
    rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
    ${test_msgs_TARGETS}
    rosidl_typesupport_c::rosidl_typesupport_c
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    vsomeip3
    yaml-cpp::yaml-cpp
  )
  # Tests against the public rmw API link the installed .so instead:
  # compiling the sources in would define the C API symbols twice.
  find_package(rmw_someip_cpp REQUIRED)
  # alloc_hooks.cpp substitutes allocation functions. Symbols defined in the
  # executable resolve ahead of a shared library's, so allocations inside
  # librmw are caught too.
  ament_add_gtest(rmw_api_tests
    main.cpp
    alloc_hooks.cpp
    test_rmw_api.cpp
    test_alloc_failures.cpp
  )
  target_link_libraries(rmw_api_tests
    ${rmw_someip_cpp_LIBRARIES}
    rmw::rmw
    rcutils::rcutils
    ${test_msgs_TARGETS}
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
  )
endif()

ament_package()
