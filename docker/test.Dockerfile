# syntax=docker/dockerfile:1
#
# Adds what the official ROS 2 test suites (test_rmw_implementation,
# test_communication) need on top of the base image.
#
# The test packages themselves are not distributed through apt and have to be
# fetched from source, but their dependencies are available.
FROM rmw_someip/base:jazzy

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-osrf-testing-tools-cpp \
      ros-${ROS_DISTRO}-test-msgs \
      ros-${ROS_DISTRO}-ament-cmake-gtest \
      ros-${ROS_DISTRO}-ament-cmake-pytest \
      ros-${ROS_DISTRO}-ament-lint-auto \
      ros-${ROS_DISTRO}-ament-lint-common \
      ros-${ROS_DISTRO}-launch-testing \
      ros-${ROS_DISTRO}-launch-testing-ament-cmake \
      ros-${ROS_DISTRO}-launch-testing-ros \
      ros-${ROS_DISTRO}-rclpy \
      ros-${ROS_DISTRO}-rosidl-default-generators \
      python3-pytest \
      lcov \
      gcovr \
  && rm -rf /var/lib/apt/lists/*
