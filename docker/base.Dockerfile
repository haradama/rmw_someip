# syntax=docker/dockerfile:1
#
# Base image for developing and running rmw_someip:
#   ROS 2 Jazzy (Ubuntu 24.04) plus vsomeip built from source.
#
# vsomeip is not packaged for Ubuntu, so it always has to be built.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG VSOMEIP_VERSION=3.7.4
ARG ROS_DISTRO=jazzy

# --- Build dependencies, plus what RMW development on ROS 2 needs ------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      ca-certificates \
      pkg-config \
      libboost-system-dev \
      libboost-thread-dev \
      libboost-filesystem-dev \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-rmw \
      ros-${ROS_DISTRO}-rmw-implementation \
      ros-${ROS_DISTRO}-rmw-implementation-cmake \
      ros-${ROS_DISTRO}-rosidl-typesupport-introspection-c \
      ros-${ROS_DISTRO}-rosidl-typesupport-introspection-cpp \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-example-interfaces \
      iproute2 \
      iputils-ping \
      tcpdump \
      net-tools \
      gdb \
      vim \
  && rm -rf /var/lib/apt/lists/*

# --- vsomeip ---------------------------------------------------------------
# ENABLE_SIGNAL_HANDLING must stay off. With it, vsomeip installs its own
# SIGINT and SIGTERM handlers and takes them from the application. An RMW is a
# library and must not intercept an application's signal handling: rclcpp's
# signal_handler and any user handler would stop working, which test_rclcpp's
# test_sigint_handler and test_sigterm_handler detect.
# Teardown is rmw_shutdown and rmw_context_fini's job.
RUN git clone --depth 1 --branch ${VSOMEIP_VERSION} \
        https://github.com/COVESA/vsomeip.git /tmp/vsomeip \
 && cmake -S /tmp/vsomeip -B /tmp/vsomeip/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
 && cmake --build /tmp/vsomeip/build -j"$(nproc)" \
 && cmake --install /tmp/vsomeip/build \
 && ldconfig \
 && rm -rf /tmp/vsomeip

# vsomeip uses UNIX domain sockets under /tmp for local routing.
RUN mkdir -p /tmp/vsomeip && chmod 1777 /tmp

ENV VSOMEIP_VERSION=${VSOMEIP_VERSION}
WORKDIR /ws
