ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE} as base

# Environment variables
ENV ROS_WS=/root/ros2_ws
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Use bash
SHELL ["/bin/bash", "-c"]

# Source ROS entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Install Cyclone DDS
RUN apt update \
 && apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Install GPD https://github.com/atenpas/gpd
RUN apt update && apt install -y \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
 && rm -rf /var/lib/apt/lists/*
COPY gpd /opt/gpd
WORKDIR /opt/gpd
RUN mkdir build && cd build && \
    cmake .. && make -j4

 # Install dependencies
COPY simple_grasping/package.xml ${ROS_WS}/src/simple_grasping/package.xml
COPY simple_grasping_interfaces/package.xml ${ROS_WS}/src/simple_grasping_interfaces/package.xml 
RUN apt update \
 && rosdep update \
 && rosdep install --from-paths ${ROS_WS}/src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*

# Build
WORKDIR ${ROS_WS}
COPY simple_grasping ${ROS_WS}/src/simple_grasping
COPY simple_grasping_interfaces ${ROS_WS}/src/simple_grasping_interfaces
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc
