ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE} AS base

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
# Make all member variables public to overwrite them later
# Ugly ^^ but works
RUN sed -i '/^[[:space:]]*private:/d' /opt/gpd/include/gpd/grasp_detector.h
WORKDIR /opt/gpd
RUN mkdir build && cd build && \
    cmake .. && make -j4

# Install Pointcloud Server deps
RUN apt update && apt install -y \
    libnanoflann-dev \
    && rm -rf /var/lib/apt/lists/*

 # Install dependencies
COPY pointcloud_server/pointcloud_server/package.xml ${ROS_WS}/src/pointcloud_server/pointcloud_server/package.xml
COPY pointcloud_server/pointcloud_server_interfaces/package.xml ${ROS_WS}/src/pointcloud_server/pointcloud_server_interfaces/package.xml
COPY simple_grasping_interfaces/package.xml ${ROS_WS}/src/simple_grasping/simple_grasping_interfaces/package.xml 
COPY simple_grasping_perception/package.xml ${ROS_WS}/src/simple_grasping/simple_grasping_perception/package.xml
COPY simple_grasping_world/package.xml ${ROS_WS}/src/simple_grasping/simple_grasping_world/package.xml
RUN apt update \
 && rosdep update \
 && rosdep install --from-paths ${ROS_WS}/src --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/*

# Build
WORKDIR ${ROS_WS}
COPY pointcloud_server/pointcloud_server ${ROS_WS}/src/pointcloud_server/pointcloud_server
COPY pointcloud_server/pointcloud_server_interfaces ${ROS_WS}/src/pointcloud_server/pointcloud_server_interfaces 
COPY simple_grasping_interfaces ${ROS_WS}/src/simple_grasping/simple_grasping_interfaces
COPY simple_grasping_perception ${ROS_WS}/src/simple_grasping/simple_grasping_perception
COPY simple_grasping_world ${ROS_WS}/src/simple_grasping/simple_grasping_world
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && MAKEFLAGS="-j2" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
RUN echo "source ${ROS_WS}/install/setup.bash" >> /root/.bashrc
