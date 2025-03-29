cmake_minimum_required(VERSION 3.8)
project(simple_grasping_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
    "msg/Furniture.msg"
    "msg/Grasp.msg"
    "msg/Object.msg"
    "msg/Plane.msg"
)

set(service_files
    "srv/AddFrame.srv"
    "srv/StartPerception.srv"
    "srv/GenerateGrasps.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${service_files}
  DEPENDENCIES geometry_msgs visualization_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)


ament_package()