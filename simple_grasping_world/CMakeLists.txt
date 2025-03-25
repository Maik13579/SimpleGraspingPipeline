cmake_minimum_required(VERSION 3.8)
project(simple_grasping_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(simple_grasping_interfaces REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search segmentation)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)



# Build the component
add_library(simple_grasping_world_component SHARED
  src/node.cpp
  src/params.cpp
)

target_include_directories(simple_grasping_world_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(simple_grasping_world_component ${PCL_LIBRARIES} yaml-cpp)

target_compile_definitions(simple_grasping_world_component PUBLIC ${PCL_DEFINITIONS} -DPCL_NO_PRECOMPILE)

ament_target_dependencies(simple_grasping_world_component
  simple_grasping_interfaces
  composition_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  geometry_msgs
  visualization_msgs
  yaml-cpp
)

rclcpp_components_register_nodes(simple_grasping_world_component "SimpleGraspingWorldNode")

install(TARGETS simple_grasping_world_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install launch and params directories
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
