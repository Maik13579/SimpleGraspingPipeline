cmake_minimum_required(VERSION 3.8)
project(simple_grasping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(simple_grasping_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search segmentation)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(visualization_msgs REQUIRED)

# Define the traversability_estimation_node executable
add_executable(simple_grasping_node
  src/main.cpp
  src/node.cpp
  src/params.cpp
)

# Add include directories
target_include_directories(simple_grasping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Link against PCL libraries
target_link_libraries(simple_grasping_node ${PCL_LIBRARIES})

# Add PCL definitions
target_compile_definitions(simple_grasping_node PUBLIC ${PCL_DEFINITIONS} -DPCL_NO_PRECOMPILE)


# Add dependencies for traversability_estimation_node
ament_target_dependencies(simple_grasping_node
  simple_grasping_interfaces
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  geometry_msgs
)

# Set C++ standard
set_target_properties(simple_grasping_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)

# Link OpenMP to both nodes
if(OpenMP_CXX_FOUND)
  target_link_libraries(simple_grasping_node OpenMP::OpenMP_CXX)
  target_compile_definitions(simple_grasping_node PUBLIC -DUSE_OPENMP)
endif()

# Install targets
install(TARGETS simple_grasping_node
  DESTINATION lib/${PROJECT_NAME})

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
  # Skip specific linter if necessary
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()