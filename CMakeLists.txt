cmake_minimum_required(VERSION 3.8)
project(rtabmap_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rtabmap_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(slam_toolbox REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  rviz_configs
  urdf
  pushcart_mesh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
