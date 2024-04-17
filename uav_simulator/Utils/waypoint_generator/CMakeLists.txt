cmake_minimum_required(VERSION 3.5)
project(waypoint_generator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(Eigen3 REQUIRED)
include_directories(include)

add_executable(waypoint_generator src/waypoint_generator.cpp)
target_link_libraries(waypoint_generator)

ament_target_dependencies(waypoint_generator
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

ament_package()
