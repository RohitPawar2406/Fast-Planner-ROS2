cmake_minimum_required(VERSION 3.5)
project(path_searching)

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
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
include(ConfigExtras.cmake)
include_directories( 
    include
    include/plan_env
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${plan_env_INCLUDE_DIRS}
)

add_library( path_searching 
    src/kinodynamic_astar.cpp 
    src/astar.cpp
    src/topo_prm.cpp
)

ament_target_dependencies(path_searching 
rclcpp
rclpy
std_msgs
plan_env

) 

target_link_libraries( path_searching
${plan_env_LIBRARIES}
rclcpp
plan_env
) 

install(TARGETS path_searching
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(DIRECTORY include/
DESTINATION include/) 

ament_export_include_directories(include)
ament_package()
