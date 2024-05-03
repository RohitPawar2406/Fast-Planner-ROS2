cmake_minimum_required(VERSION 3.5)
project(traj_utils)

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
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(plan_env REQUIRED)

include_directories( 
    include 
    include/bspline
    include/path_searching
    include/plan_env
    include/poly_traj
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${bspline_INCLUDE_DIRS}
    ${path_searching_INCLUDE_DIRS}
    ${plan_env_INCLUDE_DIRS}
    ${poly_traj_INCLUDE_DIRS}
)

add_library( traj_utils
    src/planning_visualization.cpp 
  )

ament_target_dependencies(traj_utils
    rclcpp
    rclpy
    std_msgs
    bspline
    path_searching
    plan_env
    poly_traj
  ) 

target_link_libraries(traj_utils
  ${PCL_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION include/)  

install(TARGETS traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(traj_utils)
ament_export_include_directories(include)
ament_package()
