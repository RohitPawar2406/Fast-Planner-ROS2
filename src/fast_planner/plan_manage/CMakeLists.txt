cmake_minimum_required(VERSION 3.5)
project(plan_manage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(NLopt REQUIRED)

include_directories(include
  include/bspline
  include/bspline_opt
  include/path_searching
  include/plan_env
  include/poly_traj
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${bspline_INCLUDE_DIRS}
  ${bspline_opt_INCLUDE_DIRS}
)  


add_executable(fast_planner_node  
  src/planner_manager.cpp
)

ament_target_dependencies(fast_planner_node 
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt

)

target_link_libraries(fast_planner_node
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  NLopt::nlopt
)

install(TARGETS fast_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include)          # this line works

ament_package()

