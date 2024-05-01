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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include
${Eigen3_INCLUDE_DIRS} 
${PCL_INCLUDE_DIRS})  


# rosidl_generate_interfaces(${PROJECT_NAME}

#   DEPENDENCIES
#   nav_msgs
#   # Add more message files as needed
# )
# ament_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan_manage
#   #DEPENDS plan_env path_searching bspline bspline_opt traj_utils message_runtime
# #  DEPENDS system_lib
# )

add_executable(fast_planner_node
  src/fast_planner_node.cpp 
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
  )

target_link_libraries(fast_planner_node
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
ament_target_dependencies(fast_planner_node           ######### ERROR
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server 
${PCL_LIBRARIES}
${Eigen3_LIBRARIES})
ament_target_dependencies(traj_server           ######### ERROR
  rclcpp
  std_msgs
  nav_msgs
  visualization_msgs)
# # add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
# # CHANGED TO : 
# ament_target_dependencies(traj_server
#   rclcpp
#   std_msgs
#   nav_msgs) 

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include)          # this line works
  
install(TARGETS
  fast_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

######################## NOTES 
# Invoke ament_package instead but after all targets have been registered.

# The only valid argument for ament_package is CONFIG_EXTRAS. All other arguments are covered by separate functions which all need to be invoked before ament_package:
#     Instead of passing CATKIN_DEPENDS ... call ament_export_dependencies(...) before.
#     Instead of passing INCLUDE_DIRS ... call ament_export_include_directories(...) before.
#     Instead of passing LIBRARIES ... call ament_export_libraries(...) before.


