cmake_minimum_required(VERSION 3.5)
project(plan_env)

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
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(fast_planner REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(plan_env SHARED
    src/sdf_map.cpp 
    # src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
  )

add_library( test_plan_env_sdf SHARED
  src/test_plan_env_sdf.cpp
)
target_include_directories(plan_env PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_include_directories(test_plan_env_sdf PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


add_executable(obj_generator 
    src/obj_generator.cpp 
)

add_executable(test_plan_env 
  src/test_plan_env.cpp
)

target_link_libraries(test_plan_env
  # plan_env
  test_plan_env_sdf
)

install(TARGETS test_plan_env
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_target_dependencies(plan_env 
  rclcpp
  rclpy
  std_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs
  cv_bridge
  message_filters
  fast_planner
)

ament_target_dependencies(test_plan_env_sdf
  rclcpp
  std_msgs
)


ament_target_dependencies(obj_generator rclcpp geometry_msgs visualization_msgs nav_msgs)

target_link_libraries(obj_generator 
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
target_link_libraries(plan_env  
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${OpenCV_LIBS}
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS plan_env
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    # RUNTIME DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  ) 

install(
    TARGETS test_plan_env_sdf
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    # RUNTIME DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  ) 

ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_export_libraries(test_plan_env_sdf)
ament_export_dependencies(
    # rclcpp
    # std_msgs
    # visualization_msgs
    fast_planner
    message_filters
    Eigen3
    PCL
    OpenCV
    # nav_msgs
    # plan_env
)
# ament_export_include_directories(include)
ament_package()
