cmake_minimum_required(VERSION 3.5)
project(bspline)

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

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(plan_env REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(include
${Eigen3_INCLUDE_DIRS} 
${PCL_INCLUDE_DIRS}
)  

add_library(bspline 
src/non_uniform_bspline.cpp )

ament_target_dependencies(bspline 
  rclcpp
  rclpy
  std_msgs
  nav_msgs
  visualization_msgs
)

target_link_libraries(bspline 
${PCL_LIBRARIES}
${Eigen3_LIBRARIES})


install(TARGETS bspline
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/)  

ament_export_targets(export_${PROJECT_NAME} )
ament_export_include_directories(include)
ament_package()
