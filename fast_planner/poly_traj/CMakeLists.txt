cmake_minimum_required(VERSION 3.5)
project(poly_traj)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include
${Eigen3_INCLUDE_DIRS} )

add_library( poly_traj 
    src/polynomial_traj.cpp 
)

# Link libraries
ament_target_dependencies(poly_traj
  rclcpp
  std_msgs
)

target_link_libraries(poly_traj 
${Eigen3_LIBRARIES})


# Install library
install(TARGETS poly_traj
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # Install headers
install(DIRECTORY include/
  DESTINATION include/
)
# Export package information
ament_export_include_directories(include)
#ament_export_libraries(poly_traj)

ament_package()
