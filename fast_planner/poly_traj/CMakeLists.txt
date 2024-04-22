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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rclcpp)
find_package(std_msgs)

message("=========")
message(${PROJECT_SOURCE_DIR}/include)
include_directories(
   ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_library( poly_traj 
    src/polynomial_traj.cpp 
    )

target_link_libraries( poly_traj)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
    TARGETS poly_traj
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  ) 

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(poly_traj)
ament_package()
