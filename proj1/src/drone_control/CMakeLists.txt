cmake_minimum_required(VERSION 3.5)
project(drone_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories for tf2 and tf2_geometry_msgs
include_directories(
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  include
)

# Declare libraries
add_library(drone_control_lib src/drone_control.cpp)
add_library(path_finding_lib src/path_finding.cpp)

# Link libraries to dependencies using ament_target_dependencies
ament_target_dependencies(drone_control_lib rclcpp geometry_msgs mavros_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(path_finding_lib rclcpp)

# Declare executables
add_executable(drone_control_node nodes/drone_control_node.cpp)
add_executable(path_finding_node nodes/path_finding_node.cpp)

# Link executables to libraries using ament_target_dependencies (automatically handles linking)
ament_target_dependencies(drone_control_node rclcpp geometry_msgs mavros_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(path_finding_node rclcpp)

# Link libraries to executables
target_link_libraries(drone_control_node drone_control_lib)
target_link_libraries(path_finding_node path_finding_lib)

# Install the library and executable
install(TARGETS drone_control_lib drone_control_node path_finding_lib path_finding_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install include files
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Ament package
ament_package()
