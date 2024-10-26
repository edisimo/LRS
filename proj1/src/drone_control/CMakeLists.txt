cmake_minimum_required(VERSION 3.5)
project(drone_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Include directory for headers
include_directories(include)

# Declare a C++ library
add_library(drone_control_lib src/drone_control.cpp)
add_library(path_finding_lib src/path_finding.cpp)


# Declare a C++ executable
add_executable(drone_control_node nodes/drone_control_node.cpp)
add_executable(path_finding_node nodes/path_finding_node.cpp)


# Specify libraries to link a library or executable target against
ament_target_dependencies(drone_control_lib rclcpp geometry_msgs mavros_msgs)
ament_target_dependencies(drone_control_node rclcpp geometry_msgs mavros_msgs)
ament_target_dependencies(path_finding_lib rclcpp)
ament_target_dependencies(path_finding_node rclcpp)

# Link the library to the executable
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
