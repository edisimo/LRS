cmake_minimum_required(VERSION 3.5)
project(drone_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
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
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "srv/CustomPath.srv"
  DEPENDENCIES geometry_msgs mavros_msgs
)

# Export the dependencies for the generated interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()

# Include directories
include_directories(
  include
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Declare and link libraries
add_library(drone_control_lib SHARED src/drone_control.cpp)
ament_target_dependencies(drone_control_lib 
  rclcpp 
  geometry_msgs 
  mavros_msgs 
  tf2 
  tf2_geometry_msgs
)

add_library(path_finding_lib SHARED src/path_finding.cpp)
ament_target_dependencies(path_finding_lib 
  rclcpp
)

# Add dependencies for generated interfaces
add_dependencies(drone_control_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)
add_dependencies(path_finding_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Add dependencies for generated interfaces to executables
add_executable(drone_control_node nodes/drone_control_node.cpp)
target_link_libraries(drone_control_node drone_control_lib)
ament_target_dependencies(drone_control_node 
  rclcpp 
  geometry_msgs 
  mavros_msgs 
  tf2 
  tf2_geometry_msgs
)

add_executable(path_finding_node nodes/path_finding_node.cpp)
target_link_libraries(path_finding_node path_finding_lib)
ament_target_dependencies(path_finding_node 
  rclcpp
)

# Make sure the generated code is linked properly
target_link_libraries(drone_control_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(path_finding_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(drone_control_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(path_finding_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install targets
install(TARGETS
  drone_control_lib
  path_finding_lib
  drone_control_node
  path_finding_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
