cmake_minimum_required(VERSION 3.8)
project(lab3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(line_control_node
  src/line_control_node.cpp
  src/line_control.cpp)
ament_target_dependencies(line_control_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  line_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
