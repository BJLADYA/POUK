cmake_minimum_required(VERSION 3.8)
project(lab4)

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

add_executable(control_selector_node
	src/control_selector_node.cpp
	src/control_selector.cpp
	src/dummy_control.cpp
	src/voyager_control.cpp
	src/wallfollower_control.cpp 
)
ament_target_dependencies(control_selector_node
	rclcpp
	std_msgs
	sensor_msgs
	nav_msgs
	geometry_msgs
)

add_executable(selector_node
	src/selector_node.cpp 
)
ament_target_dependencies(selector_node
	rclcpp
	std_msgs
)

install(TARGETS
	control_selector_node
	selector_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
