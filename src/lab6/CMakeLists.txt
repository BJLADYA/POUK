cmake_minimum_required(VERSION 3.8)
project(lab6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)


add_executable(patrol_bot_node 
	src/patrol_bot_node.cpp
	src/patrol_bot.cpp
)
ament_target_dependencies(patrol_bot_node
	rclcpp
	rclcpp_action
	sensor_msgs
	nav_msgs
	geometry_msgs
	tf2_ros
	tf2_geometry_msgs
	nav2_msgs
)

install(TARGETS patrol_bot_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
	config
	maps
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
