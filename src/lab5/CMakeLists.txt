cmake_minimum_required(VERSION 3.8)
project(lab5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(simple_map_node 
	src/simple_map_node.cpp
	src/mapper.cpp
)
ament_target_dependencies(simple_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS simple_map_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
