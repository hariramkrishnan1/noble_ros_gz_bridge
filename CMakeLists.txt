cmake_minimum_required(VERSION 3.10)
project(noble_ros_gz_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)

include_directories(include)

add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node rclcpp std_msgs ros_gz_bridge)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
