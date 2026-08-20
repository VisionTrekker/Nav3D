cmake_minimum_required(VERSION 3.16)
project(goal_marker_server LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
add_executable(goal_marker_node src/goal_marker_node.cpp)
ament_target_dependencies(goal_marker_node
  rclcpp
  visualization_msgs
  interactive_markers
)

install(TARGETS goal_marker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
