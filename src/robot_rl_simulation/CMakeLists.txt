cmake_minimum_required(VERSION 3.16)
project(robot_rl_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(a1_body_state_plugin SHARED gazebo_plugins/a1_body_state_plugin.cpp)
target_include_directories(a1_body_state_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(a1_body_state_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(a1_body_state_plugin gazebo_ros nav_msgs rclcpp)

add_library(robot_rl_simulation_logic INTERFACE)
target_include_directories(robot_rl_simulation_logic INTERFACE include)

install(TARGETS a1_body_state_plugin
  LIBRARY DESTINATION lib)

install(DIRECTORY include config launch worlds models description policies
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_filter test/test_safety_filter.cpp)
  target_link_libraries(test_safety_filter robot_rl_simulation_logic)
endif()

ament_package()
