cmake_minimum_required(VERSION 3.16)
project(map_loader LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/LoadMap.srv
  DEPENDENCIES std_msgs
)

add_executable(map_loader_node src/map_loader_node.cpp)
ament_target_dependencies(map_loader_node
  rclcpp
  std_msgs
  PCL
  octomap_msgs
)
target_link_libraries(map_loader_node ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS map_loader_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_loader test/test_map_loader.cpp)
  ament_target_dependencies(test_map_loader rclcpp std_msgs PCL octomap_msgs)
  target_link_libraries(test_map_loader ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
endif()

ament_package()
