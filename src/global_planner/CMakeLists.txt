cmake_minimum_required(VERSION 3.10)

project(global_planner LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(global_planner_node
    src/global_planner_node.cpp
    src/global_planner.cpp
    src/pcd2octomap_converter.cpp
)
target_include_directories(global_planner_node
    PRIVATE
        include
        ${PCL_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(global_planner_node
    rclcpp
    geometry_msgs
    nav_msgs
    octomap_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)
target_link_libraries(global_planner_node
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)

install(TARGETS global_planner_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
