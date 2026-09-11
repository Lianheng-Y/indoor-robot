cmake_minimum_required(VERSION 3.16)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(planner_node src/planner_node.cpp)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(
    planner_node geometry_msgs nav_msgs rclcpp rclcpp_action robot_interfaces)
else()
  target_link_libraries(
    planner_node
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${robot_interfaces_TARGETS}
  )
endif()
target_compile_features(planner_node PRIVATE cxx_std_17)
target_compile_options(planner_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})

ament_package()
