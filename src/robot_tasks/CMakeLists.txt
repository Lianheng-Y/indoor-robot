cmake_minimum_required(VERSION 3.16)
project(robot_tasks)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(task_manager_node src/task_manager_node.cpp)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(
    task_manager_node geometry_msgs rclcpp rclcpp_action robot_interfaces std_msgs sensor_msgs)
else()
  target_link_libraries(
    task_manager_node
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${geometry_msgs_TARGETS}
    ${robot_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
  )
endif()
target_compile_features(task_manager_node PRIVATE cxx_std_17)
target_compile_options(task_manager_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS task_manager_node DESTINATION lib/${PROJECT_NAME})

ament_package()
