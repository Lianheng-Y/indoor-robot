cmake_minimum_required(VERSION 3.16)
project(robot_navigation)
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)

add_executable(point_controller_node src/point_controller_node.cpp)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(
    point_controller_node geometry_msgs nav_msgs rclcpp rclcpp_action robot_interfaces tf2)
else()
  target_link_libraries(
    point_controller_node
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${robot_interfaces_TARGETS}
    tf2::tf2
  )
endif()
target_compile_features(point_controller_node PRIVATE cxx_std_17)
target_compile_options(point_controller_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS point_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_math test/test_controller_math.cpp)
  target_include_directories(test_controller_math PRIVATE include)
endif()

ament_package()
