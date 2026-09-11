cmake_minimum_required(VERSION 3.16)
project(robot_base_driver)
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(base_driver_node src/base_driver_node.cpp)
add_executable(velocity_safety_node src/velocity_safety_node.cpp)
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(base_driver_node geometry_msgs nav_msgs rclcpp tf2 tf2_ros)
else()
  target_link_libraries(
    base_driver_node
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
  )
endif()
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(velocity_safety_node geometry_msgs rclcpp)
else()
  target_link_libraries(velocity_safety_node rclcpp::rclcpp ${geometry_msgs_TARGETS})
endif()
target_compile_features(base_driver_node PRIVATE cxx_std_17)
target_compile_options(base_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(velocity_safety_node PRIVATE cxx_std_17)
target_compile_options(velocity_safety_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS base_driver_node velocity_safety_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety test/test_safety.cpp)
  target_include_directories(test_safety PRIVATE include)
  target_link_libraries(test_safety ${geometry_msgs_TARGETS})
endif()

ament_package()
