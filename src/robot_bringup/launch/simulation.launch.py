from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_base_driver', executable='base_driver_node', name='base_driver', parameters=['/ws/src/robot_bringup/config/robot.yaml'], output='screen'),
        Node(package='robot_navigation', executable='planner_node', name='planner', output='screen'),
        Node(package='robot_tasks', executable='task_manager_node', name='task_manager', output='screen'),
    ])
