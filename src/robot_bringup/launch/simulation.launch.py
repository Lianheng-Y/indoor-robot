from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='robot_base_driver', executable='base_driver_node', name='base_driver', parameters=[{'max_speed': 0.6}], output='screen')])
