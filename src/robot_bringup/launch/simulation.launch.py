from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    description_share = Path(get_package_share_directory("robot_description"))
    robot_description = (description_share / "urdf" / "robot.urdf").read_text(
        encoding="utf-8"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_start = LaunchConfiguration("auto_start")
    parameters = str(bringup_share / "config" / "robot.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use an external simulation clock",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="Start the configured patrol after launch",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
                output="screen",
            ),
            Node(
                package="robot_base_driver",
                executable="base_driver_node",
                name="base_driver",
                parameters=[
                    parameters,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                package="robot_navigation",
                executable="planner_node",
                name="planner",
                parameters=[
                    parameters,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                package="robot_tasks",
                executable="task_manager_node",
                name="task_manager",
                parameters=[
                    parameters,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "auto_start": ParameterValue(auto_start, value_type=bool),
                    },
                ],
                output="screen",
            ),
        ]
    )
