from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    description_share = Path(get_package_share_directory("robot_description"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    robot_description = (description_share / "urdf" / "robot.urdf").read_text(
        encoding="utf-8"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "map", default_value=str(bringup_share / "maps" / "demo_map.yaml")
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(bringup_share / "config" / "nav2_params.yaml"),
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
                    str(bringup_share / "config" / "robot.yaml"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_share / "launch" / "bringup_launch.py")),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
