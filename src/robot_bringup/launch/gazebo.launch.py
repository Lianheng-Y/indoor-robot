from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    description_share = Path(get_package_share_directory("robot_description"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    gz_share = Path(get_package_share_directory("ros_gz_sim"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    robot_description = xacro.process_file(
        str(description_share / "urdf" / "robot.xacro")
    ).toxml()

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=str(description_share / "worlds" / "indoor_world.sdf"),
            ),
            DeclareLaunchArgument(
                "map", default_value=str(bringup_share / "maps" / "demo_map.yaml")
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(bringup_share / "config" / "nav2_params.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
                launch_arguments={"gz_args": ["-r -v 3 ", LaunchConfiguration("world")]}.items(),
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
                package="ros_gz_sim",
                executable="create",
                name="spawn_robot",
                arguments=["-topic", "robot_description", "-name", "indoor_robot"],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                ],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
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
