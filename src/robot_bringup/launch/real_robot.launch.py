from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("robot_bringup"))
    description_share = Path(get_package_share_directory("robot_description"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    robot_description = ParameterValue(
        Command([
            "xacro ", str(description_share / "urdf" / "robot.xacro"),
            " hardware_plugin:=", hardware_plugin,
            " transport:=", LaunchConfiguration("transport"),
            " serial_port:=", LaunchConfiguration("serial_port"),
            " baud_rate:=", LaunchConfiguration("baud_rate"),
            " can_interface:=", LaunchConfiguration("can_interface"),
            " encoder_cpr:=", LaunchConfiguration("encoder_cpr"),
            " gear_ratio:=", LaunchConfiguration("gear_ratio"),
        ]),
        value_type=str,
    )
    control_params = str(bringup_share / "config" / "ros2_control.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="mock_components/GenericSystem",
                description="ros2_control plugin; set this to the serial/CAN driver plugin on the robot",
            ),
            DeclareLaunchArgument("transport", default_value="serial"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("encoder_cpr", default_value="4096"),
            DeclareLaunchArgument("gear_ratio", default_value="1.0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description,
                     "use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
                output="screen",
            ),
            Node(
                package="robot_base_driver",
                executable="velocity_safety_node",
                name="velocity_safety",
                parameters=[
                    str(bringup_share / "config" / "robot.yaml"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                parameters=[control_params, {"robot_description": robot_description}],
                remappings=[
                    ("~/robot_description", "/robot_description"),
                    ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel_safe"),
                    ("/diff_drive_controller/odom", "/odom"),
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ]
    )
