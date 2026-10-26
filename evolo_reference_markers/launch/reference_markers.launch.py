"""Start selected reference marker nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    # Before params_file, so a key in the node's own YAML section overrides it.
    derived = {
        "world_frame": LaunchConfiguration("world_frame"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="ROS parameter YAML.",
            ),
            DeclareLaunchArgument("world_frame", default_value="evolo/map"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "start_fixed_marker",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "start_smarcduino_marker",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "start_waraps_marker",
                default_value="true",
            ),
            Node(
                package="evolo_reference_markers",
                executable="fixed_position_marker_node",
                name="fixed_position_marker_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_fixed_marker")
                ),
                parameters=[derived, params_file],
            ),
            Node(
                package="evolo_reference_markers",
                executable="smarcduino_marker_node",
                name="smarcduino_position_marker_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_smarcduino_marker")
                ),
                parameters=[derived, params_file],
            ),
            Node(
                package="evolo_reference_markers",
                executable="smarcduino_waraps_position_marker_node",
                name="smarcduino_waraps_position_marker_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_waraps_marker")
                ),
                parameters=[derived, params_file],
            ),
        ]
    )
