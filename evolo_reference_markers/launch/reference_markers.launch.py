"""Start selected reference marker nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="ROS parameter YAML.",
            ),
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
            DeclareLaunchArgument(
                "start_lidar_marker",
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
                parameters=[params_file],
            ),
            Node(
                package="evolo_reference_markers",
                executable="smarcduino_marker_node",
                name="smarcduino_position_marker_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_smarcduino_marker")
                ),
                parameters=[params_file],
            ),
            Node(
                package="evolo_reference_markers",
                executable="smarcduino_waraps_position_marker_node",
                name="smarcduino_waraps_position_marker_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_waraps_marker")
                ),
                parameters=[params_file],
            ),
            Node(
                package="evolo_reference_markers",
                executable="lidar_position_marker_node",
                name="lidar_position_marker_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_lidar_marker")
                ),
                parameters=[params_file],
            ),
        ]
    )
