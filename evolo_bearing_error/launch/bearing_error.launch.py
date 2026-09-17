"""Start the bearing-error node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            Node(
                package="evolo_bearing_error",
                executable="bearing_error_node",
                name="bearing_error_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
