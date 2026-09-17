"""Start the bearing-ray nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BEARING_TOPIC = "/evolo/gimbal_camera/target_bearing_marker"


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="ROS parameter YAML.",
            ),
            DeclareLaunchArgument(
                "start_bearing_ray",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "start_bearing_ray_ids",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value=BEARING_TOPIC,
            ),
            Node(
                package="evolo_bearing",
                executable="bearing_marker_node",
                name="bearing_ray_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_bearing_ray")
                ),
                parameters=[params_file],
                remappings=[
                    (
                        BEARING_TOPIC,
                        LaunchConfiguration("output_topic"),
                    )
                ],
            ),
            Node(
                package="evolo_bearing",
                executable="bearing_marker_ids_node",
                name="bearing_ray_ids_node",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_bearing_ray_ids")
                ),
                parameters=[params_file],
            ),
        ]
    )
