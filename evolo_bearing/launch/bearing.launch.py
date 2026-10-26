"""Start the bearing-ray nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BEARING_TOPIC = "/evolo/gimbal_camera/target_bearing_marker"


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    camera_frame = LaunchConfiguration("camera_frame")
    camera_info = LaunchConfiguration("camera_info")
    # Before params_file, so a key in the node's own YAML section overrides it.
    derived = {
        "camera_frame": camera_frame,
        "camera_info": camera_info,
        "corrected": LaunchConfiguration("corrected"),
        "world_frame": LaunchConfiguration("world_frame"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="ROS parameter YAML.",
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="evolo/z1_camera_link",
            ),
            DeclareLaunchArgument(
                "camera_info",
                default_value="",
                description="Camera calibration YAML path, ""=linear aperture model.",
            ),
            DeclareLaunchArgument(
                "corrected",
                default_value="false",
                description="camera_frame is a corrected TF; colors rays by correction validity.",
            ),
            DeclareLaunchArgument("world_frame", default_value="evolo/odom"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
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
                condition=IfCondition(LaunchConfiguration("start_bearing_ray")),
                parameters=[derived, params_file],
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
                condition=IfCondition(LaunchConfiguration("start_bearing_ray_ids")),
                parameters=[derived, params_file],
            ),
        ]
    )
