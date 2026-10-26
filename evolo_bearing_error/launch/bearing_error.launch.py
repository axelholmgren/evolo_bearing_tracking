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
            DeclareLaunchArgument(
                "bearing_topic",
                default_value="/evolo/gimbal_camera/target_bearing_marker",
            ),
            DeclareLaunchArgument("truth_topic", default_value="/fixed_position_marker"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument("extend_yaw_correction", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="evolo_bearing_error",
                executable="bearing_error_node",
                name="bearing_error_node",
                output="screen",
                # Before params_file, so bearing_error_node's YAML section overrides it.
                parameters=[
                    {
                        name: LaunchConfiguration(name)
                        for name in (
                            "bearing_topic",
                            "truth_topic",
                            "yaw_correction_mode",
                            "negate_yaw_correction",
                            "extend_yaw_correction",
                            "use_sim_time",
                        )
                    },
                    params_file,
                ],
            ),
        ]
    )
