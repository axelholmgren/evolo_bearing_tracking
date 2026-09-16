"""Start one bearing-ray node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from evolo_bearing.launch_support import bearing_node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("apply_yaw_correction", default_value="true"),
            DeclareLaunchArgument(
                "yaw_correction_mode",
                default_value="absolute",
                description="Yaw correction mode: absolute or shape.",
            ),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/evolo/gimbal_camera/target_bearing_marker",
                description="Bearing marker topic.",
            ),
            bearing_node(
                "bearing_ray_node",
                use_sim_time=LaunchConfiguration("use_sim_time"),
                apply_yaw_correction=LaunchConfiguration("apply_yaw_correction"),
                yaw_correction_mode=LaunchConfiguration("yaw_correction_mode"),
                negate_yaw_correction=LaunchConfiguration("negate_yaw_correction"),
                output_topic=LaunchConfiguration("output_topic"),
            ),
        ]
    )
