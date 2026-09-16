"""Start the bearing-ray nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BEARING_TOPIC = "/evolo/gimbal_camera/target_bearing_marker"


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
            DeclareLaunchArgument("track_ids", default_value=""),
            DeclareLaunchArgument(
                "output_topic",
                default_value=BEARING_TOPIC,
                description="Bearing marker topic.",
            ),
            Node(
                package="evolo_bearing",
                executable="bearing_marker_node",
                name="bearing_ray_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "apply_yaw_correction": ParameterValue(
                            LaunchConfiguration("apply_yaw_correction"), value_type=bool
                        ),
                        "yaw_correction_mode": LaunchConfiguration(
                            "yaw_correction_mode"
                        ),
                        "negate_yaw_correction": ParameterValue(
                            LaunchConfiguration("negate_yaw_correction"),
                            value_type=bool,
                        ),
                    }
                ],
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
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("track_ids"), "' != ''"])
                ),
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "track_ids": LaunchConfiguration("track_ids"),
                        "apply_yaw_correction": ParameterValue(
                            LaunchConfiguration("apply_yaw_correction"), value_type=bool
                        ),
                        "yaw_correction_mode": LaunchConfiguration(
                            "yaw_correction_mode"
                        ),
                        "negate_yaw_correction": ParameterValue(
                            LaunchConfiguration("negate_yaw_correction"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
