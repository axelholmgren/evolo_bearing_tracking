"""Start the live bearing visualisation and reference markers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from evolo_bearing.launch_support import rviz_node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    yaw_correction_mode = LaunchConfiguration("yaw_correction_mode")
    negate_yaw_correction = LaunchConfiguration("negate_yaw_correction")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "yaw_correction_mode",
                default_value="absolute",
                description="Yaw correction mode passed to bearing.launch.py.",
            ),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument(
                "track_ids",
                default_value="",
                description="Optional selected YOLO track IDs, for example '[44,68,99]'.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("evolo_bearing"),
                        "config",
                        "tracking_ray_evolo_smarcduino.rviz",
                    ]
                ),
                description="RViz configuration file.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("evolo_bearing"), "launch", "bearing.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "yaw_correction_mode": yaw_correction_mode,
                    "negate_yaw_correction": negate_yaw_correction,
                }.items(),
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
                        "use_sim_time": use_sim_time,
                        "track_ids": LaunchConfiguration("track_ids"),
                        "yaw_correction_mode": yaw_correction_mode,
                        "negate_yaw_correction": negate_yaw_correction,
                    }
                ],
            ),
            Node(
                package="evolo_reference_markers",
                executable="smarcduino_marker_node",
                name="smarcduino_position_marker_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="evolo_reference_markers",
                executable="smarcduino_waraps_position_marker_node",
                name="smarcduino_waraps_position_marker_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="evolo_reference_markers",
                executable="fixed_position_marker_node",
                name="fixed_position_marker_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            rviz_node(
                use_sim_time=use_sim_time,
                rviz_config=LaunchConfiguration("rviz_config"),
            ),
        ]
    )
