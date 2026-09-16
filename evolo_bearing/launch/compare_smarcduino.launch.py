"""Compare both bearing rays against the Smarcduino reference marker."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("evolo_bearing"), "launch", "compare.launch.py"]
                    )
                ),
                launch_arguments={
                    "run_id": LaunchConfiguration("run_id"),
                    "use_sim_time": use_sim_time,
                    "truth_source": "marker",
                    "truth_topic": "/smarcduino/position_marker",
                    "yaw_correction_mode": LaunchConfiguration("yaw_correction_mode"),
                    "negate_yaw_correction": LaunchConfiguration("negate_yaw_correction"),
                    "show_rviz": "true",
                }.items(),
            ),
            Node(
                package="evolo_reference_markers",
                executable="smarcduino_marker_node",
                name="smarcduino_truth",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
