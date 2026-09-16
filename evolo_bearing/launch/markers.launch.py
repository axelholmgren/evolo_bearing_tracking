"""Launch the bearing-marker visualisation and its reference markers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("evolo_bearing")
    use_sim_time = LaunchConfiguration("use_sim_time")
    yaw_correction_mode = LaunchConfiguration("yaw_correction_mode")
    negate_yaw_correction = LaunchConfiguration("negate_yaw_correction")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock; set true when replaying a bag with --clock.",
            ),
            DeclareLaunchArgument(
                "yaw_correction_mode",
                default_value="absolute",
                description="Yaw correction mode: shape or absolute.",
            ),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument(
                "track_ids",
                default_value="",
                description=(
                    "Optional YOLO track IDs for selected bearing rays, for example "
                    "'[44,68,99]'. Leave empty to disable selected-ID rays."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "tracking_ray_evolo_smarcduino.rviz"]
                ),
                description="Absolute path to an RViz configuration file.",
            ),
            DeclareLaunchArgument(
                "lidar_boxes",
                default_value="false",
                description=(
                    "Start LiDAR preprocessing, clustering, and corrected "
                    "bounding-box tracking."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, "launch", "bearing.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "yaw_correction_mode": yaw_correction_mode,
                    "negate_yaw_correction": negate_yaw_correction,
                    "track_ids": LaunchConfiguration("track_ids"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("evolo_reference_markers"),
                            "launch",
                            "reference_markers.launch.py",
                        ]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        )
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("pointcloud_preprocessing"),
                            "launch",
                            "pointcloud_preprocessing_launch_evolo.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("lidar_boxes")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("clustering_segmentation"),
                            "launch",
                            "mapping_clustering_segmentation_launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("lidar_boxes")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("bb_dataass_tracking"),
                            "launch",
                            "tracking_launch_evolo.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("lidar_boxes")),
            ),
        ]
    )
