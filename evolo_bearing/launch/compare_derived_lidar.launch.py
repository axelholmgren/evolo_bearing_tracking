"""Compare corrected and uncorrected rays against LiDAR-derived truth."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    use_sim_time = LaunchConfiguration("use_sim_time")
    replay_rate = LaunchConfiguration("replay_rate")
    start_offset = LaunchConfiguration("start_offset")

    player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag,
            "--clock",
            "--rate",
            replay_rate,
            "--start-offset",
            start_offset,
        ],
        name="input_bag",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", default_value=""),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("replay_rate", default_value="1.0"),
            DeclareLaunchArgument("start_offset", default_value="0.0"),
            DeclareLaunchArgument(
                "lidar_boxes_topic", default_value="/bounding_boxes/corrected"
            ),
            DeclareLaunchArgument("lidar_box_id", default_value="0"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
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
                )
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
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("evolo_bearing"), "launch", "compare.launch.py"]
                    )
                ),
                launch_arguments={
                    "run_id": LaunchConfiguration("run_id"),
                    "use_sim_time": use_sim_time,
                    "truth_source": "lidar_box",
                    "truth_topic": LaunchConfiguration("lidar_boxes_topic"),
                    "lidar_box_id": LaunchConfiguration("lidar_box_id"),
                    "yaw_correction_mode": LaunchConfiguration("yaw_correction_mode"),
                    "negate_yaw_correction": LaunchConfiguration(
                        "negate_yaw_correction"
                    ),
                    "show_rviz": "false",
                }.items(),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=player,
                    on_exit=[EmitEvent(event=Shutdown(reason="rosbag playback finished"))],
                )
            ),
            TimerAction(period=2.0, actions=[player]),
        ]
    )
