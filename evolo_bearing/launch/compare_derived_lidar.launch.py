"""Compare corrected and uncorrected rays against LiDAR-derived truth."""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from evolo_bearing.launch_support import include_package_launch


def _launch(context):
    bag_value = LaunchConfiguration("bag").perform(context).strip()
    if not bag_value:
        raise RuntimeError("compare_derived_lidar requires bag:=<rosbag-directory>")
    bag = Path(bag_value).expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"not a rosbag directory with metadata.yaml: {bag}")

    use_sim_time_value = LaunchConfiguration("use_sim_time").perform(context).lower()
    if use_sim_time_value not in {"true", "1"}:
        raise RuntimeError("rosbag replay uses /clock; keep use_sim_time:=true")

    rate = float(LaunchConfiguration("replay_rate").perform(context))
    offset = float(LaunchConfiguration("start_offset").perform(context))
    if not math.isfinite(rate) or rate <= 0:
        raise RuntimeError("replay_rate must be a positive finite number")
    if not math.isfinite(offset) or offset < 0:
        raise RuntimeError("start_offset must be a non-negative finite number")

    use_sim_time = LaunchConfiguration("use_sim_time")
    player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            str(bag),
            "--clock",
            "--rate",
            str(rate),
            "--start-offset",
            str(offset),
        ],
        name="input_bag",
        output="screen",
    )
    return [
        include_package_launch(
            "pointcloud_preprocessing",
            "pointcloud_preprocessing_launch_evolo.py",
            launch_arguments={"use_sim_time": use_sim_time},
        ),
        include_package_launch(
            "clustering_segmentation",
            "mapping_clustering_segmentation_launch.py",
        ),
        include_package_launch("bb_dataass_tracking", "tracking_launch_evolo.py"),
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
                "negate_yaw_correction": LaunchConfiguration("negate_yaw_correction"),
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


def generate_launch_description():
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
            OpaqueFunction(function=_launch),
        ]
    )
