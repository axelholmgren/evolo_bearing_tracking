"""Replay a rosbag alongside the normal bearing visualisation."""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch(context):
    bag_value = LaunchConfiguration("bag").perform(context).strip()
    if not bag_value:
        raise RuntimeError("replay requires bag:=<rosbag-directory>")
    bag = Path(bag_value).expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise RuntimeError(f"not a rosbag directory with metadata.yaml: {bag}")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower()
    if use_sim_time not in {"true", "1"}:
        raise RuntimeError("rosbag replay uses /clock; keep use_sim_time:=true")

    rate = float(LaunchConfiguration("replay_rate").perform(context))
    offset = float(LaunchConfiguration("start_offset").perform(context))
    if not math.isfinite(rate) or rate <= 0:
        raise RuntimeError("replay_rate must be a positive finite number")
    if not math.isfinite(offset) or offset < 0:
        raise RuntimeError("start_offset must be a non-negative finite number")

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
        LogInfo(msg=f"Replaying {bag} (use_sim_time=true)"),
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
            DeclareLaunchArgument("replay_rate", default_value="1.0"),
            DeclareLaunchArgument("start_offset", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument("track_ids", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("evolo_bearing"), "launch", "observe.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "yaw_correction_mode": LaunchConfiguration("yaw_correction_mode"),
                    "negate_yaw_correction": LaunchConfiguration("negate_yaw_correction"),
                    "track_ids": LaunchConfiguration("track_ids"),
                }.items(),
            ),
            OpaqueFunction(function=_launch),
        ]
    )
