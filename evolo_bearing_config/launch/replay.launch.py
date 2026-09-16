"""Replay a rosbag alongside the normal bearing visualisation."""

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
            DeclareLaunchArgument("replay_rate", default_value="1.0"),
            DeclareLaunchArgument("start_offset", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument("track_ids", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("evolo_bearing_config"),
                            "launch",
                            "observe.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "yaw_correction_mode": LaunchConfiguration("yaw_correction_mode"),
                    "negate_yaw_correction": LaunchConfiguration(
                        "negate_yaw_correction"
                    ),
                    "track_ids": LaunchConfiguration("track_ids"),
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
