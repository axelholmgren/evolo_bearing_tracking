"""Launch a bearing-tracking experiment from one YAML file."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

BEARING_TOPIC = "/evolo/gimbal_camera/target_bearing_marker"


def _bool(value):
    return "true" if value else "false"


def _include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(package),
                    "launch",
                    filename,
                ]
            )
        ),
        launch_arguments=(arguments or {}).items(),
    )


def _launch_experiment(context):
    config_path = Path(
        LaunchConfiguration("config").perform(context)
    ).expanduser().resolve()

    with config_path.open() as file:
        config = yaml.safe_load(file) or {}

    holy = config.get("holy", {}).get("ros__parameters", {})
    common = config.get("/**", {}).get("ros__parameters", {})

    use_sim_time = common.get("use_sim_time", False)

    actions = []

    # Bearing
    bearing_ray = holy.get("bearing_ray", False)
    bearing_ray_ids = holy.get("bearing_ray_ids", False)

    if bearing_ray or bearing_ray_ids:
        actions.append(
            _include(
                "evolo_bearing",
                "bearing.launch.py",
                {
                    "params_file": str(config_path),
                    "start_bearing_ray": _bool(bearing_ray),
                    "start_bearing_ray_ids": _bool(
                        bearing_ray_ids
                    ),
                    "output_topic": holy.get(
                        "bearing_output_topic",
                        BEARING_TOPIC,
                    ),
                },
            )
        )

    # Bearing error
    if holy.get("bearing_error", False):
        actions.append(
            _include(
                "evolo_bearing_error",
                "bearing_error.launch.py",
                {
                    "params_file": str(config_path),
                },
            )
        )

    # Reference markers
    marker_arguments = {
        "params_file": str(config_path),
        "start_fixed_marker": _bool(
            holy.get("fixed_marker", False)
        ),
        "start_smarcduino_marker": _bool(
            holy.get("smarcduino_marker", False)
        ),
        "start_waraps_marker": _bool(
            holy.get("waraps_marker", False)
        ),
        "start_lidar_marker": _bool(
            holy.get("lidar_marker", False)
        ),
    }

    if any(
        [
            holy.get("fixed_marker", False),
            holy.get("smarcduino_marker", False),
            holy.get("waraps_marker", False),
            holy.get("lidar_marker", False),
        ]
    ):
        actions.append(
            _include(
                "evolo_reference_markers",
                "reference_markers.launch.py",
                marker_arguments,
            )
        )

    # LiDAR preprocessing / clustering / tracking
    if holy.get("lidar_boxes", False):
        actions.append(
            _include(
                "pointcloud_preprocessing",
                "pointcloud_preprocessing_launch_evolo.py",
                {
                    "use_sim_time": _bool(use_sim_time),
                },
            )
        )

        actions.append(
            _include(
                "clustering_segmentation",
                "mapping_clustering_segmentation_launch.py",
                { "use_sim_time": _bool(use_sim_time)},
            )
        )

        actions.append(
            _include(
                "bb_dataass_tracking",
                "tracking_launch_evolo.py",
                {"use_sim_time": _bool(use_sim_time)},

            )
        )

    # RViz
    if holy.get("rviz", False):
        rviz_config = holy.get("rviz_config", "")

        if not rviz_config:
            rviz_config = PathJoinSubstitution(
                [
                    FindPackageShare("evolo_bearing_config"),
                    "config",
                    "rviz",
                    "rviz",
                    "tracking_ray_evolo_smarcduino.rviz",
                ]
            )

        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    rviz_config,
                ],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        )

    # Rosbag
    if holy.get("rosbag", False):
        bag_path = holy.get("bag_path", "")

        if not bag_path:
            raise ValueError(
                "rosbag is enabled but bag_path is empty"
            )

        player = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                str(Path(bag_path).expanduser()),
                *(["--clock"] if holy.get("clock", True) else []),
                "--rate",
                str(holy.get("replay_rate", 1.0)),
                "--start-offset",
                str(holy.get("start_offset", 0.0)),
            ],
            name="input_bag",
            output="screen",
        )

        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=player,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="rosbag playback finished"
                            )
                        )
                    ],
                )
            )
        )

        actions.append(
            TimerAction(
                period=2.0,
                actions=[player],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare(
                            "evolo_bearing_config"
                        ),
                        "config",
                        "experiments",
                        "template.yaml",
                    ]
                ),
                description="Experiment YAML.",
            ),
            OpaqueFunction(
                function=_launch_experiment
            ),
        ]
    )
