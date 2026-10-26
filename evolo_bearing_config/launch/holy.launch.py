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
SELECTED_BEARING_TOPIC = "/evolo/gimbal_camera/selected_bearing_marker"


def _bool(value):
    return "true" if value else "false"


def _section(config, node):
    # Sections with every key commented out parse as None.
    return (config.get(node) or {}).get("ros__parameters") or {}


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
    config_path = (
        Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    )

    with config_path.open() as file:
        config = yaml.safe_load(file) or {}

    holy = _section(config, "holy")
    common = _section(config, "/**")
    correction = _section(config, "corrected_camera_tf_node")
    error = _section(config, "bearing_error_node")

    # Derived values go before the YAML in each node's parameters, so a key set
    # in the node's own section (or /**) overrides them.
    rosbag = holy.get("rosbag", False)
    use_sim_time = common.get("use_sim_time", rosbag)
    world_frame = holy.get("world_frame", "evolo/odom")

    actions = []

    # Bearing
    bearing_ray = holy.get("bearing_ray", False)
    bearing_ray_ids = holy.get("bearing_ray_ids", False)
    corrected_camera_tf = holy.get("corrected_camera_tf", False)
    camera = holy.get("camera", "raw")
    if camera not in ("raw", "global"):
        raise ValueError("camera must be raw or global")
    raw_camera_frame = correction.get(
        "camera_frame",
        "evolo/z1_global_camera_link" if camera == "global" else "evolo/z1_camera_link",
    )
    corrected_camera_frame = correction.get(
        "corrected_camera_frame", raw_camera_frame.replace("_link", "_corrected_link")
    )
    # Rays read the frame the correction node publishes (or reads).
    camera_frame = corrected_camera_frame if corrected_camera_tf else raw_camera_frame
    bearing_output_topic = holy.get("bearing_output_topic", BEARING_TOPIC)

    # ""=linear aperture model, otherwise a file in evolo_config/config
    # Uses the similar camera_calibration_file pattern as alars ekf_launch.py
    camera_calibration_file = holy.get("camera_calibration_file", "")
    camera_info = (
        str(
            Path(FindPackageShare("evolo_config").perform(context))
            / "config"
            / camera_calibration_file
        )
        if camera_calibration_file
        else ""
    )

    if corrected_camera_tf:
        actions.append(
            Node(
                package="evolo_gimbal_calibration",
                executable="corrected_camera_tf_node",
                name="corrected_camera_tf_node",
                output="screen",
                parameters=[
                    {
                        "camera_frame": raw_camera_frame,
                        "corrected_camera_frame": corrected_camera_frame,
                        # Global-camera bags publish the camera TF, not the yaw TF.
                        "camera_tf_driven": camera == "global",
                        "use_sim_time": use_sim_time,
                    },
                    str(config_path),
                ],
            )
        )

    if bearing_ray or bearing_ray_ids:
        actions.append(
            _include(
                "evolo_bearing",
                "bearing.launch.py",
                {
                    "params_file": str(config_path),
                    "start_bearing_ray": _bool(bearing_ray),
                    "start_bearing_ray_ids": _bool(bearing_ray_ids),
                    "camera_frame": camera_frame,
                    "corrected": _bool(corrected_camera_tf),
                    "world_frame": world_frame,
                    "camera_info": camera_info,
                    "output_topic": bearing_output_topic,
                    "use_sim_time": _bool(use_sim_time),
                },
            )
        )

    # Reference markers
    marker_topics = {
        "fixed_marker": _section(config, "fixed_position_marker_node").get(
            "marker_topic", "/fixed_position_marker"
        ),
        "smarcduino_marker": "/smarcduino/position_marker",
        "waraps_marker": "/smarcduino/waraps/position_marker",
    }
    enabled_marker_topics = [
        topic for marker, topic in marker_topics.items() if holy.get(marker, False)
    ]

    # Bearing error
    if holy.get("bearing_error", False):
        if (
            error.get("truth_source", "marker") == "marker"
            and "truth_topic" not in error
            and len(enabled_marker_topics) != 1
        ):
            raise ValueError(
                "bearing_error needs exactly one enabled marker, "
                "or set bearing_error_node.truth_topic"
            )
        actions.append(
            _include(
                "evolo_bearing_error",
                "bearing_error.launch.py",
                {
                    "params_file": str(config_path),
                    "bearing_topic": (
                        SELECTED_BEARING_TOPIC
                        if bearing_ray_ids and not bearing_ray
                        else bearing_output_topic
                    ),
                    "truth_topic": (
                        enabled_marker_topics[0]
                        if len(enabled_marker_topics) == 1
                        else "/fixed_position_marker"
                    ),
                    # Same correction as corrected_camera_tf_node; node defaults otherwise.
                    "yaw_correction_mode": correction.get(
                        "yaw_correction_mode", "absolute"
                    ),
                    "negate_yaw_correction": _bool(
                        correction.get("negate_yaw_correction", False)
                    ),
                    "extend_yaw_correction": _bool(
                        correction.get("extend_yaw_correction", False)
                    ),
                    "use_sim_time": _bool(use_sim_time),
                },
            )
        )

    if enabled_marker_topics:
        actions.append(
            _include(
                "evolo_reference_markers",
                "reference_markers.launch.py",
                {
                    "params_file": str(config_path),
                    "start_fixed_marker": _bool(holy.get("fixed_marker", False)),
                    "start_smarcduino_marker": _bool(
                        holy.get("smarcduino_marker", False)
                    ),
                    "start_waraps_marker": _bool(holy.get("waraps_marker", False)),
                    "world_frame": world_frame,
                    "use_sim_time": _bool(use_sim_time),
                },
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
                {"use_sim_time": _bool(use_sim_time)},
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
    if rosbag:
        bag_path = holy.get("bag_path", "")
        replay_rate = float(holy.get("replay_rate", 1.0))
        start_offset = float(holy.get("start_offset", 0.0))
        stop_time = holy.get("stop_time")

        if not bag_path:
            raise ValueError("rosbag is enabled but bag_path is empty")
        if replay_rate <= 0:
            raise ValueError("replay_rate must be positive")
        if stop_time is not None:
            stop_time = float(stop_time)
            if stop_time <= start_offset:
                raise ValueError("stop_time must be greater than start_offset")

        player = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                str(Path(bag_path).expanduser()),
                *(["--clock"] if holy.get("clock", use_sim_time) else []),
                "--rate",
                str(replay_rate),
                "--start-offset",
                str(start_offset),
            ],
            name="input_bag",
            output="screen",
        )

        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=player,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="rosbag playback finished"))
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
        if stop_time is not None:
            actions.append(
                TimerAction(
                    period=2.0 + (stop_time - start_offset) / replay_rate,
                    actions=[
                        EmitEvent(
                            event=Shutdown(reason="configured rosbag stop time reached")
                        )
                    ],
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
                        FindPackageShare("evolo_bearing_config"),
                        "config",
                        "experiments",
                        "template.yaml",
                    ]
                ),
                description="Experiment YAML.",
            ),
            OpaqueFunction(function=_launch_experiment),
        ]
    )
