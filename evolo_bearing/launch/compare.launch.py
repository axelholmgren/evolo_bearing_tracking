"""Run corrected and uncorrected bearing rays against one truth source."""

import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


SAFE_RUN_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")


def _comparison_actions(
    *,
    use_sim_time,
    run_id,
    truth_source,
    truth_topic,
    lidar_box_id,
    yaw_correction_mode,
    negate_yaw_correction,
    show_rviz,
    rviz_config,
):
    uncorrected_error_parameters = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "bearing_topic": "/comparison/uncorrected",
        "truth_source": truth_source,
        "yaw_correction_mode": yaw_correction_mode,
        "negate_yaw_correction": ParameterValue(
            negate_yaw_correction, value_type=bool
        ),
        "save_csv": True,
        "output_file": f"results/{run_id}_uncorrected.csv",
        "error_topic": "/bearing_errors/uncorrected",
    }
    corrected_error_parameters = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "bearing_topic": "/comparison/corrected",
        "truth_source": truth_source,
        "yaw_correction_mode": yaw_correction_mode,
        "negate_yaw_correction": ParameterValue(
            negate_yaw_correction, value_type=bool
        ),
        "save_csv": True,
        "output_file": f"results/{run_id}_corrected.csv",
        "error_topic": "/bearing_errors/corrected",
    }
    if truth_source == "marker":
        uncorrected_error_parameters["truth_topic"] = truth_topic
        corrected_error_parameters["truth_topic"] = truth_topic
    else:
        uncorrected_error_parameters["lidar_boxes_topic"] = truth_topic
        uncorrected_error_parameters["lidar_box_id"] = lidar_box_id
        corrected_error_parameters["lidar_boxes_topic"] = truth_topic
        corrected_error_parameters["lidar_box_id"] = lidar_box_id

    actions = [
        Node(
            package="evolo_bearing",
            executable="bearing_marker_node",
            name="bearing_uncorrected",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "apply_yaw_correction": ParameterValue(False, value_type=bool),
                    "yaw_correction_mode": yaw_correction_mode,
                    "negate_yaw_correction": ParameterValue(
                        negate_yaw_correction, value_type=bool
                    ),
                }
            ],
            remappings=[
                (
                    "/evolo/gimbal_camera/target_bearing_marker",
                    "/comparison/uncorrected",
                )
            ],
        ),
        Node(
            package="evolo_bearing",
            executable="bearing_marker_node",
            name="bearing_corrected",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "apply_yaw_correction": ParameterValue(True, value_type=bool),
                    "yaw_correction_mode": yaw_correction_mode,
                    "negate_yaw_correction": ParameterValue(
                        negate_yaw_correction, value_type=bool
                    ),
                }
            ],
            remappings=[
                (
                    "/evolo/gimbal_camera/target_bearing_marker",
                    "/comparison/corrected",
                )
            ],
        ),
        Node(
            package="evolo_bearing_error",
            executable="bearing_error_node",
            name="error_uncorrected",
            output="screen",
            parameters=[uncorrected_error_parameters],
        ),
        Node(
            package="evolo_bearing_error",
            executable="bearing_error_node",
            name="error_corrected",
            output="screen",
            parameters=[corrected_error_parameters],
        ),
    ]
    if show_rviz:
        config = rviz_config or PathJoinSubstitution(
            [
                FindPackageShare("evolo_bearing"),
                "config",
                "tracking_ray_evolo_smarcduino.rviz",
            ]
        )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", config],
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            )
        )
    return actions


def _launch(context):
    run_id = LaunchConfiguration("run_id").perform(context).strip()
    if not SAFE_RUN_ID.fullmatch(run_id):
        raise RuntimeError(
            "comparison requires a run_id using letters, numbers, '.', '_' or '-'."
        )
    outputs = [
        Path(f"results/{run_id}_{variant}.csv")
        for variant in ("uncorrected", "corrected")
    ]
    existing = next((path.resolve() for path in outputs if path.exists()), None)
    if existing:
        raise RuntimeError(f"output file already exists; choose another run_id: {existing}")

    truth_source = LaunchConfiguration("truth_source").perform(context)
    if truth_source not in {"marker", "lidar_box"}:
        raise RuntimeError("truth_source must be 'marker' or 'lidar_box'")

    try:
        lidar_box_id = int(LaunchConfiguration("lidar_box_id").perform(context))
    except ValueError as exc:
        raise RuntimeError("lidar_box_id must be an integer") from exc

    yaw_correction_mode = LaunchConfiguration("yaw_correction_mode").perform(context)
    if yaw_correction_mode not in {"absolute", "shape"}:
        raise RuntimeError("yaw_correction_mode must be 'absolute' or 'shape'")
    negate_value = LaunchConfiguration("negate_yaw_correction").perform(context).lower()
    if negate_value not in {"true", "false", "1", "0"}:
        raise RuntimeError("negate_yaw_correction must be true or false")

    return _comparison_actions(
        use_sim_time=LaunchConfiguration("use_sim_time").perform(context),
        run_id=run_id,
        truth_source=truth_source,
        truth_topic=LaunchConfiguration("truth_topic").perform(context),
        lidar_box_id=lidar_box_id,
        yaw_correction_mode=yaw_correction_mode,
        negate_yaw_correction=negate_value in {"true", "1"},
        show_rviz=LaunchConfiguration("show_rviz").perform(context).lower()
        in {"true", "1"},
        rviz_config=LaunchConfiguration("rviz_config"),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("truth_source", default_value="lidar_box"),
            DeclareLaunchArgument(
                "truth_topic",
                default_value="/bounding_boxes/corrected",
                description="Marker topic or LiDAR MarkerArray topic used as truth.",
            ),
            DeclareLaunchArgument("lidar_box_id", default_value="0"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument("show_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("evolo_bearing"),
                        "config",
                        "tracking_ray_evolo_smarcduino.rviz",
                    ]
                ),
            ),
            OpaqueFunction(function=_launch),
        ]
    )
