"""Run corrected and uncorrected bearing rays against one truth source."""

import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from evolo_bearing.launch_support import bearing_error_node, bearing_node, rviz_node


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
    actions = [
        bearing_node(
            "bearing_uncorrected",
            use_sim_time=use_sim_time,
            apply_yaw_correction=False,
            yaw_correction_mode=yaw_correction_mode,
            negate_yaw_correction=negate_yaw_correction,
            output_topic="/comparison/uncorrected",
        ),
        bearing_node(
            "bearing_corrected",
            use_sim_time=use_sim_time,
            apply_yaw_correction=True,
            yaw_correction_mode=yaw_correction_mode,
            negate_yaw_correction=negate_yaw_correction,
            output_topic="/comparison/corrected",
        ),
        bearing_error_node(
            "error_uncorrected",
            use_sim_time=use_sim_time,
            bearing_topic="/comparison/uncorrected",
            truth_source=truth_source,
            truth_topic=truth_topic,
            lidar_box_id=lidar_box_id,
            run_id=run_id,
            yaw_correction_mode=yaw_correction_mode,
            negate_yaw_correction=negate_yaw_correction,
            error_topic="/bearing_errors/uncorrected",
        ),
        bearing_error_node(
            "error_corrected",
            use_sim_time=use_sim_time,
            bearing_topic="/comparison/corrected",
            truth_source=truth_source,
            truth_topic=truth_topic,
            lidar_box_id=lidar_box_id,
            run_id=run_id,
            yaw_correction_mode=yaw_correction_mode,
            negate_yaw_correction=negate_yaw_correction,
            error_topic="/bearing_errors/corrected",
        ),
    ]
    if show_rviz:
        actions.append(rviz_node(use_sim_time=use_sim_time, rviz_config=rviz_config))
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
