"""Start the bearing view or one of its comparison workflows."""

from pathlib import Path
import math
import re

from ament_index_python.packages import get_package_share_directory
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BAG_WORKFLOWS = {
    "replay",
    "compare",
    "compare_fixed",
    "compare_smarcduino",
    "compare_derived_lidar",
}
COMPARE_WORKFLOWS = {
    "compare",
    "compare_fixed",
    "compare_smarcduino",
    "compare_live_lidar",
    "compare_external",
    "compare_derived_lidar",
}
WORKFLOWS = {"observe", *BAG_WORKFLOWS, "compare_live_lidar", "compare_external"}
SAFE_RUN_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")


def _node(package, executable, name, *, parameters=None, remappings=None, arguments=None):
    return Node(
        package=package,
        executable=executable,
        name=name,
        parameters=parameters or [],
        remappings=remappings or [],
        arguments=arguments or [],
        output="screen",
    )


def _bearing_node(name, corrected, use_sim_time, topic, yaw_correction_mode, negate_yaw_correction):
    return _node(
        "evolo_bearing",
        "bearing_marker_node",
        name,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "apply_yaw_correction": corrected,
                "yaw_correction_mode": yaw_correction_mode,
                "negate_yaw_correction": negate_yaw_correction,
            }
        ],
        remappings=[("/evolo/gimbal_camera/target_bearing_marker", topic)],
    )


def _error_node(
    name,
    bearing_topic,
    truth_source,
    truth_topic,
    use_sim_time,
    run_id,
    yaw_correction_mode,
    negate_yaw_correction,
):
    parameters = {
        "use_sim_time": use_sim_time,
        "bearing_topic": bearing_topic,
        "truth_source": truth_source,
        "yaw_correction_mode": yaw_correction_mode,
        "negate_yaw_correction": negate_yaw_correction,
        "save_csv": True,
        "output_file": f"results/{run_id}_{name.removeprefix('error_')}.csv",
        "error_topic": f"/bearing_errors/{name.removeprefix('error_')}",
    }
    if truth_source == "marker":
        parameters["truth_topic"] = truth_topic
    else:
        parameters["lidar_boxes_topic"] = truth_topic
        parameters["lidar_box_id"] = 0
    return _node("evolo_bearing_error", "bearing_error_node", name, parameters=[parameters])


def _rviz_node(use_sim_time):
    return _node(
        "rviz2",
        "rviz2",
        "rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("evolo_bearing"), "config", "tracking_ray_evolo_smarcduino.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )


def _lidar_include(package, launch_file, arguments=None):
    package_share = Path(get_package_share_directory(package))
    launch_path = package_share / "launch" / launch_file
    if not launch_path.is_file():
        raise RuntimeError(f"launch file is unavailable: {launch_path}")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(arguments or {}).items(),
    )


def _workflow_actions(
    workflow,
    use_sim_time,
    run_id,
    yaw_correction_mode="absolute",
    negate_yaw_correction=False,
):
    actions = []

    if workflow == "compare_derived_lidar":
        actions.extend(
            [
                _lidar_include(
                    "pointcloud_preprocessing",
                    "pointcloud_preprocessing_launch_evolo.py",
                    {"use_sim_time": str(use_sim_time).lower()},
                ),
                _lidar_include(
                    "clustering_segmentation",
                    "mapping_clustering_segmentation_launch.py",
                ),
                _lidar_include("bb_dataass_tracking", "tracking_launch_evolo.py"),
            ]
        )

    if workflow in {"observe", "replay"}:
        actions.append(
            _node(
                "evolo_bearing",
                "bearing_marker_node",
                "bearing_ray_node",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaw_correction_mode": yaw_correction_mode,
                        "negate_yaw_correction": negate_yaw_correction,
                    }
                ],
            )
        )
        for package, executable, name in (
            ("evolo_reference_markers", "smarcduino_marker_node", "smarcduino_position_marker_node"),
            (
                "evolo_reference_markers",
                "smarcduino_waraps_position_marker_node",
                "smarcduino_waraps_position_marker_node",
            ),
            ("evolo_reference_markers", "fixed_position_marker_node", "fixed_position_marker_node"),
        ):
            parameters = {"use_sim_time": use_sim_time}
            if executable == "fixed_position_marker_node":
                parameters.update(
                    {"latitude": 59.2985134, "longitude": 18.2146923, "altitude": 0.0}
                )
            actions.append(_node(package, executable, name, parameters=[parameters]))
        actions.append(_rviz_node(use_sim_time))
        return actions

    if workflow in COMPARE_WORKFLOWS:
        truth_source = "marker" if workflow in {"compare_fixed", "compare_smarcduino"} else "lidar_box"
        truth_topic = (
            "/fixed_position_marker"
            if workflow == "compare_fixed"
            else "/smarcduino/position_marker"
            if workflow == "compare_smarcduino"
            else "/bounding_boxes/corrected"
        )

        if workflow == "compare_fixed":
            actions.append(
                _node(
                    "evolo_reference_markers",
                    "fixed_position_marker_node",
                    "fixed_truth",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "latitude": 59.2985134,
                            "longitude": 18.2146923,
                            "altitude": 0.0,
                        }
                    ],
                )
            )
        elif workflow == "compare_smarcduino":
            actions.append(
                _node(
                    "evolo_reference_markers",
                    "smarcduino_marker_node",
                    "smarcduino_truth",
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            )

        actions.extend(
            [
                _bearing_node(
                    "bearing_uncorrected", False, use_sim_time, "/comparison/uncorrected",
                    yaw_correction_mode, negate_yaw_correction,
                ),
                _bearing_node(
                    "bearing_corrected", True, use_sim_time, "/comparison/corrected",
                    yaw_correction_mode, negate_yaw_correction,
                ),
                _error_node(
                    "error_uncorrected",
                    "/comparison/uncorrected",
                    truth_source,
                    truth_topic,
                    use_sim_time,
                    run_id,
                    yaw_correction_mode,
                    negate_yaw_correction,
                ),
                _error_node(
                    "error_corrected",
                    "/comparison/corrected",
                    truth_source,
                    truth_topic,
                    use_sim_time,
                    run_id,
                    yaw_correction_mode,
                    negate_yaw_correction,
                ),
            ]
        )
        if workflow in {"compare", "compare_smarcduino"}:
            actions.append(_rviz_node(use_sim_time))
    return actions


def _launch(context):
    workflow = LaunchConfiguration("workflow").perform(context)
    if workflow not in WORKFLOWS:
        choices = ", ".join(sorted(WORKFLOWS))
        raise RuntimeError(f"unknown workflow {workflow!r}; choose from: {choices}")

    bag = LaunchConfiguration("bag").perform(context).strip()
    run_id = LaunchConfiguration("run_id").perform(context).strip()
    if workflow in BAG_WORKFLOWS:
        if not bag:
            raise RuntimeError(f"workflow {workflow!r} requires bag:=<rosbag-directory>")
        bag_path = Path(bag).expanduser().resolve()
        if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
            raise RuntimeError(f"not a rosbag directory with metadata.yaml: {bag_path}")
    elif bag:
        raise RuntimeError(f"workflow {workflow!r} does not use a bag; remove bag:=...")
    else:
        bag_path = None

    if workflow in COMPARE_WORKFLOWS:
        if not SAFE_RUN_ID.fullmatch(run_id):
            raise RuntimeError(
                "comparison workflows require a run_id using letters, numbers, '.', '_' or '-'."
            )
        outputs = [Path(f"results/{run_id}_{variant}.csv") for variant in ("uncorrected", "corrected")]
        existing = next((path.resolve() for path in outputs if path.exists()), None)
        if existing:
            raise RuntimeError(f"output file already exists; choose another run_id: {existing}")
    elif run_id:
        raise RuntimeError(f"workflow {workflow!r} does not use run_id; remove run_id:=...")

    requested_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower()
    if requested_sim_time == "auto":
        use_sim_time = workflow in BAG_WORKFLOWS or workflow == "compare_external"
    elif requested_sim_time in {"true", "1"}:
        use_sim_time = True
    elif requested_sim_time in {"false", "0"}:
        use_sim_time = False
    else:
        raise RuntimeError("use_sim_time must be true, false, or auto")
    if bag_path is not None and not use_sim_time:
        raise RuntimeError("rosbag replay uses /clock; keep use_sim_time:=true")

    rate = float(LaunchConfiguration("replay_rate").perform(context))
    offset = float(LaunchConfiguration("start_offset").perform(context))
    if not math.isfinite(rate) or rate <= 0:
        raise RuntimeError("replay_rate must be a positive finite number")
    if not math.isfinite(offset) or offset < 0:
        raise RuntimeError("start_offset must be a non-negative finite number")

    yaw_correction_mode = LaunchConfiguration("yaw_correction_mode").perform(context)
    if yaw_correction_mode not in {"absolute", "shape"}:
        raise RuntimeError("yaw_correction_mode must be 'absolute' or 'shape'")
    negate_value = LaunchConfiguration("negate_yaw_correction").perform(context).lower()
    if negate_value not in {"true", "false", "1", "0"}:
        raise RuntimeError("negate_yaw_correction must be true or false")
    negate_yaw_correction = negate_value in {"true", "1"}

    actions = [LogInfo(msg=f"Starting bearing workflow '{workflow}' (use_sim_time={use_sim_time})")]
    actions.extend(
        _workflow_actions(
            workflow,
            use_sim_time,
            run_id,
            yaw_correction_mode,
            negate_yaw_correction,
        )
    )
    if bag_path is not None:
        player = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                str(bag_path),
                "--clock",
                "--rate",
                str(rate),
                "--start-offset",
                str(offset),
            ],
            name="input_bag",
            output="screen",
        )
        actions.extend(
            [
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=player,
                        on_exit=[EmitEvent(event=Shutdown(reason="rosbag playback finished"))],
                    )
                ),
                TimerAction(period=2.0, actions=[player]),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workflow",
                default_value="observe",
                description=f"Workflow to run: {', '.join(sorted(WORKFLOWS))}.",
            ),
            DeclareLaunchArgument(
                "bag", default_value="", description="Rosbag directory for replay workflows."
            ),
            DeclareLaunchArgument(
                "run_id", default_value="", description="Unique output name for comparison CSV files."
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="auto",
                description="true, false, or auto for the selected workflow.",
            ),
            DeclareLaunchArgument(
                "yaw_correction_mode",
                default_value="absolute",
                description="Yaw correction mode: absolute or shape.",
            ),
            DeclareLaunchArgument(
                "negate_yaw_correction",
                default_value="false",
                description="Apply the yaw correction with the opposite sign.",
            ),
            DeclareLaunchArgument(
                "replay_rate", default_value="1.0", description="Positive rosbag playback rate."
            ),
            DeclareLaunchArgument(
                "start_offset",
                default_value="0.0",
                description="Non-negative rosbag start offset in seconds.",
            ),
            OpaqueFunction(function=_launch),
        ]
    )
