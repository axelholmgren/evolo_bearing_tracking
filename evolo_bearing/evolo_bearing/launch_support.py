"""Shared construction helpers for the bearing launch files."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _bool_parameter(value):
    return ParameterValue(value, value_type=bool)


def bearing_node(
    name,
    *,
    use_sim_time,
    apply_yaw_correction=True,
    yaw_correction_mode="absolute",
    negate_yaw_correction=False,
    output_topic=None,
):
    remappings = []
    if output_topic is not None:
        remappings.append(
            ("/evolo/gimbal_camera/target_bearing_marker", output_topic)
        )
    return Node(
        package="evolo_bearing",
        executable="bearing_marker_node",
        name=name,
        output="screen",
        parameters=[
            {
                "use_sim_time": _bool_parameter(use_sim_time),
                "apply_yaw_correction": _bool_parameter(apply_yaw_correction),
                "yaw_correction_mode": yaw_correction_mode,
                "negate_yaw_correction": _bool_parameter(negate_yaw_correction),
            }
        ],
        remappings=remappings,
    )


def bearing_error_node(
    name,
    *,
    use_sim_time,
    bearing_topic,
    truth_source,
    truth_topic,
    run_id,
    yaw_correction_mode,
    negate_yaw_correction,
    lidar_box_id=0,
    error_topic=None,
):
    parameters = {
        "use_sim_time": _bool_parameter(use_sim_time),
        "bearing_topic": bearing_topic,
        "truth_source": truth_source,
        "yaw_correction_mode": yaw_correction_mode,
        "negate_yaw_correction": _bool_parameter(negate_yaw_correction),
        "save_csv": True,
        "output_file": f"results/{run_id}_{name.removeprefix('error_')}.csv",
    }
    if truth_source == "marker":
        parameters["truth_topic"] = truth_topic
    else:
        parameters["lidar_boxes_topic"] = truth_topic
        parameters["lidar_box_id"] = lidar_box_id
    if error_topic is not None:
        parameters["error_topic"] = error_topic
    return Node(
        package="evolo_bearing_error",
        executable="bearing_error_node",
        name=name,
        output="screen",
        parameters=[parameters],
    )


def rviz_node(*, use_sim_time, rviz_config=None):
    config = rviz_config or PathJoinSubstitution(
        [FindPackageShare("evolo_bearing"), "config", "tracking_ray_evolo_smarcduino.rviz"]
    )
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", config],
        parameters=[{"use_sim_time": _bool_parameter(use_sim_time)}],
    )


def include_package_launch(package, launch_file, *, launch_arguments=None):
    package_share = Path(get_package_share_directory(package))
    launch_path = package_share / "launch" / launch_file
    if not launch_path.is_file():
        raise RuntimeError(f"launch file is unavailable: {launch_path}")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(launch_arguments or {}).items(),
    )
