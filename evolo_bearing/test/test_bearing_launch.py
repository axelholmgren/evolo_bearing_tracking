import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


_LAUNCH_DIR = Path(__file__).parents[1] / "launch"


def _load(name):
    path = _LAUNCH_DIR / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _argument_names(description):
    return {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }


def test_bearing_launch_contains_bearing_nodes_and_track_ids_argument():
    description = _load("bearing.launch.py").generate_launch_description()

    assert len([action for action in description.entities if isinstance(action, Node)]) == 2
    assert _argument_names(description) == {
        "use_sim_time",
        "apply_yaw_correction",
        "yaw_correction_mode",
        "negate_yaw_correction",
        "track_ids",
        "output_topic",
    }


def test_observe_launch_composes_bearing_markers_and_rviz():
    description = _load("observe.launch.py").generate_launch_description()

    includes = [
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 2
    included_locations = [
        str(action.launch_description_source.location) for action in includes
    ]
    assert any(
        "evolo_bearing" in location and "bearing.launch.py" in location
        for location in included_locations
    )
    assert any(
        "evolo_reference_markers" in location
        and "reference_markers.launch.py" in location
        for location in included_locations
    )
    assert len([action for action in description.entities if isinstance(action, Node)]) == 1


def test_comparison_launch_contains_paired_nodes_and_arguments():
    description = _load("compare.launch.py").generate_launch_description()

    assert len([action for action in description.entities if isinstance(action, Node)]) == 5
    assert _argument_names(description) == {
        "run_id",
        "use_sim_time",
        "truth_source",
        "truth_topic",
        "lidar_box_id",
        "yaw_correction_mode",
        "negate_yaw_correction",
        "show_rviz",
        "rviz_config",
    }


def test_derived_lidar_launch_exposes_replay_and_lidar_arguments():
    description = _load(
        "compare_derived_lidar.launch.py"
    ).generate_launch_description()

    assert _argument_names(description) == {
        "bag",
        "run_id",
        "use_sim_time",
        "replay_rate",
        "start_offset",
        "lidar_boxes_topic",
        "lidar_box_id",
        "yaw_correction_mode",
        "negate_yaw_correction",
    }
