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

    assert [
        action._Node__node_name
        for action in description.entities
        if isinstance(action, Node)
    ] == ["bearing_ray_node", "bearing_ray_ids_node"]
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
    assert [
        action._Node__node_name
        for action in description.entities
        if isinstance(action, Node)
    ] == ["rviz2"]


def test_comparison_builds_paired_bearings_and_error_nodes():
    actions = _load("compare.launch.py")._comparison_actions(
        use_sim_time=True,
        run_id="trial_01",
        truth_source="lidar_box",
        truth_topic="/bounding_boxes/corrected",
        lidar_box_id=0,
        yaw_correction_mode="absolute",
        negate_yaw_correction=False,
        show_rviz=False,
        rviz_config=None,
    )

    assert [action._Node__node_name for action in actions if isinstance(action, Node)] == [
        "bearing_uncorrected",
        "bearing_corrected",
        "error_uncorrected",
        "error_corrected",
    ]


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
