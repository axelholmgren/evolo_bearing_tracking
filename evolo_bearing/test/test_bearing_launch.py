import importlib.util
from pathlib import Path

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


_LAUNCH_FILE = (
    Path(__file__).parents[1] / "launch" / "bearing.launch.py"
)
_SPEC = importlib.util.spec_from_file_location("bearing_launch", _LAUNCH_FILE)
_LAUNCH = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_LAUNCH)


def test_observe_starts_bearing_reference_markers_and_rviz():
    actions = _LAUNCH._workflow_actions("observe", use_sim_time=False, run_id="")

    assert all(isinstance(action, Node) for action in actions)
    assert [action._Node__node_name for action in actions] == [
        "bearing_ray_node",
        "smarcduino_position_marker_node",
        "smarcduino_waraps_position_marker_node",
        "fixed_position_marker_node",
        "rviz2",
    ]


def test_comparison_starts_paired_bearings_and_error_nodes():
    actions = _LAUNCH._workflow_actions("compare", use_sim_time=True, run_id="trial_01")

    assert [action._Node__node_name for action in actions if isinstance(action, Node)] == [
        "bearing_uncorrected",
        "bearing_corrected",
        "error_uncorrected",
        "error_corrected",
        "rviz2",
    ]


def test_derived_lidar_comparison_includes_component_launches():
    actions = _LAUNCH._workflow_actions(
        "compare_derived_lidar", use_sim_time=True, run_id="trial_02"
    )

    assert sum(isinstance(action, IncludeLaunchDescription) for action in actions) == 3
    assert [action._Node__node_name for action in actions if isinstance(action, Node)] == [
        "bearing_uncorrected",
        "bearing_corrected",
        "error_uncorrected",
        "error_corrected",
    ]
