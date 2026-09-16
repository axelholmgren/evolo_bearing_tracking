import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
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
