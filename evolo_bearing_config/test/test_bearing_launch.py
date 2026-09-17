import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


_LAUNCH_DIR = Path(__file__).parents[1] / "launch"


def _load(name):
    path = _LAUNCH_DIR / name

    spec = importlib.util.spec_from_file_location(
        path.stem,
        path,
    )

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def test_bearing_launch():
    description = _load(
        "bearing.launch.py"
    ).generate_launch_description()

    arguments = {
        action.name
        for action in description.entities
        if isinstance(
            action,
            DeclareLaunchArgument,
        )
    }

    nodes = [
        action
        for action in description.entities
        if isinstance(action, Node)
    ]

    assert arguments == {
        "params_file",
        "start_bearing_ray",
        "start_bearing_ray_ids",
        "output_topic",
    }

    assert len(nodes) == 2
