import importlib.util
from pathlib import Path

import yaml

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch import LaunchContext


_PACKAGE_DIR = Path(__file__).parents[1]

_LAUNCH_FILE = (
    _PACKAGE_DIR
    / "launch"
    / "holy.launch.py"
)

_TEMPLATE_FILE = (
    _PACKAGE_DIR
    / "config"
    / "experiments"
    / "template.yaml"
)


def _load_holy():
    spec = importlib.util.spec_from_file_location(
        "holy_launch",
        _LAUNCH_FILE,
    )

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def _load_template():
    with _TEMPLATE_FILE.open() as file:
        return yaml.safe_load(file)


def test_holy_has_one_launch_argument():
    description = (
        _load_holy()
        .generate_launch_description()
    )

    arguments = [
        action
        for action in description.entities
        if isinstance(
            action,
            DeclareLaunchArgument,
        )
    ]

    opaque_functions = [
        action
        for action in description.entities
        if isinstance(
            action,
            OpaqueFunction,
        )
    ]

    assert len(arguments) == 1
    assert arguments[0].name == "config"

    assert len(opaque_functions) == 1


def test_template_has_launch_settings():
    config = _load_template()

    settings = config[
        "holy"
    ]["ros__parameters"]

    expected = {
        "bearing_ray",
        "bearing_ray_ids",
        "bearing_error",
        "fixed_marker",
        "smarcduino_marker",
        "waraps_marker",
        "lidar_marker",
        "lidar_boxes",
        "rviz",
        "rosbag",
        "bearing_output_topic",
        "rviz_config",
        "bag_path",
        "replay_rate",
        "start_offset",
        "stop_time",
    }

    assert expected <= set(settings)


def test_stop_time_uses_bag_time_after_start_offset(tmp_path):
    config = _load_template()
    settings = config["holy"]["ros__parameters"]
    settings.update(
        rosbag=True, bag_path="/tmp/input", start_offset=100.0,
        replay_rate=2.0, stop_time=150.0,
    )
    config_path = tmp_path / "experiment.yaml"
    config_path.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    context.launch_configurations["config"] = str(config_path)
    timers = [
        action for action in _load_holy()._launch_experiment(context)
        if isinstance(action, TimerAction)
    ]
    assert [timer._TimerAction__period for timer in timers] == [2.0, 27.0]


def test_template_has_bearing_parameters():
    config = _load_template()

    bearing = config[
        "bearing_ray_node"
    ]["ros__parameters"]

    assert set(bearing) == {
        "gimbal_gcu_feedback_topic",
        "apply_yaw_correction",
        "yaw_correction_mode",
        "negate_yaw_correction",
    }


def test_template_has_track_parameters():
    config = _load_template()

    bearing_ids = config[
        "bearing_ray_ids_node"
    ]["ros__parameters"]

    assert set(bearing_ids) == {
        "track_ids",
        "gimbal_gcu_feedback_topic",
        "apply_yaw_correction",
        "yaw_correction_mode",
        "negate_yaw_correction",
    }


def test_template_has_error_parameters():
    config = _load_template()

    error = config[
        "bearing_error_node"
    ]["ros__parameters"]

    assert set(error) == {
        "bearing_topic",
        "bearing_track_id",
        "truth_source",
        "truth_topic",
        "lidar_boxes_topic",
        "lidar_box_id",
        "yaw_correction_mode",
        "negate_yaw_correction",
        "gimbal_gcu_feedback_topic",
        "bag",
        "save_csv",
        "output_file",
        "error_topic",
    }
