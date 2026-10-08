"""Run the existing YOLO executables with the local simulation environment.

Parameters left out of the experiment YAML follow the real Evolo stack:
smarc2 evolo_bringup.sh arguments over yolo_bringup/yolo.launch.py defaults.
"""

import importlib.util
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

# Arguments smarc2/scripts/smarc_bringups/scripts/evolo_bringup.sh passes on the boat.
EVOLO_YOLO_ARGS = {
    "model_type": "YOLOE", "image_reliability": 2, "device": "cuda:0",
    "tracker": "botsort.yaml",
}
EVOLO_ACTION_ARGS = {
    "image_poi_output": "/evolo/gimbal_camera/tracked_poi_image",
    "startup_classes": ["person", "boat", "buoy"],
}
# Parameters yolo.launch.py hands to each node.
NODE_PARAMS = {
    "yolo_node": (
        "model_type", "model", "device", "fuse_model", "yolo_encoding", "enable",
        "threshold", "iou", "imgsz_height", "imgsz_width", "half", "max_det",
        "augment", "agnostic_nms", "retina_masks", "image_reliability",
    ),
    "tracking_node": ("tracker", "image_reliability"),
    "debug_node": ("image_reliability",),
}


def _yolo_bringup_defaults():
    """yolo.launch.py argument defaults, typed as ROS parameters."""
    path = Path(get_package_share_directory("yolo_bringup")) / "launch" / "yolo.launch.py"
    spec = importlib.util.spec_from_file_location("yolo_bringup_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    run_yolo = next(
        entity for entity in module.generate_launch_description().entities
        if isinstance(entity, OpaqueFunction)
    )
    context = LaunchContext()
    context.launch_configurations.update(use_tracking="True", use_3d="False")
    return {
        action.name: yaml.safe_load(perform_substitutions(context, action.default_value))
        for action in run_yolo.execute(context)
        if isinstance(action, DeclareLaunchArgument)
    }


def _launch_yolo(context):
    params_file = Path(
        LaunchConfiguration("params_file").perform(context)
    ).expanduser().resolve()
    with params_file.open() as file:
        config = yaml.safe_load(file)
    venv = Path(
        config["holy"]["ros__parameters"]["yolo_venv"]
    ).expanduser().resolve()
    site_packages = venv / "lib/python3.10/site-packages"
    models = venv / "models"
    model = models / "yoloe-26s-seg.pt"
    for required in (site_packages, model, models / "mobileclip2_b.ts"):
        if not required.exists():
            raise FileNotFoundError(f"YOLO simulation requires {required}")

    environment = {
        "PYTHONPATH": f"{site_packages}:{os.environ.get('PYTHONPATH', '')}",
    }
    defaults = {**_yolo_bringup_defaults(), **EVOLO_YOLO_ARGS, "model": str(model)}
    nodes = []
    for executable, remappings in (
        ("yolo_node", []),
        ("tracking_node", []),
        ("debug_node", [("detections", "tracking")]),
    ):
        nodes.append(
            Node(
                package="yolo_ros",
                executable=executable,
                name=executable,
                namespace="yolo",
                output="screen",
                # mobileclip2_b.ts is resolved from the working directory.
                cwd=str(models),
                additional_env=environment,
                parameters=[
                    {key: defaults[key] for key in NODE_PARAMS[executable]},
                    str(params_file),
                ],
                remappings=[
                    ("image_raw", "/evolo/sensors/gimbal_camera/camera/image_raw"),
                    *remappings,
                ],
            )
        )
    nodes.append(
        Node(
            package="yolo_smarc_actions",
            executable="yolo_action.py",
            name="yolo_action_server_node",
            namespace="evolo",
            output="screen",
            parameters=[EVOLO_ACTION_ARGS, str(params_file)],
        )
    )
    # Sim gimbal loop: Unity has no Gcudata, so echo commands as feedback.
    nodes += [
        Node(
            package="evolo_bearing",
            executable="sim_gimbal_feedback_node",
            name="sim_gimbal_feedback_node",
            output="screen",
            parameters=[str(params_file)],
        ),
        Node(
            package="z1_pro_driver",
            executable="gimbal_action.py",
            name="gimbal_camera_action_server",
            namespace="evolo",
            output="screen",
            # z1_pro_action_launch.py default; the node's own is /yolo/tracked_poi.
            parameters=[
                {"img_poi_topic": EVOLO_ACTION_ARGS["image_poi_output"]},
                str(params_file),
            ],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", description="Holy experiment YAML."),
        OpaqueFunction(function=_launch_yolo),
    ])
