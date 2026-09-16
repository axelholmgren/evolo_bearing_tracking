"""Run raw and yaw-corrected bearing rays side by side."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    run_id = LaunchConfiguration("run_id")
    correction_mode = LaunchConfiguration("yaw_correction_mode")
    negate_yaw_correction = LaunchConfiguration("negate_yaw_correction")
    lidar_boxes = LaunchConfiguration("lidar_boxes")
    lidar_boxes_topic = LaunchConfiguration("lidar_boxes_topic")
    lidar_box_id = LaunchConfiguration("lidar_box_id")
    rviz_config = LaunchConfiguration("rviz_config")

    common_ray_parameters = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "yaw_correction_mode": correction_mode,
        "negate_yaw_correction": ParameterValue(
            negate_yaw_correction, value_type=bool
        ),
    }
    common_logger_parameters = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "truth_source": "lidar_box",
        "lidar_boxes_topic": lidar_boxes_topic,
        "lidar_box_id": ParameterValue(lidar_box_id, value_type=int),
        "yaw_correction_mode": correction_mode,
        "negate_yaw_correction": ParameterValue(
            negate_yaw_correction, value_type=bool
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "run_id",
                default_value="replay",
                description="Identifier shared by the two output CSV filenames.",
            ),
            DeclareLaunchArgument(
                "yaw_correction_mode",
                default_value="absolute",
                description="Calibration mode: absolute (default) or shape.",
            ),
            DeclareLaunchArgument(
                "negate_yaw_correction",
                default_value="false",
                description="Apply the calibrated correction with the opposite sign.",
            ),
            DeclareLaunchArgument(
                "lidar_boxes",
                default_value="true",
                description="Start the LiDAR box pipeline from observe.launch.py.",
            ),
            DeclareLaunchArgument(
                "lidar_boxes_topic",
                default_value="/bounding_boxes/corrected",
                description="MarkerArray topic supplying LiDAR truth boxes.",
            ),
            DeclareLaunchArgument(
                "lidar_box_id",
                default_value="0",
                description="ID of the LiDAR box used as truth for both conditions.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("evolo_bearing_config"),
                        "config",
                        "tracking_ray_evolo_smarcduino.rviz",
                    ]
                ),
                description="RViz configuration passed to observe.launch.py.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("evolo_bearing_config"),
                            "launch",
                            "observe.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "yaw_correction_mode": correction_mode,
                    "negate_yaw_correction": negate_yaw_correction,
                    "lidar_boxes": lidar_boxes,
                    "rviz_config": rviz_config,
                }.items(),
            ),
            Node(
                package="evolo_bearing",
                executable="bearing_marker_node",
                name="bearing_ray_raw",
                output="screen",
                parameters=[
                    {**common_ray_parameters, "apply_yaw_correction": False}
                ],
                remappings=[
                    (
                        "/evolo/gimbal_camera/target_bearing_marker",
                        "/bearing_experiment/raw_marker",
                    )
                ],
            ),
            Node(
                package="evolo_bearing",
                executable="bearing_marker_node",
                name="bearing_ray_corrected",
                output="screen",
                parameters=[
                    {**common_ray_parameters, "apply_yaw_correction": True}
                ],
                remappings=[
                    (
                        "/evolo/gimbal_camera/target_bearing_marker",
                        "/bearing_experiment/corrected_marker",
                    )
                ],
            ),
            Node(
                package="evolo_bearing_error",
                executable="bearing_error_node",
                name="bearing_error_raw",
                output="screen",
                parameters=[
                    {
                        **common_logger_parameters,
                        "bearing_topic": "/bearing_experiment/raw_marker",
                        "bag": PythonExpression(["'", run_id, "' + '_raw'"]),
                    }
                ],
            ),
            Node(
                package="evolo_bearing_error",
                executable="bearing_error_node",
                name="bearing_error_corrected",
                output="screen",
                parameters=[
                    {
                        **common_logger_parameters,
                        "bearing_topic": "/bearing_experiment/corrected_marker",
                        "bag": PythonExpression(["'", run_id, "' + '_corrected'"]),
                    }
                ],
            ),
        ]
    )
