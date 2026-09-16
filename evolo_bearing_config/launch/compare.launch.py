"""Run corrected and uncorrected bearing rays against one truth source."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    truth_source = LaunchConfiguration("truth_source")
    truth_topic = LaunchConfiguration("truth_topic")
    lidar_box_id = LaunchConfiguration("lidar_box_id")
    yaw_correction_mode = LaunchConfiguration("yaw_correction_mode")
    negate_yaw_correction = LaunchConfiguration("negate_yaw_correction")

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("truth_source", default_value="lidar_box"),
            DeclareLaunchArgument(
                "truth_topic",
                default_value="/bounding_boxes/corrected",
                description="Marker topic or LiDAR MarkerArray topic used as truth.",
            ),
            DeclareLaunchArgument("lidar_box_id", default_value="0"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument("show_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("evolo_bearing_config"),
                        "config",
                        "tracking_ray_evolo_smarcduino.rviz",
                    ]
                ),
            ),
            Node(
                package="evolo_bearing",
                executable="bearing_marker_node",
                name="bearing_uncorrected",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "apply_yaw_correction": False,
                        "yaw_correction_mode": yaw_correction_mode,
                        "negate_yaw_correction": ParameterValue(
                            negate_yaw_correction, value_type=bool
                        ),
                    }
                ],
                remappings=[
                    (
                        "/evolo/gimbal_camera/target_bearing_marker",
                        "/comparison/uncorrected",
                    )
                ],
            ),
            Node(
                package="evolo_bearing",
                executable="bearing_marker_node",
                name="bearing_corrected",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "apply_yaw_correction": True,
                        "yaw_correction_mode": yaw_correction_mode,
                        "negate_yaw_correction": ParameterValue(
                            negate_yaw_correction, value_type=bool
                        ),
                    }
                ],
                remappings=[
                    (
                        "/evolo/gimbal_camera/target_bearing_marker",
                        "/comparison/corrected",
                    )
                ],
            ),
            Node(
                package="evolo_bearing_error",
                executable="bearing_error_node",
                name="error_uncorrected",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "bearing_topic": "/comparison/uncorrected",
                        "truth_topic": truth_topic,
                        "truth_source": truth_source,
                        "lidar_boxes_topic": truth_topic,
                        "lidar_box_id": ParameterValue(lidar_box_id, value_type=int),
                        "yaw_correction_mode": yaw_correction_mode,
                        "negate_yaw_correction": ParameterValue(
                            negate_yaw_correction, value_type=bool
                        ),
                        "save_csv": True,
                        "output_file": PathJoinSubstitution(
                            [
                                "results",
                                [LaunchConfiguration("run_id"), "_uncorrected.csv"],
                            ]
                        ),
                        "error_topic": "/bearing_errors/uncorrected",
                    }
                ],
            ),
            Node(
                package="evolo_bearing_error",
                executable="bearing_error_node",
                name="error_corrected",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "bearing_topic": "/comparison/corrected",
                        "truth_topic": truth_topic,
                        "truth_source": truth_source,
                        "lidar_boxes_topic": truth_topic,
                        "lidar_box_id": ParameterValue(lidar_box_id, value_type=int),
                        "yaw_correction_mode": yaw_correction_mode,
                        "negate_yaw_correction": ParameterValue(
                            negate_yaw_correction, value_type=bool
                        ),
                        "save_csv": True,
                        "output_file": PathJoinSubstitution(
                            [
                                "results",
                                [LaunchConfiguration("run_id"), "_corrected.csv"],
                            ]
                        ),
                        "error_topic": "/bearing_errors/corrected",
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(LaunchConfiguration("show_rviz")),
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
        ]
    )
