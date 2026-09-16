"""Start the bearing-error logger."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "bearing_topic",
                default_value="/evolo/gimbal_camera/target_bearing_marker",
            ),
            DeclareLaunchArgument("bearing_track_id", default_value=""),
            DeclareLaunchArgument("truth_topic", default_value="/fixed_position_marker"),
            DeclareLaunchArgument("truth_source", default_value="marker"),
            DeclareLaunchArgument(
                "lidar_boxes_topic", default_value="/bounding_boxes/corrected"
            ),
            DeclareLaunchArgument("lidar_box_id", default_value="0"),
            DeclareLaunchArgument("yaw_correction_mode", default_value="absolute"),
            DeclareLaunchArgument("negate_yaw_correction", default_value="false"),
            DeclareLaunchArgument(
                "gimbal_gcu_feedback_topic",
                default_value="/evolo/gimbal_camera/gimbal_gcu_fb",
            ),
            DeclareLaunchArgument("bag", default_value=""),
            DeclareLaunchArgument("save_csv", default_value="true"),
            DeclareLaunchArgument("output_file", default_value=""),
            DeclareLaunchArgument("error_topic", default_value=""),
            Node(
                package="evolo_bearing_error",
                executable="bearing_error_node",
                name="bearing_error_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "bearing_topic": LaunchConfiguration("bearing_topic"),
                        "bearing_track_id": LaunchConfiguration("bearing_track_id"),
                        "truth_topic": LaunchConfiguration("truth_topic"),
                        "truth_source": LaunchConfiguration("truth_source"),
                        "lidar_boxes_topic": LaunchConfiguration("lidar_boxes_topic"),
                        "lidar_box_id": ParameterValue(
                            LaunchConfiguration("lidar_box_id"), value_type=int
                        ),
                        "yaw_correction_mode": LaunchConfiguration(
                            "yaw_correction_mode"
                        ),
                        "negate_yaw_correction": ParameterValue(
                            LaunchConfiguration("negate_yaw_correction"),
                            value_type=bool,
                        ),
                        "gimbal_gcu_feedback_topic": LaunchConfiguration(
                            "gimbal_gcu_feedback_topic"
                        ),
                        "bag": LaunchConfiguration("bag"),
                        "save_csv": ParameterValue(
                            LaunchConfiguration("save_csv"), value_type=bool
                        ),
                        "output_file": LaunchConfiguration("output_file"),
                        "error_topic": LaunchConfiguration("error_topic"),
                    }
                ],
            ),
        ]
    )
