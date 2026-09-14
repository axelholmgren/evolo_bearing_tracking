#!/usr/bin/env python3
"""Logs the angle between a bearing marker and a known target marker to csv.

Consumes the markers the other nodes already publish rather than recomputing
any geometry, so this measures what actually gets drawn. Both markers are in
the map frame, so no tf or convergence correction is needed here.

    bearing_topic          an ARROW marker, points[0] = camera, points[1] = ray tip
    truth_topic            a SPHERE marker at the known position
    gimbal_gcu_feedback_topic  raw z1 pro Gcudata, logged alongside for
                           gimbal_yaw_correction.py analysis

Pick which pair to compare with the parameters, e.g. the chosen track ids
against the fixed coordinate. bag only names the output file:

    ros2 run ... --ros-args \\
        -p bearing_topic:=/evolo/gimbal_camera/selected_bearing_marker \\
        -p truth_topic:=/fixed_position_marker \\
        -p bag:=rosbag2_2026_08_17-15_02_44

Use `-p use_sim_time:=true` when replaying with `ros2 bag play --clock`.
"""

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from z1_pro_msgs.msg import Gcudata

from .bearing_error import bearing_error_2d

from evolo_gimbal_calibration.gimbal_yaw_correction import correct_yaw

RESULTS_DIR = Path(__file__).resolve().parents[2] / "results"
TRUTH_MAX_AGE_S = 3.0  # skip rows where the truth marker is older than this


class BearingErrorNode(Node):
    """
    Logs bearing error against a known target to csv.
    """

    def __init__(self):
        super().__init__("bearing_error_node")

        self.declare_parameter(
            "bearing_topic", "/evolo/gimbal_camera/target_bearing_marker"
        )
        self.declare_parameter("truth_topic", "/fixed_position_marker")
        self.declare_parameter("truth_source", "marker")
        self.declare_parameter("lidar_boxes_topic", "/bounding_boxes/corrected")
        self.declare_parameter("lidar_box_id", 0)
        self.declare_parameter("yaw_correction_mode", "absolute")
        self.declare_parameter("negate_yaw_correction", False)
        self.declare_parameter(
            "gimbal_gcu_feedback_topic", "/evolo/gimbal_camera/gimbal_gcu_fb"
        )
        self.declare_parameter("bag", "rosbag2_2026_08_17-15_02_44")

        truth_source = self.get_parameter("truth_source").value
        yaw_correction_mode = self.get_parameter("yaw_correction_mode").value
        bearing_name = self.get_parameter("bearing_topic").value.strip("/").replace("/", "_")
        if truth_source == "lidar_box":
            truth_name = (
                f"lidar_box_{self.get_parameter('lidar_box_id').value}_"
                f"{self.get_parameter('lidar_boxes_topic').value.strip('/').replace('/', '_')}"
            )
        else:
            truth_name = self.get_parameter("truth_topic").value.strip("/").replace("/", "_")

        output_csv = RESULTS_DIR / (
            f"bearing_error_{self.get_parameter('bag').value}_"
            f"{bearing_name}_vs_{truth_name}_yaw_{yaw_correction_mode}.csv"
        )
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = output_csv.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "t",
                "ns",
                "marker_id",
                "track_id",
                "boresight_deg",
                "angle_in_frame_deg",
                "gimbal_yaw_deg",
                "corrected_yaw_deg",
                "yaw_correction_valid",
                "yaw_sigma_deg",
                "angle_error_deg",
                "miss_distance_m",
                "range_m",
            ]
        )

        self.truth = None
        self.truth_stamp = None
        self.gimbal_yaw_deg = None  # raw z1 pro readout, for gimbal_yaw_correction.py
        self.corrected_yaw_deg = None
        self.yaw_correction_valid = None
        self.yaw_sigma_deg = None

        self.gimbal_subscription = self.create_subscription(
            msg_type=Gcudata,
            topic=self.get_parameter("gimbal_gcu_feedback_topic").value,
            callback=self.gimbal_callback,
            qos_profile=10,
        )
        if truth_source == "lidar_box":
            self.truth_subscription = self.create_subscription(
                msg_type=MarkerArray,
                topic=self.get_parameter("lidar_boxes_topic").value,
                callback=self.lidar_truth_callback,
                qos_profile=10,
            )
        elif truth_source == "marker":
            self.truth_subscription = self.create_subscription(
                msg_type=Marker,
                topic=self.get_parameter("truth_topic").value,
                callback=self.truth_callback,
                qos_profile=10,
            )
        else:
            raise ValueError("truth_source must be 'marker' or 'lidar_box'")
        self.bearing_subscription = self.create_subscription(
            msg_type=Marker,
            topic=self.get_parameter("bearing_topic").value,
            callback=self.bearing_callback,
            qos_profile=10,
        )

        truth_description = (
            f"lidar box {self.get_parameter('lidar_box_id').value} on "
            f"{self.get_parameter('lidar_boxes_topic').value}"
            if truth_source == "lidar_box"
            else self.get_parameter("truth_topic").value
        )
        self.get_logger().info(
            f"{self.get_parameter('bearing_topic').value} vs "
            f"{truth_description} -> {output_csv}"
        )

    def truth_callback(self, msg: Marker):
        self.truth = msg
        self.truth_stamp = self.get_clock().now()

    def lidar_truth_callback(self, msg: MarkerArray):
        box_id = self.get_parameter("lidar_box_id").value
        self.truth = next(
            (
                marker
                for marker in msg.markers
                if marker.type == Marker.CUBE and marker.id == box_id
            ),
            None,
        )
        if self.truth is not None:
            self.truth_stamp = self.get_clock().now()

    def gimbal_callback(self, msg: Gcudata):
        self.gimbal_yaw_deg = msg.relative_yaw  # matches gimbal_yaw_correction.py's psi_readout
        result = correct_yaw(
            self.gimbal_yaw_deg,
            mode=self.get_parameter("yaw_correction_mode").value,
            negate=self.get_parameter("negate_yaw_correction").value,
        )
        self.corrected_yaw_deg = result.yaw_deg
        self.yaw_correction_valid = result.valid
        self.yaw_sigma_deg = result.sigma_deg

    def bearing_callback(self, msg: Marker):
        if self.truth is None:
            return  # nothing to compare against yet

        age_s = (self.get_clock().now() - self.truth_stamp).nanoseconds / 1e9
        if age_s > TRUTH_MAX_AGE_S:
            return  # stale position, the comparison would be meaningless

        if len(msg.points) < 2:
            return  # not the two point ARROW form

        if msg.header.frame_id != self.truth.header.frame_id:
            self.get_logger().warning(
                f"Frame mismatch: {msg.header.frame_id} vs {self.truth.header.frame_id}"
            )
            return

        start, tip = msg.points[0], msg.points[1]
        origin = (start.x, start.y, start.z)
        bearing = (tip.x - start.x, tip.y - start.y, tip.z - start.z)
        truth = (
            self.truth.pose.position.x,
            self.truth.pose.position.y,
            self.truth.pose.position.z,
        )

        # 2d, not 3d: both vessels are on the surface and pitch is the noisy axis
        result = bearing_error_2d(origin, bearing, truth)
        if result is None:
            return
        angle_error_deg, miss_distance_m = result

        t = self.get_clock().now().nanoseconds / 1e9
        range_m = math.hypot(truth[0] - origin[0], truth[1] - origin[1])

        # marker.text carries "track_id,boresight_deg,angle_in_frame_deg" from
        # bearing_marker_ids_node, so the decomposition rides along with the
        # marker it describes instead of needing a second, unsynced topic.
        # Nodes that don't set it (bearing_marker_node) leave these blank.
        track_id, boresight_deg, angle_in_frame_deg = "", "", ""
        if msg.text:
            track_id, boresight_deg, angle_in_frame_deg = msg.text.split(",")

        gimbal_yaw_deg = "" if self.gimbal_yaw_deg is None else f"{self.gimbal_yaw_deg:.3f}"
        corrected_yaw_deg = "" if self.corrected_yaw_deg is None else f"{self.corrected_yaw_deg:.3f}"
        yaw_correction_valid = "" if self.yaw_correction_valid is None else str(bool(self.yaw_correction_valid))
        yaw_sigma_deg = "" if self.yaw_sigma_deg is None else f"{self.yaw_sigma_deg:.3f}"

        self.csv_writer.writerow(
            [
                f"{t:.3f}",
                msg.ns,
                msg.id,
                track_id,
                boresight_deg,
                angle_in_frame_deg,
                gimbal_yaw_deg,
                corrected_yaw_deg,
                yaw_correction_valid,
                yaw_sigma_deg,
                f"{angle_error_deg:.3f}",
                f"{miss_distance_m:.2f}",
                f"{range_m:.2f}",
            ]
        )
        self.csv_file.flush()  # ctrl-c should not lose the run


def main():
    rclpy.init()
    node = BearingErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
