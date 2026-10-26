#!/usr/bin/env python3
"""Publishes camera bearing to track ids chosen with the track_ids parameter.

Same as bearing_marker_node, except it reads /yolo/tracking instead of
/yolo/target, so it can point at an id yolo_action has not selected.

Track ids are per bag:
    ros2 run ... --ros-args -p track_ids:=[44,68,99]
"""

import math

import rclpy
from geometry_msgs.msg import Point, Vector3, Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from yolo_msgs.msg import DetectionArray

from evolo_bearing.bearing_math import load_camera_info, pixel_to_link_ray

WORLD_FRAME = "evolo/odom"
CAMERA_FRAME = "evolo/z1_camera_link"
CAMERA_APERTURE = 57.1  # same value yolo_action.py uses
RAY_LENGTH = 300
CORRECTION_VALID_TOPIC = "/evolo/gimbal_camera/yaw_correction_valid"
MARKER_COLOR_RAW = (1.0, 0.2, 0.6)
MARKER_COLOR_INVALID = (1.0, 0.0, 0.0)
MARKER_COLOR_VALID = (0.0, 1.0, 0.0)


class BearingRayIdsNode(Node):
    """
    Publishes camera bearing to the chosen track ids.

    Each selected id is visualized as its own ARROW marker.
    """

    def __init__(self):
        super().__init__("bearing_ray_ids_node")

        self.declare_parameter("track_ids", [44, 68, 99])
        self.declare_parameter("camera_frame", CAMERA_FRAME)
        self.camera_frame = self.get_parameter("camera_frame").value
        self.declare_parameter("world_frame", WORLD_FRAME)
        self.world_frame = self.get_parameter("world_frame").value
        # true when camera_frame is a corrected TF: color rays by correction validity
        self.declare_parameter("corrected", False)
        self.corrected = self.get_parameter("corrected").value
        # Must match yolo_action.py camera aperture
        self.declare_parameter("camera_aperture", CAMERA_APERTURE)
        self.camera_aperture = self.get_parameter("camera_aperture").value
        # "" = linear aperture model, otherwise a camera calibration YAML path
        self.declare_parameter("camera_info", "")
        camera_info_file = self.get_parameter("camera_info").value
        self.camera_info = (
            load_camera_info(camera_info_file) if camera_info_file else None
        )
        self.get_logger().info(
            f"Camera model: {camera_info_file or f'linear {self.camera_aperture} deg'}"
        )
        self.correction_valid = False
        self.track_ids = {str(i) for i in self.get_parameter("track_ids").value}
        self.marker_ids = {
            track_id: index
            for index, track_id in enumerate(sorted(self.track_ids, key=int))
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_publisher = self.create_publisher(
            Marker, "/evolo/gimbal_camera/selected_bearing_marker", qos_profile=10
        )
        self.subscription = self.create_subscription(
            msg_type=DetectionArray,
            topic="/yolo/tracking",
            callback=self.tracking_callback,
            qos_profile=10,
        )
        if self.corrected:
            self.create_subscription(
                Bool,
                CORRECTION_VALID_TOPIC,
                self.correction_valid_callback,
                10,
            )
        self.get_logger().info(
            f"Pointing at track ids: {sorted(self.track_ids, key=int)}"
        )

    def correction_valid_callback(self, msg):
        self.correction_valid = msg.data

    def tracking_callback(self, msg: DetectionArray):
        wanted = [det for det in msg.detections if det.id in self.track_ids]
        if not wanted:
            return  # none of the chosen ids in frame, let the marker expire

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.world_frame,
                source_frame=self.camera_frame,
                time=Time.from_msg(msg.header.stamp),
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {self.world_frame} to {self.camera_frame}: {ex}"
            )
            return

        # read before do_transform_vector3, it zeroes the translation
        origin = Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z,
        )

        # camera's own world-frame heading, with no pixel offset applied -- the
        # "boresight" half of the decomposition, same for every detection this callback
        camera_only = do_transform_vector3(
            Vector3Stamped(vector=Vector3(x=1.0, y=0.0, z=0.0)), transform
        ).vector
        boresight_yaw_deg = math.degrees(math.atan2(camera_only.x, camera_only.y))

        for detection in wanted:
            if detection.mask.width <= 0 or detection.mask.height <= 0:
                self.get_logger().warning(
                    f"Ignoring track id {detection.id}: invalid mask dimensions"
                )
                continue

            # bbox center to ray in camera link (x forward)
            ray = pixel_to_link_ray(
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.mask.width,
                detection.mask.height,
                self.camera_aperture,
                self.camera_info,
            )

            forward = Vector3Stamped(vector=Vector3(x=ray[0], y=ray[1], z=ray[2]))

            bearing = do_transform_vector3(forward, transform).vector

            end_point = Point(
                x=origin.x + bearing.x * RAY_LENGTH,
                y=origin.y + bearing.y * RAY_LENGTH,
                z=origin.z + bearing.z * RAY_LENGTH,
            )

            # RViz distinguishes markers by their (namespace, id) pair.
            marker = Marker()
            marker.header.frame_id = self.world_frame
            # marker.header.stamp = self.get_clock().now().to_msg() #NOTE original
            marker.header.stamp = msg.header.stamp
            marker.ns = "selected_bearing_rays"
            marker.id = self.marker_ids[detection.id]
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.points = [origin, end_point]
            # not rendered by ARROW markers -- reused to carry the bearing
            # decomposition out to bearing_error_node without a second topic
            # to keep in sync (id, boresight_yaw_deg, angle_in_frame_deg)
            marker.text = (
                f"{detection.id},{boresight_yaw_deg},"
                f"{math.degrees(math.atan2(ray[1],ray[0]))}"
            )
            marker.scale.x = 0.1  # shaft
            marker.scale.y = 1.0  # head width
            marker.scale.z = 1.0  # head length
            marker.color.a = 1.0
            color = (
                MARKER_COLOR_RAW
                if not self.corrected
                else (
                    MARKER_COLOR_VALID
                    if self.correction_valid
                    else MARKER_COLOR_INVALID
                )
            )
            marker.color.r, marker.color.g, marker.color.b = color
            marker.lifetime = Duration(
                seconds=1
            ).to_msg()  # vanishes when the id leaves frame
            self.marker_publisher.publish(marker)


def main():
    rclpy.init()
    node = BearingRayIdsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
