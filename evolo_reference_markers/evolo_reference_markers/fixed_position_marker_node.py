#!/usr/bin/env python3
"""Publishes a fixed lat/lon as a SPHERE marker in the map frame.

Position is set with ros parameters so it can be moved without editing this file:
    ros2 run ... --ros-args -p latitude:=59.3006 -p longitude:=18.2206

convert_latlon_to_utm returns a point in utm_<zone>_<band>, which is already the
root of evolo's tf tree, so one transform() gets it into the map frame.
"""

import rclpy
import tf2_geometry_msgs  # unused by name, registers PointStamped for Buffer.transform
from geographic_msgs.msg import GeoPoint
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from smarc_utilities.georef_utils import convert_latlon_to_utm
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

WORLD_FRAME = "evolo/map"
PUBLISH_PERIOD_S = 1.0


class FixedPositionMarkerNode(Node):
    """
    Publishes a fixed lat/lon as a SPHERE marker.
    """

    def __init__(self):
        super().__init__("fixed_position_marker_node")

        # decimal = degrees + minutes/60 + seconds/3600
        self.declare_parameter("latitude", 59.2985134)
        self.declare_parameter("longitude", 18.2146923)
        self.declare_parameter("altitude", 0.0)
        self.declare_parameter("marker_topic", "/fixed_position_marker")

        self.geo_point = GeoPoint(
            latitude=self.get_parameter("latitude").value,
            longitude=self.get_parameter("longitude").value,
            altitude=self.get_parameter("altitude").value,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_publisher = self.create_publisher(
            Marker,
            self.get_parameter("marker_topic").value,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Retry until TF is available, then retain the marker through QoS.
        self.timer = self.create_timer(PUBLISH_PERIOD_S, self.publish_marker)

        self.get_logger().info(
            f"Fixed position: {self.geo_point.latitude}, {self.geo_point.longitude}"
        )

    def publish_marker(self):
        utm_point = convert_latlon_to_utm(self.geo_point)

        try:
            map_point = self.tf_buffer.transform(utm_point, WORLD_FRAME)

        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {utm_point.header.frame_id} to {WORLD_FRAME}: {ex}",
                throttle_duration_sec=5.0,
            )
            return

        # Populate marker
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = map_point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.lifetime = Duration(
            seconds=0
        ).to_msg()  # 0 = never expires, the point cannot move

        self.marker_publisher.publish(marker)
        self.timer.cancel()


def main():
    rclpy.init()
    node = FixedPositionMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
