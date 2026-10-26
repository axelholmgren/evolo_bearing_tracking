import json

import rclpy
import tf2_geometry_msgs  # unused by name, registers PointStamped for Buffer.transform
from geographic_msgs.msg import GeoPoint
from rclpy.duration import Duration
from rclpy.node import Node
from smarc_utilities.georef_utils import convert_latlon_to_utm
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

WORLD_FRAME = "evolo/map"


class SmarcduinoWarapsPositionMarkerNode(Node):
    """
    Publishes smarcduino's GPS position as a SPHERE marker.

    Same as smarcduino_marker_node but reads the waraps stream, which is JSON in a String and updates about every 1.3 s instead of every 10 s.

    convert_latlon_to_utm returns a point in utm_<zone>_<band>, which is already
    the root of evolo's tf tree, so one transform() gets it into the map frame.
    """

    def __init__(self):
        super().__init__("smarcduino_waraps_position_marker_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("world_frame", WORLD_FRAME)
        self.world_frame = self.get_parameter("world_frame").value

        self.marker_publisher = self.create_publisher(
            Marker, "/smarcduino/waraps/position_marker", qos_profile=10
        )

        self.subscription = self.create_subscription(
            msg_type=String,
            topic="/smarcduino/waraps/sensor/position",
            callback=self.smarcduino_position_callback,
            qos_profile=10,
        )

    def smarcduino_position_callback(self, msg: String):
        data = json.loads(msg.data)
        geo_point = GeoPoint(
            latitude=data["latitude"],
            longitude=data["longitude"],
            altitude=data["altitude"],
        )
        utm_point = convert_latlon_to_utm(geo_point)

        try:
            map_point = self.tf_buffer.transform(utm_point, self.world_frame)

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {utm_point.header.frame_id} to {self.world_frame}: {ex}"
            )
            return

        # Populate marker
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = map_point.point
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.lifetime = Duration(
            seconds=3
        ).to_msg()  # waraps position updates every ~1.3 s

        self.marker_publisher.publish(marker)


def main():
    rclpy.init()
    node = SmarcduinoWarapsPositionMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
