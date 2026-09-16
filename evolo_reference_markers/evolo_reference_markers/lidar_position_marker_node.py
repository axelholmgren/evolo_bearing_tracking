import json

import rclpy
import tf2_geometry_msgs  # unused by name, registers PointStamped for Buffer.transform
from geographic_msgs.msg import GeoPoint
from rclpy.duration import Duration
from rclpy.node import Node
from smarc_utilities.georef_utils import convert_latlon_to_utm
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

WORLD_FRAME = "evolo/odom"

LIDAR_BBOX = "/bounding_boxes/corrected"

class LidarPositionMarkerNode(Node):
    """
    Publishes lidar marker based on /bounding_boxes/corrected as a SPHERE with 
    poisition in the middle of the bounding box.
    """

    def __init__(self):
        super().__init__("lidar_position_marker_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_publisher = self.create_publisher(
            Marker, "/lidar/position_marker", qos_profile=10
        )

        self.subscription = self.create_subscription(
            msg_type=MarkerArray,
            topic=LIDAR_BBOX,
            callback=self.lidar_position_callback,
            qos_profile=10,
        )

    def lidar_position_callback(self, msg: MarkerArray):
        lidar_box = next(
            (
                marker
                for marker in msg.markers
                if marker.type == Marker.CUBE and marker.id == 0
            ),
            None,
        )
        if lidar_box is None:
            return

        # Populate marker
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = lidar_box.pose.position
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
    node = LidarPositionMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
