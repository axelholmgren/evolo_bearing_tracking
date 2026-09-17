import rclpy
from geometry_msgs.msg import (
    Point,
    QuaternionStamped,
    Transform,
    TransformStamped,
    Vector3,
    Vector3Stamped,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from z1_pro_msgs.msg import Gcudata

from evolo_gimbal_calibration.gimbal_yaw_correction import correct_yaw

from .bearing_math import rotate_bearing_xy

# WORLD_FRAME = "evolo/map"
WORLD_FRAME = "evolo/odom"
RAY_LENGTH = 300  # Arbitrary ray length for visualization
MARKER_COLOR_CORRECTION_DISABLED = (1.0, 0.2, 0.6)
MARKER_COLOR_CORRECTION_INVALID = (1.0, 0.0, 0.0)
MARKER_COLOR_YAW_CORRECTION_ACTIVE = (0.0, 1.0, 0.0)


class BearingRayNode(Node):
    """
    Publishes camera bearing to the currently tracked target.

    Visualization as an ARROW marker.

    tracked_poi_image is a small rotation off the camera boresight, from
    yolo_action.py's detection bbox center. Not a world frame bearing on its
    own, it gets composed with the camera's TF orientation to get a real
    direction.
    """

    def __init__(self):
        super().__init__("bearing_ray_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_publisher = self.create_publisher(
            Marker, "/evolo/gimbal_camera/target_bearing_marker", qos_profile=10
        )
        self.declare_parameter(
            "gimbal_gcu_feedback_topic", "/evolo/gimbal_camera/gimbal_gcu_fb"
        )
        self.declare_parameter("apply_yaw_correction", True)
        self.declare_parameter("yaw_correction_mode", "absolute")
        self.declare_parameter("negate_yaw_correction", False)
        self.yaw_correction_valid = False
        self.yaw_correction_deg = 0.0

        self.subscription = self.create_subscription(
            msg_type=QuaternionStamped,
            topic="/evolo/gimbal_camera/tracked_poi_image",
            callback=self.poi_callback,
            qos_profile=10,
        )
        self.gimbal_subscription = self.create_subscription(
            msg_type=Gcudata,
            topic=self.get_parameter("gimbal_gcu_feedback_topic").value,
            callback=self.gimbal_callback,
            qos_profile=10,
        )

    def gimbal_callback(self, msg: Gcudata):
        result = correct_yaw(
            msg.relative_yaw,
            mode=self.get_parameter("yaw_correction_mode").value,
            negate=self.get_parameter("negate_yaw_correction").value,
        )
        self.yaw_correction_valid = bool(result.valid)
        self.yaw_correction_deg = (
            float(result.yaw_deg - msg.relative_yaw)
            if self.yaw_correction_valid
            else 0.0
        )

    def poi_callback(self, msg: QuaternionStamped):
        correction_active = (
            self.get_parameter("apply_yaw_correction").value
            and self.yaw_correction_valid
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=WORLD_FRAME,
                source_frame=msg.header.frame_id,
                time=Time(),
            ) #NOTE original using latest time

           # transform = self.tf_buffer.lookup_transform(
           #      target_frame=WORLD_FRAME,
           #      source_frame=msg.header.frame_id,
           #      time=Time.from_msg(msg.header.stamp),
           #  ) #NOTE perserve time 

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {WORLD_FRAME} to {msg.header.frame_id}: {ex}"
            )
            return

        # Must be set before do_transform_vector3() as it overwrites
        # transform.transform.translation with (0,0,0)
        origin = Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            # z=0,
            z=transform.transform.translation.z,
        )

        # x is forward in camera link (not optical frame)
        forward = Vector3Stamped(vector=Vector3(x=1.0, y=0.0, z=0.0))

        # Quaternion for offset, wrapping for do_transform_vector3()
        offset_transform = TransformStamped(
            transform=Transform(rotation=msg.quaternion)
        )

        # offset -> target frame direction -> map frame
        target_frame_direction = do_transform_vector3(forward, offset_transform)
        bearing_vector = do_transform_vector3(target_frame_direction, transform)

        bearing_x = bearing_vector.vector.x
        bearing_y = bearing_vector.vector.y
        bearing_z = bearing_vector.vector.z
        if correction_active:
            bearing_x, bearing_y = rotate_bearing_xy(
                bearing_x,
                bearing_y,
                self.yaw_correction_deg,
            )

        end_point = Point(
            x=origin.x + bearing_x * RAY_LENGTH,
            y=origin.y + bearing_y * RAY_LENGTH,
            # z=0,
            z=origin.z + bearing_z * RAY_LENGTH,
        )

        # Populate marker
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [origin, end_point]
        marker.scale.x = 0.1  # ray size
        marker.scale.y = 1.0  # point width
        marker.scale.z = 1.0  # point length
        marker.color.a = 1.0
        if not self.get_parameter("apply_yaw_correction").value:
            color = MARKER_COLOR_CORRECTION_DISABLED
        elif correction_active:
            color = MARKER_COLOR_YAW_CORRECTION_ACTIVE
        else:
            color = MARKER_COLOR_CORRECTION_INVALID
        marker.color.r, marker.color.g, marker.color.b = color
        marker.lifetime = Duration(seconds=1).to_msg()

        self.marker_publisher.publish(marker)


def main():
    rclpy.init()
    node = BearingRayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
