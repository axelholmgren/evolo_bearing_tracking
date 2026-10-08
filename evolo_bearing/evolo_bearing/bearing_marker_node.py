import rclpy
import math
from evolo_msgs.msg import BearingObservation
from geometry_msgs.msg import (Point, QuaternionStamped, Transform,
                               TransformStamped, Vector3, Vector3Stamped)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor  # NOTE: for waiting on tf
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

# WORLD_FRAME = "evolo/map"
WORLD_FRAME = "evolo/odom"
RAY_LENGTH = 300  # Arbitrary ray length for visualization
CORRECTED_CAMERA_FRAME = "evolo/z1_camera_corrected_link"
CORRECTION_VALID_TOPIC = "/evolo/gimbal_camera/yaw_correction_valid"
MARKER_COLOR_RAW = (1.0, 0.2, 0.6)
MARKER_COLOR_INVALID = (1.0, 0.0, 0.0)
MARKER_COLOR_VALID = (0.0, 1.0, 0.0)


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
        self.observation_publisher = self.create_publisher(
            BearingObservation,
            "/evolo/gimbal_camera/target_bearing_observation",
            qos_profile=10,
        )

        self.declare_parameter("camera_frame", "evolo/z1_camera_link")
        self.camera_frame = self.get_parameter("camera_frame").value
        self.declare_parameter("keep_ray_history", False)
        self.keep_ray_history = self.get_parameter("keep_ray_history").value
        self.declare_parameter("tf_timeout", 0.15)
        self.tf_timeout = self.get_parameter("tf_timeout").value

        self.next_marker_id = 0
        self.correction_valid = False

        self.subscription = self.create_subscription(
            msg_type=QuaternionStamped,
            topic="/evolo/gimbal_camera/tracked_poi_image",
            callback=self.poi_callback,
            qos_profile=10,
        )

        if self.camera_frame == CORRECTED_CAMERA_FRAME:
            self.create_subscription(
                Bool,
                CORRECTION_VALID_TOPIC,
                self.correction_valid_callback,
                10,
            )

    def correction_valid_callback(self, msg):
        self.correction_valid = msg.data

    def poi_callback(self, msg: QuaternionStamped):
        try:
            # transform = self.tf_buffer.lookup_transform(
            #     target_frame=WORLD_FRAME,
            #     source_frame=msg.header.frame_id,
            #     time=Time(),
            # ) #NOTE original using latest time

            # transform = self.tf_buffer.lookup_transform(
            #      target_frame=WORLD_FRAME,
            #      source_frame=msg.header.frame_id,
            #      time=Time.from_msg(msg.header.stamp),
            #  ) #NOTE perserve time
            transform = self.tf_buffer.lookup_transform(
                target_frame=WORLD_FRAME,
                source_frame=self.camera_frame,
                time=Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )  # NOTE wait for tf

        except TransformException as ex:
            self.get_logger().warning(
                f"Dropping bearing: TF unavailable at observation time "
                f"for {self.camera_frame} -> {WORLD_FRAME}: {ex}"
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
        
        direction = bearing_vector.vector

        # Add to the bearing observation message
        observation = BearingObservation()
        observation.header.stamp = msg.header.stamp
        observation.header.frame_id = WORLD_FRAME
        observation.origin = origin
        observation.direction = direction
        observation.bearing = math.atan2(direction.y, direction.x)
        self.observation_publisher.publish(observation)

        end_point = Point(
            x=origin.x + bearing_vector.vector.x * RAY_LENGTH,
            y=origin.y + bearing_vector.vector.y * RAY_LENGTH,
            # z=0,
            z=origin.z + bearing_vector.vector.z * RAY_LENGTH,
        )

        # Populate marker
        marker = Marker()
        marker.header.frame_id = WORLD_FRAME
        # marker.header.stamp = self.get_clock().now().to_msg() #NOTE original
        marker.header.stamp = msg.header.stamp  # NOTE change to output the input stamp
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        if self.keep_ray_history:
            # ponytail: RViz retains every ray; cap IDs or set a lifetime if memory grows.
            marker.id = self.next_marker_id
            self.next_marker_id += 1
        marker.points = [origin, end_point]
        marker.scale.x = 0.05  # ray size
        marker.scale.y = 1.0  # point width
        marker.scale.z = 1.0  # point length
        marker.color.a = 1.0
        color = (
            MARKER_COLOR_RAW
            if self.camera_frame != CORRECTED_CAMERA_FRAME
            else MARKER_COLOR_VALID if self.correction_valid else MARKER_COLOR_INVALID
        )
        marker.color.r, marker.color.g, marker.color.b = color
        marker.lifetime = Duration(seconds=0 if self.keep_ray_history else 1).to_msg()

        self.marker_publisher.publish(marker)


def main():
    # NOTE: original
    # rclpy.init()
    # node = BearingRayNode()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

    # NOTE: add multithread to be able to wait for tf
    rclpy.init()
    node = BearingRayNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
