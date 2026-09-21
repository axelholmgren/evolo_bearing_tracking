import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from .gimbal_yaw_correction import correct_yaw


CORRECTION_VALID_TOPIC = "/evolo/gimbal_camera/yaw_correction_valid"


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def yaw_quaternion(yaw):
    return Quaternion(
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def multiply_quaternions(left, right):
    return Quaternion(
        x=left.w * right.x + left.x * right.w + left.y * right.z
        - left.z * right.y,
        y=left.w * right.y - left.x * right.z + left.y * right.w
        + left.z * right.x,
        z=left.w * right.z + left.x * right.y - left.y * right.x
        + left.z * right.w,
        w=left.w * right.w - left.x * right.x - left.y * right.y
        - left.z * right.z,
    )


class CalibratedCameraTFNode(Node):
    """
    Raw TF -> correction curve -> corrected TF.

    Leaves the raw TF tree unchanged and publishes a separate
    corrected camera frame.
    """

    def __init__(self):
        super().__init__("corrected_camera_tf_node")

        self.declare_parameter("base_frame", "evolo/z1_base_link")
        self.declare_parameter("yaw_frame", "evolo/z1_yaw_link")
        self.declare_parameter("camera_frame", "evolo/z1_camera_link")
        self.declare_parameter(
            "corrected_camera_frame",
            "evolo/z1_camera_corrected_link",
        )

        self.declare_parameter("yaw_correction_mode", "absolute")
        self.declare_parameter("negate_yaw_correction", False)

        self.base_frame = self.get_parameter("base_frame").value
        self.yaw_frame = self.get_parameter("yaw_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.corrected_camera_frame = self.get_parameter(
            "corrected_camera_frame"
        ).value

        self.correction_mode = self.get_parameter(
            "yaw_correction_mode"
        ).value
        self.negate_correction = self.get_parameter(
            "negate_yaw_correction"
        ).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.valid_publisher = self.create_publisher(
            Bool, CORRECTION_VALID_TOPIC, 10
        )

        self.create_subscription(
            TFMessage,
            "/tf",
            self.tf_callback,
            qos_profile_sensor_data,
        )

    def tf_callback(self, msg):
        base_to_yaw = next(
            (
                tf
                for tf in msg.transforms
                if tf.header.frame_id == self.base_frame
                and tf.child_frame_id == self.yaw_frame
            ),
            None,
        )

        if base_to_yaw is None:
            return

        self.publish_corrected_transform(base_to_yaw)

    def publish_corrected_transform(self, base_to_yaw):
        stamp = Time.from_msg(base_to_yaw.header.stamp)

        try:
            yaw_to_camera = self.tf_buffer.lookup_transform(
                self.yaw_frame,
                self.camera_frame,
                stamp,
            )
        except TransformException:
            return

        raw_yaw_deg = math.degrees(
            yaw_from_quaternion(base_to_yaw.transform.rotation)
        )

        result = correct_yaw(
            raw_yaw_deg,
            mode=self.correction_mode,
            negate=self.negate_correction,
        )

        valid = bool(result.valid)
        self.valid_publisher.publish(Bool(data=valid))
        delta_yaw = (
            math.radians(float(result.yaw_deg) - raw_yaw_deg)
            if valid
            else 0.0
        )

        delta_rotation = yaw_quaternion(delta_yaw)
        translation = yaw_to_camera.transform.translation

        corrected = TransformStamped()
        corrected.header.stamp = base_to_yaw.header.stamp
        corrected.header.frame_id = self.yaw_frame
        corrected.child_frame_id = self.corrected_camera_frame

        c = math.cos(delta_yaw)
        s = math.sin(delta_yaw)

        corrected.transform.translation.x = (
            c * translation.x - s * translation.y
        )
        corrected.transform.translation.y = (
            s * translation.x + c * translation.y
        )
        corrected.transform.translation.z = translation.z

        corrected.transform.rotation = multiply_quaternions(
            delta_rotation,
            yaw_to_camera.transform.rotation,
        )

        self.tf_broadcaster.sendTransform(corrected)


def main():
    rclpy.init()

    node = CalibratedCameraTFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
