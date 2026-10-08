"""Sim-only Z1 feedback: echo gimbal_cmd as Gcudata.

Unity's Gimbal.cs applies commands instantly, so the last command is the
actual angle. gimbal_action.py image-POI tracking needs this feedback.
"""

import rclpy
from geometry_msgs.msg import Vector3
from z1_pro_msgs.msg import Gcudata


def main():
    rclpy.init()
    node = rclpy.create_node("sim_gimbal_feedback_node")
    publisher = node.create_publisher(
        Gcudata, "/evolo/gimbal_camera/gimbal_gcu_fb", 10
    )
    node.create_subscription(
        Vector3,
        "/evolo/gimbal_camera/gimbal_cmd",
        lambda cmd: publisher.publish(Gcudata(
            relative_roll=cmd.x, relative_pitch=cmd.y, relative_yaw=cmd.z,
        )),
        10,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
