import math
from datetime import datetime, timedelta, timezone

import numpy as np
import rclpy
from cartesian_estimator import CartesianEKF
from evolo_msgs.msg import BearingObservation, TargetStateStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from modified_polar_estimator import ModifiedPolarEKF
from rclpy.node import Node
from rclpy.time import Time
from stonesoup.updater.kalman import UnscentedKalmanUpdater
from visualization_msgs.msg import Marker, MarkerArray


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__("state_estimator_node")
        # Select at startup; both estimators return [x, y, vx, vy].
        self.declare_parameter(
            "coordinates", "modified_polar"
        )  # modified polar/cartesian
        self.declare_parameter("method", "ekf")  # ekf/ukf

        self.declare_parameter("initial_range", 100.0)  # meters
        self.declare_parameter("initial_range_std", 100.0)  # meters
        self.declare_parameter("initial_velocity_std", 0.01)  # m/s
        self.declare_parameter("bearing_std", 1.0)  # deg
        self.declare_parameter("acceleration", 0.001)  # m^2/s^3

        self.initial_range = self.get_parameter("initial_range").value
        self.initial_range_std = self.get_parameter("initial_range_std").value
        self.initial_velocity_std = self.get_parameter("initial_velocity_std").value
        self.bearing_std = self.get_parameter("bearing_std").value
        self.acceleration = self.get_parameter("acceleration").value

        coordinates = self.get_parameter("coordinates").value
        method = self.get_parameter("method").value
        self.estimator: CartesianEKF | ModifiedPolarEKF
        if coordinates == "cartesian" and method == "ekf":
            self.estimator = CartesianEKF(
                self.initial_range,
                self.initial_range_std,
                self.initial_velocity_std,
                self.bearing_std,
                self.acceleration,
            )
        elif coordinates == "cartesian" and method == "ukf":
            self.estimator = CartesianEKF(
                self.initial_range,
                self.initial_range_std,
                self.initial_velocity_std,
                self.bearing_std,
                self.acceleration,
                updater_type=UnscentedKalmanUpdater,
            )
        elif coordinates == "modified_polar" and method in ("ekf", "ukf"):
            self.estimator = ModifiedPolarEKF(
                self.initial_range,
                self.initial_range_std,
                self.initial_velocity_std,
                self.bearing_std,
                self.acceleration,
                method=method,
            )
        else:
            raise ValueError(
                f"Unsupported estimator: coordinates={coordinates}, method={method}"
            )

        self.EPOCH = datetime(1970, 1, 1, tzinfo=timezone.utc)  # UNIX/EPOCH time

        self.publisher = self.create_publisher(TargetStateStamped, "~/target_state", 10)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "~/target_pose", 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "~/estimate_markers", 10
        )
        self.subscription = self.create_subscription(
            BearingObservation,
            "/evolo/gimbal_camera/target_bearing_observation",
            self.bearing_callback,
            10,
        )

    def bearing_callback(self, msg):
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds

        # StoneSoup only accepts datetime
        timestamp = self.EPOCH + timedelta(
            microseconds=stamp_ns // 1000
        )  # python datetime does not support nanoseconds
        # TODO: Add frame and time stamp checks (logger() + return)

        try:
            state, covariance = self.estimator.step(
                timestamp, msg.bearing, [msg.origin.x, msg.origin.y]
            )  # NOTE: placeholder

        except ValueError as error:  # TODO:extend with error possible thrown
            self.get_logger().warning(f"Dropping estimator update: {error}")
            return

        output = TargetStateStamped()

        output.header = msg.header
        output.position.x = float(state[0])
        output.position.y = float(state[1])
        output.velocity.x = float(state[2])
        output.velocity.y = float(state[3])
        output.covariance = covariance.reshape(16).tolist()
        self.publisher.publish(output)
        self.publish_pose(output)
        self.publish_markers(output)

    def publish_pose(self, output):
        pose = PoseWithCovarianceStamped()
        pose.header = output.header
        pose.pose.pose.position = output.position
        pose.pose.pose.orientation.w = 1.0
        # Pose covariance is [x, y, z, roll, pitch, yaw]; velocity is excluded.
        covariance = np.zeros((6, 6))
        covariance[:2, :2] = np.asarray(output.covariance).reshape(4, 4)[:2, :2]
        pose.pose.covariance = covariance.reshape(36).tolist()
        self.pose_publisher.publish(pose)

    def publish_markers(self, output):
        position = Marker()
        position.header = output.header
        position.id = 0
        position.type = Marker.SPHERE
        position.pose.position = output.position
        position.pose.orientation.w = 1.0
        position.scale.x = position.scale.y = position.scale.z = 2.0
        position.color.g = position.color.b = 0.5
        position.color.a = 1.0

        velocity = Marker()
        velocity.header = output.header
        velocity.id = 1
        velocity.type = Marker.ARROW
        velocity.pose.orientation.w = 1.0
        velocity.points = [
            output.position,
            Point(
                x=output.position.x + 5.0 * output.velocity.x,
                y=output.position.y + 5.0 * output.velocity.y,
                z=output.position.z,
            ),
        ]
        velocity.scale.x = 0.5
        velocity.scale.y = 1.0
        velocity.color.b = velocity.color.a = 1.0
        if math.hypot(output.velocity.x, output.velocity.y) < 1e-6:
            velocity.action = Marker.DELETE

        self.marker_publisher.publish(MarkerArray(markers=[position, velocity]))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateEstimatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
