"""Offline check; no ROS graph or estimator initialization required."""

from types import SimpleNamespace
from datetime import datetime, timedelta, timezone
import unittest
from unittest.mock import patch, DEFAULT

import numpy as np
from evolo_msgs.msg import TargetStateStamped
from visualization_msgs.msg import Marker

from state_estimator_node import StateEstimatorNode
from cartesian_estimator import CartesianEKF
from modified_polar_estimator import ModifiedPolarEKF
from rclpy.node import Node
from stonesoup.updater.kalman import UnscentedKalmanUpdater


class EstimateMarkersTest(unittest.TestCase):
    def test_estimator_selection_without_ros_graph(self):
        for coordinates, method, expected in [
            ("cartesian", "ekf", CartesianEKF),
            ("cartesian", "ukf", CartesianEKF),
            ("modified_polar", "ekf", ModifiedPolarEKF),
            ("modified_polar", "ukf", ModifiedPolarEKF),
            ("modified_polar", "particle", None),
            ("unknown", "ekf", None),
        ]:
            parameters = {"coordinates": coordinates, "method": method}
            with patch.object(Node, "__init__", return_value=None), patch.multiple(
                StateEstimatorNode, declare_parameter=DEFAULT, get_parameter=DEFAULT,
                create_publisher=DEFAULT, create_subscription=DEFAULT,
            ) as mocks:
                mocks["declare_parameter"].side_effect = lambda name, value, *args: parameters.setdefault(name, value)
                mocks["get_parameter"].side_effect = lambda name: SimpleNamespace(value=parameters[name])
                if expected is None:
                    with self.assertRaisesRegex(ValueError, "Unsupported estimator"):
                        StateEstimatorNode()
                else:
                    estimator = StateEstimatorNode().estimator
                    self.assertIsInstance(estimator, expected)
                    if coordinates == "modified_polar":
                        self.assertEqual(estimator.method, method)
                    elif method == "ukf":
                        self.assertIsInstance(estimator.updater, UnscentedKalmanUpdater)

    def test_ukf_bearing_wrap_and_covariance(self):
        estimator = CartesianEKF(500., 100., 5., 1., 0.1,
                                 updater_type=UnscentedKalmanUpdater)
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        estimator.step(start, np.pi - 0.001, [0., 0.])
        state, covariance = estimator.step(
            start + timedelta(seconds=1), -np.pi + 0.001, [0., 0.])
        self.assertTrue(np.isfinite(state).all())
        self.assertLess(state[0], -400.)
        self.assertLess(abs(state[1]), 10.)
        np.testing.assert_allclose(covariance, covariance.T, atol=1e-9)
        self.assertGreaterEqual(np.linalg.eigvalsh(covariance).min(), -1e-9)
        previous = estimator.state
        with self.assertRaises(ValueError):
            estimator.step(start, 0., [0., 0.])
        self.assertIs(estimator.state, previous)

    def test_estimator_initialization_update_and_time(self):
        estimator = CartesianEKF(500.0, 200.0, 5.0, 1.0, 0.1)
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        state, covariance = estimator.step(start, 0.0, [10.0, 20.0])
        np.testing.assert_allclose(state, [510.0, 20.0, 0.0, 0.0])
        np.testing.assert_allclose(covariance, np.diag([40000.0, 40000.0, 25.0, 25.0]))
        estimator.state.state_vector[1, 0] = 2.0
        estimator.state.state_vector[3, 0] = -1.0
        state, covariance = estimator.step(
            start + timedelta(seconds=1), 0.0, [12.0, 19.0]
        )
        np.testing.assert_allclose(state, [512.0, 19.0, 2.0, -1.0])
        np.testing.assert_allclose(covariance, covariance.T)
        self.assertGreaterEqual(np.linalg.eigvalsh(covariance).min(), -1e-9)
        previous = estimator.state
        with self.assertRaises(ValueError):
            estimator.step(start, 0.0, [10.0, 20.0])
        self.assertIs(estimator.state, previous)

    def test_position_velocity_and_stopped_target(self):
        published = []
        node = SimpleNamespace(
            marker_publisher=SimpleNamespace(publish=published.append)
        )
        state = TargetStateStamped()
        state.header.frame_id = "test_frame"
        state.header.stamp.sec = 123
        state.position.x, state.position.y = 10.0, 20.0
        state.velocity.x, state.velocity.y = 2.0, -1.0

        StateEstimatorNode.publish_markers(node, state)
        sphere, arrow = published[-1].markers
        self.assertEqual(sphere.type, Marker.SPHERE)
        self.assertEqual(sphere.pose.position, state.position)
        self.assertEqual(sphere.header, state.header)
        self.assertEqual((sphere.color.r, sphere.color.g, sphere.color.b,
                          sphere.color.a), (0.0, 0.5, 0.5, 1.0))
        self.assertEqual(arrow.type, Marker.ARROW)
        self.assertEqual(arrow.header, state.header)
        self.assertNotEqual(sphere.id, arrow.id)
        self.assertEqual(arrow.points[0], state.position)
        self.assertEqual((arrow.points[1].x, arrow.points[1].y), (20.0, 15.0))
        self.assertEqual(arrow.action, Marker.ADD)

        state.velocity.x = state.velocity.y = 0.0
        StateEstimatorNode.publish_markers(node, state)
        self.assertEqual(published[-1].markers[0].action, Marker.ADD)
        self.assertEqual(published[-1].markers[1].action, Marker.DELETE)

    def test_pose_covariance_mapping(self):
        published = []
        node = SimpleNamespace(
            pose_publisher=SimpleNamespace(publish=published.append)
        )
        state = TargetStateStamped()
        state.header.frame_id = "test_frame"
        state.position.x, state.position.y = 10.0, 20.0
        covariance = np.array([[5.0, 4.0], [4.0, 5.0]])
        full_covariance = np.diag([5.0, 5.0, 25.0, 36.0])
        full_covariance[:2, :2] = covariance
        state.covariance = full_covariance.reshape(16).tolist()

        StateEstimatorNode.publish_pose(node, state)
        pose = published[-1]
        self.assertEqual(pose.header, state.header)
        self.assertEqual(pose.pose.pose.position, state.position)
        self.assertEqual(pose.pose.pose.orientation.w, 1.0)
        expected = np.zeros((6, 6))
        expected[:2, :2] = covariance
        np.testing.assert_allclose(np.asarray(pose.pose.covariance).reshape(6, 6), expected)

        state.covariance = [0.0] * 16
        StateEstimatorNode.publish_pose(node, state)
        np.testing.assert_allclose(published[-1].pose.covariance, np.zeros(36))


if __name__ == "__main__":
    unittest.main()
