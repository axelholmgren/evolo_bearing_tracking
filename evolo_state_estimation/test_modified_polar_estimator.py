"""Run offline: python -m unittest discover -s evolo_state_estimation -p test_modified_polar_estimator.py."""

from datetime import datetime, timedelta, timezone
import math
import unittest

import numpy as np

from modified_polar_estimator import ModifiedPolarEKF, to_cartesian, to_modified_polar


class ModifiedPolarTest(unittest.TestCase):
    def test_ukf_moving_camera_and_bearing_wrap(self):
        estimator = ModifiedPolarEKF(500., 100., 5., 1., 0.1, method="ukf")
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        target = np.array([-500., 0.])
        bearings = []
        for k in range(30):
            origin = np.array([float(k), 10 * math.sin(k / 5)])
            bearing = math.atan2(*(target - origin)[::-1])
            bearings.append(bearing)
            x, P = estimator.step(start + timedelta(seconds=k), bearing, origin)
            self.assertTrue(np.isfinite(x).all())
            self.assertTrue(np.isfinite(P).all())
            self.assertGreater(estimator.y[3], 0)
            np.testing.assert_allclose(P, P.T, atol=1e-8)
            self.assertGreaterEqual(np.linalg.eigvalsh(P).min(), -1e-8)
            residual = math.atan2(math.sin(estimator.y[2] - bearing),
                                  math.cos(estimator.y[2] - bearing))
            self.assertLess(abs(residual), 0.02)
            self.assertLess(np.linalg.norm(x[:2] - target), 50.)
        self.assertLess(min(bearings), -3.)
        self.assertGreater(max(bearings), 3.)

    def test_ukf_rejects_nonpositive_sigma_range_without_changing_state(self):
        estimator = ModifiedPolarEKF(50., 500., 1., 1., 0.1, method="ukf")
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        estimator.step(start, 0., [0., 0.])
        previous_y, previous_P = estimator.y.copy(), estimator.P.copy()
        with self.assertRaisesRegex(ValueError, "Inverse range must be positive"):
            estimator.step(start + timedelta(seconds=1), 0., [0., 0.])
        np.testing.assert_array_equal(estimator.y, previous_y)
        np.testing.assert_array_equal(estimator.P, previous_P)
        self.assertEqual(estimator.timestamp, start)

    def test_transforms_and_jacobians(self):
        origin = np.array([10., -20.])
        velocity = np.array([2., 1.])
        x = np.array([310., 380., -3., 4.])
        y, J = to_modified_polar(x, origin, velocity)
        restored, J_inverse = to_cartesian(y, origin, velocity)
        np.testing.assert_allclose(restored, x)
        np.testing.assert_allclose(J @ J_inverse, np.eye(4), atol=1e-12)
        for function, state, jacobian in [(to_modified_polar, x, J), (to_cartesian, y, J_inverse)]:
            numerical = np.empty((4, 4))
            for index in range(4):
                delta = np.zeros(4)
                delta[index] = 1e-7
                numerical[:, index] = (function(state + delta, origin, velocity)[0]
                                       - function(state - delta, origin, velocity)[0]) / 2e-7
            np.testing.assert_allclose(jacobian, numerical, rtol=1e-6, atol=1e-6)

    def test_moving_camera_wrap_covariance_and_rejection(self):
        estimator = ModifiedPolarEKF(500., 100., 5., 1., 0.1)
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        target = np.array([-500., 0.])
        for k in range(50):
            origin = np.array([float(k), 10 * math.sin(k / 10)])
            bearing = math.atan2(*(target - origin)[::-1])
            x, P = estimator.step(start + timedelta(seconds=k), bearing, origin)
            np.testing.assert_allclose(x[:2], target, atol=1e-8)
            np.testing.assert_allclose(x[2:], 0, atol=1e-8)
            np.testing.assert_allclose(P, P.T, atol=1e-9)
            self.assertGreaterEqual(np.linalg.eigvalsh(P).min(), -1e-8)
        previous = estimator.y.copy()
        for timestamp, bearing, origin in [(start, 0., [0., 0.]),
                                            (start + timedelta(seconds=50), math.nan, [0., 0.])]:
            with self.assertRaises(ValueError):
                estimator.step(timestamp, bearing, origin)
            np.testing.assert_array_equal(estimator.y, previous)

    def test_prediction_noise_and_linear_bearing_update(self):
        estimator = ModifiedPolarEKF(500., 100., 5., 1., 0.1)
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        x, P_x = estimator.step(start, 0., [0., 0.])
        np.testing.assert_allclose(estimator.y, [0., 0., 0., 1 / 500])
        dt = 2.
        origin = np.array([5., 3.])
        velocity = origin / dt
        F = np.eye(4)
        F[:2, 2:] = dt * np.eye(2)
        Q = 0.1 * np.block([[dt**3 / 3 * np.eye(2), dt**2 / 2 * np.eye(2)],
                           [dt**2 / 2 * np.eye(2), dt * np.eye(2)]])
        predicted_y, J = to_modified_polar(F @ x, origin, velocity)
        predicted_P = J @ (F @ P_x @ F.T + Q) @ J.T
        K = predicted_P[:, 2] / (predicted_P[2, 2] + estimator.bearing_std**2)
        estimator.step(start + timedelta(seconds=dt), predicted_y[2] + 0.01, origin)
        np.testing.assert_allclose(estimator.y, predicted_y + K * 0.01)
        np.testing.assert_allclose(estimator.P, predicted_P - np.outer(K, predicted_P[2, :]), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
