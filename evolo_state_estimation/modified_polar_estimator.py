"""Pre_study.tex MPC state: y = [beta_dot, r_h_dot/r_h, beta, 1/r_h]."""

import math

import numpy as np
from stonesoup.functions import gauss2sigma
from stonesoup.types.state import GaussianState


def to_modified_polar(x, origin, camera_velocity):
    """Return f_MPC(x) and its Jacobian with respect to x."""
    d = x[:2] - origin
    v_rel = x[2:] - camera_velocity
    r_h = np.linalg.norm(d)
    if r_h <= 0:
        raise ValueError("Camera-target range must be positive")
    e = d / r_h
    e_perp = np.array([-e[1], e[0]])
    beta_dot = e_perp @ v_rel / r_h
    range_rate = e @ v_rel / r_h
    y = np.array([beta_dot, range_rate, math.atan2(d[1], d[0]), 1 / r_h])
    J = np.zeros((4, 4))
    J[0, :2] = (-beta_dot * e - range_rate * e_perp) / r_h
    J[0, 2:] = e_perp / r_h
    J[1, :2] = (-range_rate * e + beta_dot * e_perp) / r_h
    J[1, 2:] = e / r_h
    J[2, :2] = e_perp / r_h
    J[3, :2] = -e / r_h**2
    return y, J


def to_cartesian(y, origin, camera_velocity):
    """Return f_MPC inverse and its Jacobian with respect to y."""
    beta_dot, range_rate, beta, inverse_range = y
    if inverse_range <= 0:
        raise ValueError("Inverse range must be positive")
    r_h = 1 / inverse_range
    e = np.array([math.cos(beta), math.sin(beta)])
    e_perp = np.array([-e[1], e[0]])
    d = r_h * e
    v_rel = r_h * (range_rate * e + beta_dot * e_perp)
    x = np.concatenate((origin + d, camera_velocity + v_rel))
    J = np.zeros((4, 4))
    J[:2, 2] = r_h * e_perp
    J[:2, 3] = -r_h * d
    J[2:, 0] = r_h * e_perp
    J[2:, 1] = d
    J[2:, 2] = r_h * (range_rate * e_perp - beta_dot * e)
    J[2:, 3] = -r_h * v_rel
    return x, J


class ModifiedPolarEKF:
    def __init__(self, initial_range, initial_range_std, initial_velocity_std,
                 bearing_std, acceleration, method="ekf"):
        if method not in ("ekf", "ukf"):
            raise ValueError("Method must be ekf or ukf")
        self.method = method
        values = [initial_range, initial_range_std, initial_velocity_std, bearing_std, acceleration]
        if not all(math.isfinite(v) for v in values) or min(values) < 0 or initial_range == 0 or bearing_std == 0:
            raise ValueError("Invalid EKF range or noise parameters")
        self.initial_range = initial_range
        self.range_std = initial_range_std
        self.velocity_std = initial_velocity_std
        self.bearing_std = math.radians(bearing_std)
        self.acceleration = acceleration  # Cartesian acceleration PSD, m²/s³.
        self.y = None
        self.P = None
        self.timestamp = None
        self.origin = None
        self.camera_velocity = None

    def step(self, timestamp, bearing, origin):
        """Return navigation-frame [x, y, vx, vy] and covariance."""
        origin = np.asarray(origin, dtype=float)
        if origin.shape != (2,) or not np.isfinite(origin).all() or not math.isfinite(bearing):
            raise ValueError("Bearing and camera position must be finite")
        if self.timestamp is not None and timestamp <= self.timestamp:
            raise ValueError("Observation time must increase")
        R = self.bearing_std**2

        # Initialize on the bearing ray, with zero target and camera velocity.
        if self.y is None:
            r_h = self.initial_range
            camera_velocity = np.zeros(2)
            y = np.array([0., 0., bearing, 1 / r_h])
            P = np.diag([(self.velocity_std / r_h)**2,
                         (self.velocity_std / r_h)**2,
                         R, (self.range_std / r_h**2)**2])
        else:
            dt = (timestamp - self.timestamp).total_seconds()
            # Camera velocity is approximated from successive observations.
            camera_velocity = (origin - self.origin) / dt

            # Predict: y_{k+1} = f_MPC(F_k f_MPC_inverse(y_k)).
            x, J_inverse = to_cartesian(self.y, self.origin, self.camera_velocity)
            F = np.eye(4)
            F[:2, 2:] = dt * np.eye(2)
            Q = self.acceleration * np.block([
                [dt**3 / 3 * np.eye(2), dt**2 / 2 * np.eye(2)],
                [dt**2 / 2 * np.eye(2), dt * np.eye(2)],
            ])
            y, J = to_modified_polar(F @ x, origin, camera_velocity)
            if self.method == "ekf":
                F_MPC = J @ F @ J_inverse
                P = F_MPC @ self.P @ F_MPC.T
            else:
                sigma, W_mean, W_cov = gauss2sigma(
                    GaussianState(self.y, self.P), alpha=0.1, beta=2, kappa=0,
                )
                predicted = []
                for point in np.asarray(sigma.state_vector).T:
                    x_point, _ = to_cartesian(point, self.origin, self.camera_velocity)
                    y_point, _ = to_modified_polar(F @ x_point, origin, camera_velocity)
                    predicted.append(y_point)
                predicted = np.asarray(predicted)

                # Unwrap bearings around the central prediction before averaging.
                beta_reference = y[2]
                predicted[:, 2] = beta_reference + (
                    predicted[:, 2] - beta_reference + np.pi
                ) % (2 * np.pi) - np.pi
                y = W_mean @ predicted
                differences = predicted - y
                differences[:, 2] = (differences[:, 2] + np.pi) % (2 * np.pi) - np.pi
                P = (differences.T * W_cov) @ differences

            # ponytail: linearized process noise; augment sigma points if noise nonlinearity matters.
            P += J @ Q @ J.T

            # Update: H = [0, 0, 1, 0], with wrapped bearing innovation.
            innovation = math.atan2(math.sin(bearing - y[2]), math.cos(bearing - y[2]))
            K = P[:, 2] / (P[2, 2] + R)
            y = y + K * innovation
            P = P - np.outer(K, P[2, :])

        # Convert MPC posterior and covariance to navigation-frame Cartesian output.
        y[2] = math.atan2(math.sin(y[2]), math.cos(y[2]))
        x, J_inverse = to_cartesian(y, origin, camera_velocity)
        P = (P + P.T) / 2
        P_x = J_inverse @ P @ J_inverse.T
        self.y, self.P = y, P
        self.timestamp = timestamp
        self.origin = origin.copy()
        self.camera_velocity = camera_velocity.copy()
        return x, P_x
