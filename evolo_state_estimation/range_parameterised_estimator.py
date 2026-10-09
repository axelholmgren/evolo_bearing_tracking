"""Range-parameterised bank of MPC filters (Peach 1995)."""

import math

import numpy as np

from modified_polar_estimator import ModifiedPolarEKF


class RangeParameterisedMPC:
    def __init__(self, min_range, max_range, n_filters, velocity_std,
                 bearing_std, acceleration, method="ekf", min_weight=1e-3):
        if not 0 < min_range < max_range or n_filters < 1:
            raise ValueError("Need 0 < min_range < max_range and n_filters >= 1")
        # Split [min_range, max_range] into geometric slices; one filter per slice.
        ratio = (max_range / min_range) ** (1 / n_filters)
        edges = min_range * ratio ** np.arange(n_filters + 1)
        self.ranges = (edges[:-1] + edges[1:]) / 2
        range_stds = np.diff(edges) / math.sqrt(12)  # Std of a uniform slice.
        self.filters = [
            ModifiedPolarEKF(r, s, velocity_std, bearing_std, acceleration, method)
            for r, s in zip(self.ranges, range_stds)
        ]
        # Equal weights per geometric slice: a prior uniform in log range.
        self.log_weights = np.full(n_filters, -math.log(n_filters))
        self.min_weight = min_weight
        self.timestamp = None

    @property
    def weights(self):
        return np.exp(self.log_weights)

    def step(self, timestamp, bearing, origin):
        """Return navigation-frame [x, y, vx, vy] and covariance."""
        # Validate here, so bad input is not mistaken for a diverged member.
        if not math.isfinite(bearing) or not np.isfinite(origin).all():
            raise ValueError("Bearing and camera position must be finite")
        if self.timestamp is not None and timestamp <= self.timestamp:
            raise ValueError("Observation time must increase")

        filters, log_weights, means, covariances = [], [], [], []
        for estimator, log_weight in zip(self.filters, self.log_weights):
            try:
                x, P = estimator.step(timestamp, bearing, origin)
            except ValueError:
                continue  # Hypothesis became infeasible, e.g. negative inverse range.
            filters.append(estimator)
            log_weights.append(log_weight + estimator.log_likelihood)
            means.append(x)
            covariances.append(P)
        if not filters:
            raise ValueError("All range hypotheses were dropped")
        self.timestamp = timestamp

        log_weights = np.array(log_weights)
        log_weights -= np.logaddexp.reduce(log_weights)  # Normalise.
        keep = np.exp(log_weights) >= self.min_weight
        self.filters = [f for f, k in zip(filters, keep) if k]
        self.log_weights = log_weights[keep] - np.logaddexp.reduce(log_weights[keep])
        means = np.array(means)[keep]
        covariances = np.array(covariances)[keep]

        # Moment-match the mixture into one Gaussian.
        weights = self.weights
        mean = weights @ means
        spread = means - mean
        covariance = (np.einsum("i,ijk->jk", weights, covariances)
                      + (spread.T * weights) @ spread)
        return mean, covariance
