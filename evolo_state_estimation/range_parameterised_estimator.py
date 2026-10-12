"""Range-parameterised bank of MPC filters (Peach 1995)."""

import math

import numpy as np

from modified_polar_estimator import ModifiedPolarEKF


class RangeParameterisedMPC:
    def __init__(self, min_range, max_range, n_filters, velocity_std,
                 bearing_std, acceleration, method="ekf", min_weight=1e-3):
        if not 0 < min_range < max_range or n_filters < 1:
            raise ValueError("Need 0 < min_range < max_range and n_filters >= 1")
        # One filter per geometric range slice; equal weights = log-uniform prior.
        edges = np.geomspace(min_range, max_range, n_filters + 1)
        self.filters = [
            ModifiedPolarEKF((a + b) / 2, (b - a) / math.sqrt(12),  # Uniform-slice std.
                             velocity_std, bearing_std, acceleration, method)
            for a, b in zip(edges, edges[1:])
        ]
        self.weights = np.full(n_filters, 1 / n_filters)
        self.min_weight = min_weight

    def step(self, timestamp, bearing, origin):
        """Return navigation-frame [x, y, vx, vy] and covariance."""
        filters, log_weights, means, covariances = [], [], [], []
        for estimator, weight in zip(self.filters, self.weights):
            try:
                x, P = estimator.step(timestamp, bearing, origin)
            except ValueError:
                continue  # Infeasible hypothesis; bad input fails every member.
            filters.append(estimator)
            log_weights.append(math.log(weight) + estimator.log_likelihood)
            means.append(x)
            covariances.append(P)
        if not filters:
            raise ValueError("All range hypotheses were dropped")

        # Normalise in log space, prune unlikely ranges, renormalise.
        weights = np.exp(np.array(log_weights) - max(log_weights))
        keep = weights >= self.min_weight * weights.sum()
        self.filters = [f for f, k in zip(filters, keep) if k]
        self.weights = weights[keep] / weights[keep].sum()

        # Moment-match the mixture into one Gaussian.
        means, covariances = np.array(means)[keep], np.array(covariances)[keep]
        mean = self.weights @ means
        spread = means - mean
        covariance = (np.tensordot(self.weights, covariances, 1)
                      + (spread.T * self.weights) @ spread)
        return mean, covariance
