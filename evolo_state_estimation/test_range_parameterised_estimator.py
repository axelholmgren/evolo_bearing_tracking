"""Run offline: python -m unittest test_range_parameterised_estimator."""

from datetime import datetime, timedelta, timezone
import math
import unittest

import numpy as np

from range_parameterised_estimator import RangeParameterisedMPC


def run(estimator, target, steps=120):
    start = datetime(2026, 1, 1, tzinfo=timezone.utc)
    rng = np.random.default_rng(0)
    errors = []
    for k in range(steps):
        # Straight leg, then a turn so range becomes observable.
        origin = (np.array([3. * k, 0.]) if k < 40
                  else np.array([120., 3. * (k - 40)]))
        position = target + np.array([1., 0.5]) * k
        bearing = math.atan2(*(position - origin)[::-1]) + rng.normal(0, math.radians(0.5))
        x, P = estimator.step(start + timedelta(seconds=k), bearing, origin)
        assert np.isfinite(x).all() and np.linalg.eigvalsh(P).min() > -1e-6
        errors.append(np.linalg.norm(x[:2] - position))
    return errors


class RangeParameterisedTest(unittest.TestCase):
    def test_bank_converges_for_near_and_far_targets(self):
        for target_range in (60., 400., 1500.):
            target = np.array([0., target_range])
            bank = RangeParameterisedMPC(20., 2000., 8, 2., 0.5, 0.01)
            errors = run(bank, target)
            final_range = np.linalg.norm(target + np.array([1., .5]) * 119 - [120., 237.])
            self.assertLess(errors[-1] / final_range, 0.25, target_range)
            self.assertLess(len(bank.filters), 8)  # Wrong ranges were pruned.

    def test_bad_input_does_not_drop_hypotheses(self):
        bank = RangeParameterisedMPC(20., 2000., 5, 2., 1., 0.01)
        start = datetime(2026, 1, 1, tzinfo=timezone.utc)
        bank.step(start, 0., [0., 0.])
        with self.assertRaises(ValueError):
            bank.step(start, 0., [0., 0.])
        self.assertEqual(len(bank.filters), 5)


if __name__ == "__main__":
    unittest.main()
