"""Tests for shared bearing geometry."""

import pytest

from evolo_bearing.bearing_math import rotate_bearing_xy


def test_zero_degree_rotation_leaves_bearing_unchanged():
    assert rotate_bearing_xy(2.0, -3.0, 0.0) == pytest.approx((2.0, -3.0))


def test_positive_rotation_is_counter_clockwise():
    assert rotate_bearing_xy(1.0, 0.0, 90.0) == pytest.approx((0.0, 1.0))


def test_negative_rotation_is_clockwise():
    assert rotate_bearing_xy(0.0, 1.0, -90.0) == pytest.approx((1.0, 0.0))
