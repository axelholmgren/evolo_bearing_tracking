"""Tests for bearing-error geometry."""

import pytest

from evolo_bearing_error.bearing_error import bearing_error_2d, bearing_error_3d


def test_bearing_error_2d_returns_signed_angle_and_miss_distance():
    angle_deg, miss_distance_m = bearing_error_2d(
        origin=(0.0, 0.0, 0.0),
        bearing=(1.0, 0.0, 0.0),
        truth=(0.0, 1.0, 0.0),
    )

    assert angle_deg == pytest.approx(90.0)
    assert miss_distance_m == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("bearing", "truth"),
    [
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ],
)
def test_bearing_error_2d_rejects_zero_length_inputs(bearing, truth):
    assert bearing_error_2d((0.0, 0.0, 0.0), bearing, truth) is None


def test_bearing_error_3d_is_preserved():
    angle_deg, miss_distance_m = bearing_error_3d(
        origin=(0.0, 0.0, 0.0),
        bearing=(1.0, 0.0, 0.0),
        truth=(1.0, 1.0, 0.0),
    )

    assert angle_deg == pytest.approx(45.0)
    assert miss_distance_m == pytest.approx(1.0)
