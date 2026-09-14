"""Unit tests for the stored gimbal-yaw correction contract."""

import numpy as np
import pytest

from evolo_gimbal_calibration.gimbal_yaw_correction import (
    C_ABSOLUTE,
    C_SHAPE,
    HYSTERESIS,
    PSI_NODES,
    correct_yaw,
)


def test_default_is_complete_absolute_correction_at_lut_node():
    psi = PSI_NODES[8]
    result = correct_yaw(psi, slew_dir=0)

    assert bool(result.valid)
    assert result.yaw_deg == pytest.approx(psi + C_SHAPE[8] + C_ABSOLUTE)


def test_shape_mode_excludes_only_the_absolute_offset():
    psi = PSI_NODES[12]
    absolute = correct_yaw(psi, mode="absolute", slew_dir=0)
    shape = correct_yaw(psi, mode="shape", slew_dir=0)

    assert absolute.yaw_deg - shape.yaw_deg == pytest.approx(C_ABSOLUTE)


def test_can_negate_complete_correction_for_comparison_experiments():
    psi = PSI_NODES[12]
    normal = correct_yaw(psi, mode="absolute")
    negated = correct_yaw(psi, mode="absolute", negate=True)

    assert negated.yaw_deg - psi == pytest.approx(-(normal.yaw_deg - psi))


def test_interpolates_shape_correction_between_lut_nodes():
    left, right = 6, 7
    psi = (PSI_NODES[left] + PSI_NODES[right]) / 2.0
    result = correct_yaw(psi, mode="shape", slew_dir=0)

    expected = psi + (C_SHAPE[left] + C_SHAPE[right]) / 2.0
    assert result.yaw_deg == pytest.approx(expected)


def test_slew_direction_applies_the_documented_backlash_sign():
    psi = PSI_NODES[10]
    clockwise = correct_yaw(psi, slew_dir=1)
    counterclockwise = correct_yaw(psi, slew_dir=-1)

    assert clockwise.yaw_deg - counterclockwise.yaw_deg == pytest.approx(-HYSTERESIS)


@pytest.mark.parametrize("psi", [-95.01, 82.01])
def test_out_of_domain_yaw_is_invalid_and_unchanged(psi):
    result = correct_yaw(psi)

    assert not bool(result.valid)
    assert result.yaw_deg == pytest.approx(psi)
    assert np.isnan(result.sigma_deg)


def test_invalid_mode_and_slew_direction_are_rejected():
    with pytest.raises(ValueError, match="mode"):
        correct_yaw(0.0, mode="other")
    with pytest.raises(ValueError, match="slew_dir"):
        correct_yaw(0.0, slew_dir=2)
    with pytest.raises(ValueError, match="negate"):
        correct_yaw(0.0, negate="true")
