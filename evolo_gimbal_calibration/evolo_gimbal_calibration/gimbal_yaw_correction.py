"""
Gimbal yaw correction for the Evolo passive bearing chain.

Maps the Z-1 Pro reported yaw to a corrected bearing, plus a per-measurement
1-sigma uncertainty suitable for populating R in a bearing-only filter.  The
default is the complete correction measured in the calibration experiment:
the repeatable shape, the measured absolute offset, and any supplied
per-boot/direction terms.

    theta_true = psi_readout + c_shape(psi_readout) + c_absolute + b + h(direction)

  c_shape  fixed, repeatable nonlinearity  (zero-mean, +/-5 deg peak)
  b        zero-point bias, RE-ESTIMATE AFTER EVERY BOOT (~1.5 deg spread)
  h        approach-direction / backlash term (+/-0.4 deg)

Derived from Exp 7, 9, 10: three independent CCW sweeps, 19 marks each,
reference = tape of known circumference (36.5 cm) wrapped on a jar,
9.863 deg per 1 cm mark.

CAVEATS -- read before using the absolute mode
  1. Validity domain is psi in [-95, +82] deg. Do not extrapolate.
  2. c_shape is well supported: the three sweeps agree to 0.68 deg RMS
     after removing one constant each, across a reboot and a pitch change.
  3. C_ABSOLUTE (+6.12 deg mean) is the absolute offset measured in the
     calibration experiment and is part of the default deployed correction.
     All sweeps used the same jar in the same position; a fixture-rotation
     test is still needed to establish that this offset transfers to a
     different fixture or world-bearing reference.
  4. Reference points are quantised to 1 deg; sub-degree structure is not
     resolved by this dataset.
"""

from dataclasses import dataclass

import numpy as np

# --- lookup table: node = mean readout over the three CCW sweeps ------------
PSI_NODES = np.array([
    -95.00, -86.33, -77.67, -69.00, -60.67, -49.00, -37.00, -23.33, -11.00,
     -1.33,   6.33,  15.00,  23.33,  32.33,  42.67,  54.67,  64.67,  73.00,
     82.00])

C_SHAPE = np.array([
     0.110,  1.306,  2.503,  3.699,  5.229,  3.425,  1.288, -2.516, -4.986,
    -4.789, -2.593, -1.397,  0.133,  0.996,  0.526, -1.611, -1.748, -0.219,
     0.644])

SIGMA_NODE = np.array([
     0.72,  0.99,  0.45,  0.72,  0.45,  0.72,  0.72,  0.28,  0.72,
     0.28,  0.45,  0.31,  0.55,  0.88,  0.81,  0.81,  0.81,  1.10,
     1.10])

C_ABSOLUTE = 6.12      # deg, measured absolute offset -- see caveat 3
PSI_MIN, PSI_MAX = -95.0, 82.0

HYSTERESIS = 0.79      # deg, CW readout minus CCW readout (Exp7, same session)
BOOT_BIAS_SIGMA = 0.9  # deg, 1-sigma of the boot-to-boot zero shift
QUANT_SIGMA = 1.0 / np.sqrt(12)   # 1 deg reading quantisation of the reference

# degree-8 fit of C_SHAPE against (psi / 90). RMS 0.28 deg.
# Provided for embedded use; prefer the LUT where you can.
POLY_COEF = np.array([-115.3330, -105.7740, 242.3175, 194.6320, -180.0030,
                      -104.2027,  55.0547,  12.2314,   -4.2403])


@dataclass
class YawCorrectionResult:
    """Yaw value and calibration-correction status."""

    yaw_deg: object
    sigma_deg: object
    valid: object


def correct_yaw(psi_deg, bias_deg=0.0, slew_dir=0, mode="absolute",
                clip=True, use_poly=False):
    """Correct a reported gimbal yaw into a bearing.

    psi_deg   reported yaw, scalar or array [deg]
    bias_deg  per-boot zero-point offset from your own boresight calibration.
              Pass 0.0 only if you re-zero the gimbal in software at startup.
    slew_dir  +1 if yaw is currently increasing (CW approach),
              -1 if decreasing (CCW), 0 to ignore backlash.
    mode      "absolute" -> complete measured correction (default), including
                             C_ABSOLUTE
              "shape"    -> zero-mean curve only, for diagnostics/comparison
    clip      retained for backwards-compatible calls; corrections are never
              applied outside the calibrated domain.
    Returns a YawCorrectionResult with yaw_deg, sigma_deg, and valid.
    """
    if (not np.isscalar(slew_dir) or isinstance(slew_dir, (bool, np.bool_))
            or slew_dir not in (-1, 0, 1)):
        raise ValueError("slew_dir must be -1 (CCW), 0 (unknown), or +1 (CW)")

    psi = np.asarray(psi_deg, dtype=float)
    valid = (PSI_MIN <= psi) & (psi <= PSI_MAX)
    if mode not in ("shape", "absolute"):
        raise ValueError("mode must be 'shape' or 'absolute'")

    correction = np.zeros_like(psi)
    psi_valid = psi[valid]
    if use_poly:
        correction[valid] = np.polyval(POLY_COEF, psi_valid / 90.0)
    else:
        correction[valid] = np.interp(psi_valid, PSI_NODES, C_SHAPE)

    if mode == "absolute":
        correction[valid] = correction[valid] + C_ABSOLUTE

    # Backlash: CW readings sit HYSTERESIS deg above CCW readings, so remove
    # half of it in each direction to land on the mid-value the LUT encodes.
    correction[valid] = (
        correction[valid] - np.sign(slew_dir) * HYSTERESIS / 2.0 + bias_deg
    )

    return YawCorrectionResult(
        yaw_deg=psi + correction,
        sigma_deg=yaw_sigma(psi),
        valid=valid,
    )


def yaw_sigma(psi_deg, bias_known=True, slew_dir_known=True):
    """1-sigma bearing uncertainty [deg] for the corrected yaw.

    Combine in quadrature with your image-plane and camera-to-INS terms to
    build R. Set bias_known=False if you have NOT re-zeroed since boot.
    Set slew_dir_known=False if the slew direction is unavailable.
    """
    psi = np.asarray(psi_deg, dtype=float)
    valid = (PSI_MIN <= psi) & (psi <= PSI_MAX)
    sigma = np.full_like(psi, np.nan)
    psi_valid = psi[valid]
    var = np.interp(psi_valid, PSI_NODES, SIGMA_NODE) ** 2
    var = var + QUANT_SIGMA ** 2
    if not bias_known:
        var = var + BOOT_BIAS_SIGMA ** 2
    if not slew_dir_known:
        var = var + (HYSTERESIS / 2.0) ** 2
    sigma[valid] = np.sqrt(var)
    return sigma


def yaw_variance_rad(psi_deg, **kw):
    """Same as yaw_sigma but returns variance in rad^2, ready to drop into R."""
    return np.deg2rad(yaw_sigma(psi_deg, **kw)) ** 2


if __name__ == "__main__":
    for p in [-90, -60, -45, -20, 0, 20, 45, 70, 82]:
        result = correct_yaw(p)
        print(f"psi {p:+4.0f} -> corrected {result.yaw_deg:+7.2f}  "
              f"sigma {result.sigma_deg:.2f}  "
              f"sigma(no re-zero) {yaw_sigma(p, bias_known=False):.2f}")
