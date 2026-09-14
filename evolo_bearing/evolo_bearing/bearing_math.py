"""Small geometry helpers shared by bearing nodes."""

import math


def rotate_bearing_xy(x, y, angle_deg):
    """Rotate an XY bearing counter-clockwise by ``angle_deg``."""
    angle_rad = math.radians(angle_deg)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    return (
        cos_angle * x - sin_angle * y,
        sin_angle * x + cos_angle * y,
    )
