"""Small geometry helpers shared by bearing nodes."""

import math

import cv2
import numpy as np
import yaml


def rotate_bearing_xy(x, y, angle_deg):
    """Rotate an XY bearing counter-clockwise by ``angle_deg``."""
    angle_rad = math.radians(angle_deg)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    return (
        cos_angle * x - sin_angle * y,
        sin_angle * x + cos_angle * y,
    )


def load_camera_info(path):
    """Read a camera calubreation YAML"""
    with open(path, "r") as f:
        calibration = yaml.safe_load(f) or {}

    return {
        "width": int(calibration["image_width"]),
        "height": int(calibration["image_height"]),
        "K": np.array(calibration["camera_matrix"]["data"]).reshape(3, 3),
        "D": np.array(calibration["distortion_coefficients"]["data"]),
    }


def pixel_to_link_ray(u, v, width, height, aperture_deg, camera_info=None):
    """
    Ray in camera link, unit length
    (x forward, y left, z up) from pixel (u,v)

    camera_info= None uses the linear aperture model, otherwise the camera distortion is mitigated by K, D
    """
    if camera_info is None:
        angle_per_pixel = math.radians(aperture_deg) / width
        yaw = -(u - 0.5 * width) * angle_per_pixel
        pitch = (v - 0.5 * height) * angle_per_pixel

        return (
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch),
        )

    if (width, height) != (camera_info["width"], camera_info["height"]):
        raise ValueError(
            f"image {width}x{height} does not match calibration"
            f"{camera_info['width']}x{camera_info['height']}"
        )

    pixel = np.array([[[u, v]]], dtype=np.float64)
    x, y = cv2.undistortPoints(pixel,camera_info["K"], camera_info["D"])[0,0] # optical axis to link as z1_optical_frame

    norm = math.sqrt(1.0 + x*x +y*y)
    return 1.0/norm , -x/norm, -y/norm


