import math
import unittest

import numpy as np

from evolo_bearing.bearing_math import pixel_to_link_ray

K = np.array([[2000.0, 0.0, 950.0], [0.0, 2000.0, 530.0], [0.0, 0.0, 1.0]])
CALIB = {"width": 1920, "height": 1080, "K": K, "D": np.zeros(5)}


class PixelToLinkRayTest(unittest.TestCase):
    def test_principal_point_is_boresight(self):
        ray = pixel_to_link_ray(950, 530, 1920, 1080, 57.1, CALIB)
        np.testing.assert_allclose(ray, (1.0, 0.0, 0.0), atol=1e-9)

    def test_one_focal_length_right_is_minus_45_deg(self):
        x, y, z = pixel_to_link_ray(950 + 2000, 530, 1920, 1080, 57.1, CALIB)
        self.assertAlmostEqual(math.degrees(math.atan2(y, x)), -45.0)
        self.assertAlmostEqual(z, 0.0)

    def test_below_center_points_down(self):
        self.assertLess(pixel_to_link_ray(950, 900, 1920, 1080, 57.1, CALIB)[2], 0.0)

    def test_linear_matches_yolo_action(self):
        u, v = 1500.0, 300.0
        angle_per_pixel = math.radians(57.1) / 1920
        yaw, pitch = -(u - 960) * angle_per_pixel, (v - 540) * angle_per_pixel
        expected = (
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            -math.sin(pitch),
        )
        np.testing.assert_allclose(pixel_to_link_ray(u, v, 1920, 1080, 57.1), expected)

    def test_size_mismatch_raises(self):
        with self.assertRaises(ValueError):
            pixel_to_link_ray(640, 360, 1280, 720, 57.1, CALIB)


if __name__ == "__main__":
    unittest.main()
