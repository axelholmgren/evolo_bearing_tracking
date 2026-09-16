import unittest

import numpy as np
import pandas as pd

from analysis.phase1_timestamp_audit import (
    angular_rate_deg_s,
    exact_header_stage,
    header_stamp_ns,
    observed_message_age_ms,
    summarize_distribution,
    target_image_angle_rows,
)


def stamped_frame(record_ms, header_ms):
    header_ns = np.asarray(header_ms, dtype=np.int64) * 1_000_000
    return pd.DataFrame(
        {
            "t_ns": np.asarray(record_ms, dtype=np.int64) * 1_000_000,
            "header_stamp_sec": header_ns // 1_000_000_000,
            "header_stamp_nanosec": header_ns % 1_000_000_000,
        }
    )


class Phase1TimestampAuditTest(unittest.TestCase):
    def test_header_stamp_and_observed_age_use_integer_nanoseconds(self):
        frame = stamped_frame([1500, 2200], [1000, 2000])

        self.assertEqual(header_stamp_ns(frame).tolist(), [1_000_000_000, 2_000_000_000])
        self.assertEqual(observed_message_age_ms(frame).tolist(), [500.0, 200.0])

    def test_exact_header_stage_keeps_negative_delays_and_unmatched_headers(self):
        earlier = stamped_frame([1400, 2400, 3400], [1000, 2000, 3000])
        later = stamped_frame([1475, 2350, 9999], [1000, 2000, 9000])

        paired = exact_header_stage(earlier, later, "a", "b")

        self.assertEqual(paired["header_ns"].tolist(), [1_000_000_000, 2_000_000_000])
        self.assertEqual(paired["delay_ms"].tolist(), [75.0, -50.0])

    def test_distribution_retains_negative_values_and_flags_tukey_outlier(self):
        result = summarize_distribution("age", [-1, 0, 1, 2, 100], "ms", "bag")

        self.assertEqual(result["samples"], 5)
        self.assertEqual(result["negative_count"], 1)
        self.assertEqual(result["outlier_count"], 1)
        self.assertEqual(result["maximum"], 100.0)

    def test_angular_rate_unwraps_boundary_and_rejects_long_gap(self):
        timestamps = [0, 100_000_000, 200_000_000, 1_000_000_000]
        angles = [179.0, -179.0, -177.0, 30.0]

        rates = angular_rate_deg_s(timestamps, angles, max_gap_s=0.2)

        self.assertTrue(np.allclose(rates, [20.0, 20.0]))

    def test_target_angle_uses_pipeline_linear_fov_model(self):
        detection = {
            "id": "7",
            "bbox": {"center": {"position": {"x": 1500.0}}},
            "mask": {"width": 2000, "height": 1000},
        }
        frame = pd.DataFrame(
            {
                "header_stamp_sec": [1],
                "header_stamp_nanosec": [0],
                "detections": [[detection]],
            }
        )
        frame["detections"] = frame["detections"].map(__import__("json").dumps)

        rows = target_image_angle_rows(frame)

        self.assertAlmostEqual(rows.iloc[0]["image_angle_deg"], -14.275)
        self.assertEqual(rows.iloc[0]["track_id"], "7")


if __name__ == "__main__":
    unittest.main()
