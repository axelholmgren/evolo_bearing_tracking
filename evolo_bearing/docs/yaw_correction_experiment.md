# Yaw-correction experiment

This test answers one question: for the same observations, does the calibrated
gimbal-yaw correction reduce the angular error of the published bearing ray?
It compares the actual uncorrected and corrected `Marker` rays, not an error inferred
afterward from a yaw value.

## Before each run

Use a known target that stays visible and whose position is available in
`evolo/odom`. The default truth is LiDAR box `0` from
`/bounding_boxes/corrected`. Verify that this ID is the same physical target as
the visual detection before trusting the result. The default correction mode is
`absolute`; it includes the documented fixture-dependent +6.12° offset.

Build and source the packages, including the bringup package:

```bash
cd ~/code/ros2_ws
colcon build --packages-select evolo_gimbal_calibration evolo_bearing evolo_bearing_config evolo_bearing_error evolo_reference_markers pointcloud_preprocessing clustering_segmentation bb_dataass_tracking
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Rosbag replay experiment

The bringup launch starts exactly two comparison marker nodes in parallel (one
uncorrected, one corrected), an independent CSV logger for each, RViz, the
reference markers, and the LiDAR preprocessing/tracking chain. It also starts
the selected bag with `/clock` after the subscribers have initialized.

```bash
ros2 launch evolo_bearing_config compare_derived_lidar.launch.py \
  bag:=<bag-directory> run_id:=bag_2026_09_10_absolute
```

The launch stops after the replay. Then compare the two files named in its log
output:

```bash
cd ~/code/ros2_ws/src/evolo_bearing_tracking
python3 analysis/plot_scripts/compare_yaw_correction.py \
  results/bearing_error_bag_2026_09_10_absolute_primary_uncorrected_vs_lidar_box_0.csv \
  results/bearing_error_bag_2026_09_10_absolute_primary_corrected_vs_lidar_box_0.csv
```

Use the actual filenames printed by the logger. The terminal output reports MAE, RMSE, median absolute error, and
the paired per-sample improvement. A positive paired improvement and lower
corrected MAE/RMSE support the correction. The command writes the paired CSV
under `results/` and the plot under `results/plots/`.

**Mean Absolute Error** is the average absolute bearing error in degrees.
Lower Mean Absolute Error is better. The comparison excludes a matched pair
when either uncorrected or corrected bearing error exceeds 80°; pass
`--max-abs-error -1` to retain all matched pairs.

## Sign check

To test the opposite correction sign without changing the correction curve,
pass `negate_yaw_correction:=true` and use a distinct `run_id`. The uncorrected
bearing remains unchanged.

## Required experiment matrix

Run every row with the same bag once per row; do not toggle a parameter halfway
through a replay. Keep the camera target, truth source, and all transforms
unchanged within a paired run.

| Test | Yaw range / motion | Mode | What it checks |
| --- | --- | --- | --- |
| A1 | Slow sweep across -95° to +82° | `absolute` | Main default-mode test |
| A2 | Hold at -80, -45, 0, +45, +75° | `absolute` | Static errors and repeatability |
| A3 | Sweep CW, then CCW | `absolute` | Backlash / approach-direction sensitivity |
| A4 | Same as A1 after a power cycle and re-zero | `absolute` | Boot-to-boot bias robustness |
| S1 | Repeat A1 | `shape` | Shape-only comparison without fixture offset |
| O1 | Below -95° or above +82° | `absolute` | Guard check: correction should be inactive |

For each test, record target range, target ID, camera/gimbal boot state, sweep
direction, and any dropped tracking. Exclude samples with a stale or mismatched
truth source; do not use the result to judge correction if uncorrected and corrected
rows cannot be timestamp-paired.
