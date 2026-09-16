# Evolo bearing tracking

ROS 2 packages and analysis tools for the master's thesis
*Uncertainty-Aware Bearings-Only Tracking of Maritime Targets from a
Hydrofoiling Unmanned Surface Vessel*.

The project produces world-frame bearing rays from camera detections and gimbal
orientation, visualises them in RViz, and evaluates their error against known
or LiDAR-derived target positions. It also includes the gimbal-yaw correction
used by the bearing pipeline.

## Contents

- `evolo_bearing/` — bearing-ray nodes and RViz configuration.
- `evolo_bearing/launch/` — package-local bearing launch file.
- `evolo_bearing_config/launch/` — higher-level Python launch files for replay and comparison runs.
- `evolo_gimbal_calibration/` — yaw-correction model and calibration data.
- `evolo_reference_markers/` — reference-position marker publishers.
- `evolo_bearing_error/` — bearing-error calculation and CSV logging.
- `analysis/` — offline plotting and analysis scripts.
- `results/` — generated CSVs and plots; ignored by Git.

## Build

Use ROS 2 Humble and build from the workspace root:

```bash
cd /home/axelholmgren/code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select evolo_gimbal_calibration evolo_bearing evolo_bearing_config evolo_reference_markers evolo_bearing_error
source install/setup.bash
```

## Replay

```bash
ros2 launch evolo_bearing_config replay.launch.py bag:=<bag-directory>
```

Use `compare_derived_lidar.launch.py` to launch the LiDAR processing and
tracking packages as part of a paired correction comparison. The smaller
`bearing.launch.py`, `observe.launch.py`, and `compare.launch.py` files can be
combined or run independently when more control is useful.

See [the replay workflow](evolo_bearing/docs/rosbag_workflows.md) for CSV
error logging and [the yaw-correction experiment](evolo_bearing/docs/yaw_correction_experiment.md)
for the paired raw-versus-corrected evaluation.
