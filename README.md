# Evolo bearing tracking

ROS 2 packages and analysis tools for the master's thesis
*Uncertainty-Aware Bearings-Only Tracking of Maritime Targets from a
Hydrofoiling Unmanned Surface Vessel*.

The project produces world-frame bearing rays from camera detections and gimbal
orientation, visualises them in RViz, and evaluates their error against known
or LiDAR-derived target positions. It also includes the gimbal-yaw correction
used by the bearing pipeline.

## Contents

- `evolo_bearing/` — bearing-ray nodes, launch files, and RViz configuration.
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
colcon build --packages-select evolo_gimbal_calibration evolo_bearing evolo_reference_markers evolo_bearing_error
source install/setup.bash
```

## Replay

```bash
ros2 launch evolo_bearing markers.launch.py use_sim_time:=true
```

In another terminal:

```bash
ros2 bag play <bag-directory> --clock
```

Add `lidar_boxes:=true` to include the LiDAR bounding-box pipeline. The
`scripts/launch_markers.sh` wrapper is also available for existing workflows.

See [the replay workflow](evolo_bearing/launch/rosbag_workflows.md) for CSV
error logging and [the yaw-correction experiment](evolo_bearing/launch/yaw_correction_experiment.md)
for the paired raw-versus-corrected evaluation.
