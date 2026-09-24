# Evolo bearing tracking

ROS 2 packages for bearing tracking, yaw correction, and bearing-error analysis.

## Contents

- `evolo_bearing/` — bearing rays
- `evolo_gimbal_calibration/` — yaw correction
- `evolo_reference_markers/` — reference markers
- `evolo_bearing_error/` — error calculation and CSV logging
- `evolo_bearing_config/` — launch and experiment configs
- `analysis/` — offline analysis

## Build

Use ROS 2 Humble and build from the workspace root:

```bash
cd /home/axelholmgren/code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select evolo_gimbal_calibration evolo_bearing evolo_bearing_config evolo_reference_markers evolo_bearing_error
source install/setup.bash
```

## Run

```bash
ros2 launch evolo_bearing_config holy.launch.py \
  config:=/absolute/path/to/experiment.yaml
```

Copy `evolo_bearing_config/config/experiments/template.yaml`. Set `rosbag`,
`bag_path`, and `use_sim_time` for replay. Enable `bearing_error` to log CSV.

See [the rosbag workflow](evolo_bearing/docs/rosbag_workflows.md) and
[the yaw-correction experiment](evolo_bearing/docs/yaw_correction_experiment.md).
