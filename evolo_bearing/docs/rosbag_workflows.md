# Rosbag bearing workflows

Build and source the bearing packages and bringup package:

```bash
cd ~/code/ros2_ws
colcon build --packages-select \
  evolo_gimbal_calibration evolo_bearing evolo_reference_markers \
  evolo_bearing_error
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Choose the bag explicitly. The launch starts playback with `/clock`, waits two
seconds for subscribers, and stops the graph when playback finishes:

```bash
ros2 launch evolo_bearing bearing.launch.py \
  workflow:=replay bag:=/path/to/bag
```

For a paired uncorrected/corrected comparison against boxes already in the bag:

```bash
ros2 launch evolo_bearing bearing.launch.py \
  workflow:=compare bag:=/path/to/bag run_id:=sweep_a1
```

Use `workflow:=compare_derived_lidar` when boxes must be derived from recorded
point clouds. `replay_rate:=<positive-number>` and
`start_offset:=<non-negative-seconds>` control playback. The launch never
records a new bag, and it refuses to overwrite comparison CSV files. Set
`use_sim_time:=true` or `false` to override the workflow's automatic choice.

See the [yaw-correction experiment](yaw_correction_experiment.md)
for the paired evaluation.
