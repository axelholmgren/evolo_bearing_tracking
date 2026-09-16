# Rosbag bearing workflows

Build and source the bearing packages and bringup package:

```bash
cd ~/code/ros2_ws
colcon build --packages-select \
  evolo_gimbal_calibration evolo_bearing evolo_bearing_config evolo_reference_markers \
  evolo_bearing_error
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Choose the bag explicitly. The replay launch starts the normal observation
stack, starts playback with `/clock`, waits two seconds for subscribers, and
stops the graph when playback finishes:

```bash
ros2 launch evolo_bearing_config replay.launch.py \
  bag:=/path/to/bag
```

For a paired uncorrected/corrected comparison, start the comparison and replay
as separate, independently controllable processes:

```bash
ros2 launch evolo_bearing_config compare.launch.py \
  run_id:=sweep_a1 truth_source:=lidar_box \
  truth_topic:=/bounding_boxes/corrected use_sim_time:=true
ros2 bag play /path/to/bag --clock
```

Use `compare_derived_lidar.launch.py` when boxes must be derived from recorded
point clouds. `replay_rate:=<positive-number>` and
`start_offset:=<non-negative-seconds>` control playback. The launch never
records a new bag, and it refuses to overwrite comparison CSV files. Set
`use_sim_time:=true` for replay comparisons.

Use `compare_fixed.launch.py` or `compare_smarcduino.launch.py` when the truth
marker comes from one of the reference marker publishers.

See the [yaw-correction experiment](yaw_correction_experiment.md)
for the paired evaluation.
