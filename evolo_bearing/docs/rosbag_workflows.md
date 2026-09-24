# Rosbag replay

Copy `evolo_bearing_config/config/experiments/template.yaml`. Set `rosbag: true`,
`bag_path`, and `use_sim_time: true`. Enable `bearing_error` to save a CSV.

```bash
cd /home/axelholmgren/code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select evolo_gimbal_calibration evolo_bearing evolo_bearing_config evolo_reference_markers evolo_bearing_error
source install/setup.bash
ros2 launch evolo_bearing_config holy.launch.py config:=/path/to/run.yaml
```
