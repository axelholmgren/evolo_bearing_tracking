# Yaw correction

Run the same bag twice: once with `corrected_camera_tf: false`, once with
`corrected_camera_tf: true`. Keep the truth source and other settings the same.
Set a different `bearing_error_node.output_file` for each run.

```bash
ros2 launch evolo_bearing_config holy.launch.py config:=/path/to/run.yaml
python3 analysis/plot_scripts/compare_yaw_correction.py results/raw.csv results/corrected.csv
```
