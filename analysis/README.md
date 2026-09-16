# Analysis

Offline plotting scripts live in `plot_scripts/`.  Run them from the repository
root so relative result names resolve under `results/`.

## Phase 1 offline timestamp audit

`phase1_timestamp_audit.py` inventories the required timing signals, calculates
observed message age and header-matched pipeline delays, estimates angular-rate
distributions, and evaluates timing-induced bearing-error sensitivity. It calls
`bag record time - ROS header time` observed message age—not capture latency—until
the physical camera-stamp event is established independently.

Run all three existing exports into a new results directory:

```bash
MPLBACKEND=Agg .venv/bin/python analysis/phase1_timestamp_audit.py \
  /home/axelholmgren/code/data/ros_bags/rosbag2_2026_08_17-12_20_02 \
  /home/axelholmgren/code/data/ros_bags/rosbag2_2026_08_17-15_01_11 \
  /home/axelholmgren/code/data/ros_bags/rosbag2_2026_08_17-15_02_44 \
  --output-dir results/phase1_offline_timestamp_audit_<run-date>
```

The command refuses non-empty destinations and destinations inside a source
bag. It creates per-bag inventories, statistics, timestamp-check rows and plots,
plus an aggregate report and CSV tables.
