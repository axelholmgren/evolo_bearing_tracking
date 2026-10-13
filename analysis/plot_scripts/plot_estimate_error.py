#!/usr/bin/env python3
"""Plot target-state estimate error against simulator truth for one sim bag.

Run with ROS Python (not the uv env), after sourcing ROS and the workspace:
  source /opt/ros/humble/setup.bash && source ~/code/ros2_ws/install/setup.bash
  python3 analysis/plot_scripts/plot_estimate_error.py <bag_dir>

Needs /tf, /tf_static and the estimator's TargetStateStamped topic in the bag.
t = 0 is when evolo first moves --trigger m from its start, the same event
that starts the boats in WaypointNPCController.
Outputs results/estimate_error_<bag>.csv and results/plots/estimate_error_<bag>.png.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException

RESULTS_DIR = Path(__file__).resolve().parents[2] / "results"
PLOTS_DIR = RESULTS_DIR / "plots"


def read_bag(bag_dir, estimate_topic):
    """Fill an offline TF buffer and collect estimates as (t, msg)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if estimate_topic not in types:
        raise SystemExit(f"{estimate_topic} not in bag; topics: {sorted(types)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/tf_static", estimate_topic]))
    msg_types = {name: get_message(types[name]) for name in ("/tf", "/tf_static", estimate_topic) if name in types}

    tf = Buffer(cache_time=Duration(seconds=24 * 3600))  # default 10 s would drop old TF
    estimates, tf_stamps = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])
        if topic == estimate_topic:
            estimates.append(msg)
            continue
        for transform in msg.transforms:
            if topic == "/tf_static":
                tf.set_transform_static(transform, "bag")
            else:
                tf.set_transform(transform, "bag")
                tf_stamps.append(Time.from_msg(transform.header.stamp).nanoseconds)
    if not estimates:
        raise SystemExit(f"No messages on {estimate_topic}")
    return tf, estimates, min(tf_stamps), max(tf_stamps)


def xy(tf, world, frame, t_ns):
    """(x, y) of frame in world at t_ns, or None if TF can't answer."""
    try:
        p = tf.lookup_transform(world, frame, Time(nanoseconds=t_ns)).transform.translation
    except TransformException:
        return None
    return np.array([p.x, p.y])


def start_time(tf, world, evolo_frame, t_begin, t_end, trigger):
    """First time evolo is `trigger` m (horizontal) from its first known position."""
    first = None
    for t_ns in range(t_begin, t_end, 50_000_000):  # 20 Hz scan
        p = xy(tf, world, evolo_frame, t_ns)
        if p is None:
            continue
        if first is None:
            first = p
        elif np.linalg.norm(p - first) >= trigger:
            return t_ns
    raise SystemExit(f"{evolo_frame} never moved {trigger} m in {world}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--estimate-topic", default="/state_estimator_node/target_state")
    # Unverified default: ROSTransformTreePublisher prefixes non-robot frames with
    # the top-level Unity parent; check with tf2_echo on a recorded bag.
    parser.add_argument("--truth-frame", default="Obstacles_Evolo_HDRP/Boat1")
    parser.add_argument("--evolo-frame", default="evolo/base_link")
    parser.add_argument("--trigger", type=float, default=3.0, help="m, match evoloMovedTrigger")
    args = parser.parse_args()

    tf, estimates, t_begin, t_end = read_bag(args.bag, args.estimate_topic)
    world = estimates[0].header.frame_id  # evolo/odom from bearing_marker_node
    t0 = start_time(tf, world, args.evolo_frame, t_begin, t_end, args.trigger)

    rows = []
    for est in estimates:
        t_ns = Time.from_msg(est.header.stamp).nanoseconds
        truth = xy(tf, world, args.truth_frame, t_ns)
        evolo = xy(tf, world, args.evolo_frame, t_ns)
        before = xy(tf, world, args.truth_frame, t_ns - 500_000_000)
        after = xy(tf, world, args.truth_frame, t_ns + 500_000_000)
        if truth is None or evolo is None or before is None or after is None:
            continue
        pos = np.array([est.position.x, est.position.y])
        vel = np.array([est.velocity.x, est.velocity.y])
        cov = np.array(est.covariance).reshape(4, 4)
        rows.append({
            "t": (t_ns - t0) * 1e-9,
            "est_x": pos[0], "est_y": pos[1],
            "true_x": truth[0], "true_y": truth[1],
            "evolo_x": evolo[0], "evolo_y": evolo[1],
            "position_error": np.linalg.norm(pos - truth),
            "position_2sigma": 2 * math.sqrt(max(cov[0, 0] + cov[1, 1], 0.0)),
            "est_range": np.linalg.norm(pos - evolo),
            "true_range": np.linalg.norm(truth - evolo),
            "velocity_error": np.linalg.norm(vel - (after - before)),  # 1 s difference
        })
    if not rows:
        raise SystemExit(f"No estimate could be matched to TF for {args.truth_frame}")

    name = args.bag.resolve().name
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = RESULTS_DIR / f"estimate_error_{name}.csv"
    with csv_path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    col = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"Estimate vs truth: {name}\n{args.estimate_topic} vs {args.truth_frame} in {world}")

    ax[0, 0].plot(col["true_x"], col["true_y"], label="target truth")
    ax[0, 0].plot(col["evolo_x"], col["evolo_y"], label="evolo")
    ax[0, 0].plot(col["est_x"], col["est_y"], ".", markersize=2, label="estimate")
    ax[0, 0].set(xlabel="x east [m]", ylabel="y north [m]", aspect="equal")
    ax[0, 0].legend()

    ax[0, 1].plot(col["t"], col["position_error"], label="error")
    ax[0, 1].plot(col["t"], col["position_2sigma"], "--", label="2σ (filter)")
    ax[0, 1].set(xlabel="t [s]", ylabel="position error [m]")
    ax[0, 1].legend()

    ax[1, 0].plot(col["t"], col["true_range"], label="true")
    ax[1, 0].plot(col["t"], col["est_range"], label="estimate")
    ax[1, 0].set(xlabel="t [s]", ylabel="range evolo→target [m]")
    ax[1, 0].legend()

    ax[1, 1].plot(col["t"], col["velocity_error"])
    ax[1, 1].set(xlabel="t [s]", ylabel="velocity error [m/s]")

    for a in ax.flat[1:]:
        a.grid(True)
    fig.tight_layout()
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    png_path = PLOTS_DIR / f"estimate_error_{name}.png"
    fig.savefig(png_path, dpi=200)
    print(f"t0 = {t0 * 1e-9:.2f} s (bag time), {len(rows)}/{len(estimates)} estimates matched")
    print(f"Saved {csv_path}\nSaved {png_path}")


if __name__ == "__main__":
    main()
