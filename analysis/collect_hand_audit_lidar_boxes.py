#!/usr/bin/env python3
"""Collect LiDAR box candidates near manual hand-audit timestamps.

Run this node before an offline replay that publishes
``/bounding_boxes/corrected``.  It writes only rows close to the timestamps in
the manual measurement table; it never records or modifies a rosbag.
"""

import argparse
import csv
import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray

MATCH_WINDOW_S = 0.080
ROW = re.compile(
    r"^\|\s*-?\d+(?:\.\d+)?\s*\|\s*-?\d+(?:\.\d+)?\s*\|\s*"
    r"-?\d+(?:\.\d+)?\s*\|\s*(?P<time>-?\d+(?:\.\d+)?)\s*\|\s*$"
)


def manual_times(path: Path) -> list[float]:
    times = []
    for line in path.read_text().splitlines():
        match = ROW.match(line)
        if match:
            times.append(float(match.group("time")))
    if not times:
        raise ValueError(f"no numeric measurement rows found in {path}")
    return times


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec / 1e9


class BoxSampler(Node):
    def __init__(self, sample_times: list[float], output_dir: Path):
        super().__init__("hand_audit_lidar_box_sampler")
        self.sample_times = sample_times
        self.output_dir = output_dir
        self.current_clock_s = None
        self.rows = []
        self.finished = False
        self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.create_subscription(
            MarkerArray, "/bounding_boxes/corrected", self.box_callback, 10
        )

    def clock_callback(self, msg: Clock) -> None:
        self.current_clock_s = stamp_seconds(msg.clock)
        if (
            not self.finished
            and self.current_clock_s > self.sample_times[-1] + MATCH_WINDOW_S
        ):
            self.write_results()
            self.finished = True
            self.get_logger().info(f"Wrote {self.output_dir}")
            rclpy.shutdown()

    def box_callback(self, msg: MarkerArray) -> None:
        if self.current_clock_s is None or self.finished:
            return
        for marker in msg.markers:
            if marker.type != Marker.CUBE or marker.action == Marker.DELETE:
                continue
            marker_time_s = stamp_seconds(marker.header.stamp)
            # Tracking nodes sometimes stamp with wall time despite replay.  A
            # stamp outside this bag's time range is unusable; retain it for
            # review but pair by the simulated delivery time instead.
            use_marker_stamp = (
                self.sample_times[0] - 1.0
                <= marker_time_s
                <= self.sample_times[-1] + 1.0
            )
            pair_time_s = marker_time_s if use_marker_stamp else self.current_clock_s
            for sample_time_s in self.sample_times:
                delta_s = abs(pair_time_s - sample_time_s)
                if delta_s <= MATCH_WINDOW_S:
                    self.rows.append(
                        {
                            "manual_time_ros_s": f"{sample_time_s:.3f}",
                            "box_pair_time_s": f"{pair_time_s:.9f}",
                            "box_marker_stamp_s": f"{marker_time_s:.9f}",
                            "box_delivery_clock_s": f"{self.current_clock_s:.9f}",
                            "timestamp_source": (
                                "marker_header" if use_marker_stamp else "delivery_clock"
                            ),
                            "timestamp_difference_ms": f"{delta_s * 1000.0:.3f}",
                            "lidar_box_namespace": marker.ns,
                            "lidar_box_id": marker.id,
                            "lidar_target_x_m": f"{marker.pose.position.x:.6f}",
                            "lidar_target_y_m": f"{marker.pose.position.y:.6f}",
                            "lidar_target_z_m": f"{marker.pose.position.z:.6f}",
                        }
                    )

    def write_results(self) -> None:
        self.output_dir.mkdir(parents=True)
        candidate_path = self.output_dir / "lidar_box_candidates.csv"
        fields = [
            "manual_time_ros_s",
            "box_pair_time_s",
            "box_marker_stamp_s",
            "box_delivery_clock_s",
            "timestamp_source",
            "timestamp_difference_ms",
            "lidar_box_namespace",
            "lidar_box_id",
            "lidar_target_x_m",
            "lidar_target_y_m",
            "lidar_target_z_m",
        ]
        with candidate_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        with (self.output_dir / "README.md").open("w") as handle:
            handle.write(
                "# LiDAR-box candidates for the hand audit\n\n"
                "Derived locally from the offline replay of "
                "`rosbag2_2026_08_17-12_20_02`.\n\n"
                "`timestamp_source=marker_header` means the tracker preserved "
                "a bag-time marker stamp. `delivery_clock` means the tracker "
                "provided an out-of-range wall-time stamp, so this is a replay "
                "delivery-time candidate and must not be treated as valid LiDAR "
                "synchronization evidence.\n"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manual_measurements", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error(f"refusing to overwrite existing output directory: {args.output_dir}")

    rclpy.init()
    node = BoxSampler(manual_times(args.manual_measurements), args.output_dir)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
