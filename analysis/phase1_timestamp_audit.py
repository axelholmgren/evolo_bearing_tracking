#!/usr/bin/env python3
"""Offline Phase 1 timestamp, latency, and timing-sensitivity audit.

This tool reads per-topic Parquet exports and writes derived results to a
separate directory. It never modifies a source dataset. ``record - header`` is
called observed message age because the physical event represented by the
camera header has not been established.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


NS_PER_S = 1_000_000_000
NS_PER_MS = 1_000_000
CAMERA_HORIZONTAL_FOV_DEG = 57.1
TIMING_OFFSETS_MS = (20.0, 50.0, 100.0, 250.0, 500.0)
RATE_QUANTILES = (0.50, 0.90, 0.95)


@dataclass(frozen=True)
class TopicSpec:
    signal: str
    filename: str
    expected_header: str
    content_kind: str = "generic"


TOPICS = (
    TopicSpec("Camera image", "evolo_sensors_gimbal_camera_camera_image_raw.parquet", "yes"),
    TopicSpec("YOLO detections", "yolo_detections.parquet", "yes", "detections"),
    TopicSpec("YOLO tracking", "yolo_tracking.parquet", "yes", "detections"),
    TopicSpec("Selected target", "yolo_target.parquet", "yes", "detections"),
    TopicSpec(
        "Tracked POI",
        "evolo_gimbal_camera_tracked_poi_image.parquet",
        "yes",
    ),
    TopicSpec(
        "Published bearing",
        "evolo_gimbal_camera_target_bearing_marker.parquet",
        "yes",
    ),
    TopicSpec(
        "GCU feedback",
        "evolo_gimbal_camera_gimbal_gcu_fb.parquet",
        "no",
    ),
    TopicSpec("Gimbal joint state", "evolo_joint_states.parquet", "yes"),
    TopicSpec("Evolo odometry", "evolo_smarc_odom.parquet", "yes"),
    TopicSpec("Dynamic TF", "tf.parquet", "nested", "transforms"),
    TopicSpec("Static TF", "tf_static.parquet", "nested", "transforms"),
    TopicSpec(
        "Tender WARAPS position",
        "smarcduino_waraps_sensor_position.parquet",
        "no",
        "json_data",
    ),
    TopicSpec("LiDAR truth", "bounding_boxes_corrected.parquet", "yes"),
)


def header_stamp_ns(frame: pd.DataFrame) -> pd.Series:
    """Compose flattened ROS sec/nanosec columns into nanoseconds."""

    return (
        frame["header_stamp_sec"].astype("int64") * NS_PER_S
        + frame["header_stamp_nanosec"].astype("int64")
    )


def has_flat_header(frame: pd.DataFrame) -> bool:
    return {"header_stamp_sec", "header_stamp_nanosec"}.issubset(frame.columns)


def observed_message_age_ms(frame: pd.DataFrame) -> pd.Series:
    """Return bag-record timestamp minus ROS header timestamp in milliseconds."""

    return (frame["t_ns"].astype("int64") - header_stamp_ns(frame)) / NS_PER_MS


def finite(values: Iterable[float]) -> pd.Series:
    return (
        pd.Series(values, dtype="float64")
        .replace([np.inf, -np.inf], np.nan)
        .dropna()
    )


def summarize_distribution(
    metric: str,
    values: Iterable[float],
    unit: str,
    bag: str,
) -> dict[str, object]:
    samples = finite(values)
    row: dict[str, object] = {
        "bag": bag,
        "metric": metric,
        "unit": unit,
        "samples": len(samples),
        "negative_count": int((samples < 0).sum()),
    }
    columns = ("mean", "median", "iqr", "mad", "p90", "p95", "p99", "maximum")
    if samples.empty:
        row.update({name: math.nan for name in columns})
        row["outlier_count"] = 0
        return row

    q1 = float(samples.quantile(0.25))
    q3 = float(samples.quantile(0.75))
    iqr = q3 - q1
    median = float(samples.median())
    row.update(
        {
            "mean": float(samples.mean()),
            "median": median,
            "iqr": iqr,
            "mad": float((samples - median).abs().median()),
            "p90": float(samples.quantile(0.90)),
            "p95": float(samples.quantile(0.95)),
            "p99": float(samples.quantile(0.99)),
            "maximum": float(samples.max()),
            "outlier_count": int(((samples < q1 - 1.5 * iqr) | (samples > q3 + 1.5 * iqr)).sum()),
        }
    )
    return row


def load_topic(path: Path) -> pd.DataFrame:
    return pd.read_parquet(path)


def parse_json_list(value: object) -> list[dict]:
    if not isinstance(value, str) or not value:
        return []
    parsed = json.loads(value)
    return parsed if isinstance(parsed, list) else []


def content_summary(frame: pd.DataFrame, kind: str) -> str:
    if frame.empty:
        return "empty"
    if kind == "detections" and "detections" in frame:
        counts = frame["detections"].map(lambda value: len(parse_json_list(value)))
        ids = frame["detections"].map(
            lambda value: sum(bool(det.get("id", "")) for det in parse_json_list(value))
        )
        return (
            f"{int((counts > 0).sum())}/{len(frame)} messages contain detections; "
            f"{int((ids > 0).sum())} contain a tracked ID"
        )
    if kind == "json_data" and "data" in frame:
        valid = 0
        for value in frame["data"]:
            try:
                parsed = json.loads(value)
                valid += int("latitude" in parsed and "longitude" in parsed)
            except (TypeError, json.JSONDecodeError):
                pass
        return f"{valid}/{len(frame)} messages contain latitude and longitude"
    if kind == "transforms" and "transforms" in frame:
        count = sum(len(parse_json_list(value)) for value in frame["transforms"])
        return f"{count} nested transforms"
    return "present"


def nested_tf_metadata(frame: pd.DataFrame) -> tuple[set[str], int, int]:
    frames: set[str] = set()
    stamps: list[int] = []
    for value in frame.get("transforms", []):
        for transform in parse_json_list(value):
            header = transform.get("header", {})
            stamp = header.get("stamp", {})
            frames.add(str(header.get("frame_id", "")))
            frames.add(str(transform.get("child_frame_id", "")))
            if "sec" in stamp and "nanosec" in stamp:
                stamps.append(int(stamp["sec"]) * NS_PER_S + int(stamp["nanosec"]))
    return {frame_id for frame_id in frames if frame_id}, len(stamps), len(stamps) - len(set(stamps))


def inventory_topic(bag: str, bag_dir: Path, spec: TopicSpec) -> dict[str, object]:
    path = bag_dir / spec.filename
    base: dict[str, object] = {
        "bag": bag,
        "signal": spec.signal,
        "filename": spec.filename,
        "present": path.exists(),
        "rows": 0,
        "duration_s": math.nan,
        "nominal_rate_hz": math.nan,
        "median_gap_ms": math.nan,
        "p95_gap_ms": math.nan,
        "large_gap_count": 0,
        "record_nonmonotonic_count": 0,
        "duplicate_record_count": 0,
        "header_available": False,
        "zero_header_count": 0,
        "header_nonmonotonic_count": 0,
        "duplicate_header_count": 0,
        "frame_ids": "",
        "content": "missing",
    }
    if not path.exists():
        return base

    frame = load_topic(path)
    base["rows"] = len(frame)
    base["content"] = content_summary(frame, spec.content_kind)
    if frame.empty or "t_ns" not in frame:
        return base

    record_ns = frame["t_ns"].astype("int64")
    record_diff = record_ns.diff().dropna()
    positive_gaps = record_diff[record_diff > 0] / NS_PER_MS
    duration_s = float((record_ns.max() - record_ns.min()) / NS_PER_S)
    median_gap = float(positive_gaps.median()) if not positive_gaps.empty else math.nan
    base.update(
        {
            "duration_s": duration_s,
            "nominal_rate_hz": (len(frame) - 1) / duration_s if duration_s > 0 else math.nan,
            "median_gap_ms": median_gap,
            "p95_gap_ms": float(positive_gaps.quantile(0.95)) if not positive_gaps.empty else math.nan,
            "large_gap_count": int((positive_gaps > 5.0 * median_gap).sum()) if median_gap > 0 else 0,
            "record_nonmonotonic_count": int((record_diff < 0).sum()),
            "duplicate_record_count": int(record_ns.duplicated().sum()),
        }
    )

    if has_flat_header(frame):
        stamps = header_stamp_ns(frame)
        diff = stamps.diff().dropna()
        base.update(
            {
                "header_available": True,
                "zero_header_count": int((stamps == 0).sum()),
                "header_nonmonotonic_count": int((diff < 0).sum()),
                "duplicate_header_count": int(stamps.duplicated().sum()),
                "frame_ids": ", ".join(sorted(str(item) for item in frame.get("header_frame_id", []).unique())),
            }
        )
    elif spec.content_kind == "transforms":
        frames, stamp_count, duplicate_count = nested_tf_metadata(frame)
        base.update(
            {
                "header_available": stamp_count > 0,
                "duplicate_header_count": duplicate_count,
                "frame_ids": ", ".join(sorted(frames)),
            }
        )
    return base


def first_record_per_header(frame: pd.DataFrame, record_name: str) -> pd.DataFrame:
    result = pd.DataFrame(
        {"header_ns": header_stamp_ns(frame), record_name: frame["t_ns"].astype("int64")}
    )
    return result.groupby("header_ns", as_index=False)[record_name].min()


def exact_header_stage(
    earlier: pd.DataFrame,
    later: pd.DataFrame,
    earlier_name: str,
    later_name: str,
) -> pd.DataFrame:
    """Pair pipeline stages by identical inherited image header."""

    left = first_record_per_header(earlier, "earlier_record_ns")
    right = first_record_per_header(later, "later_record_ns")
    paired = left.merge(right, on="header_ns", how="inner")
    paired["delay_ms"] = (
        paired["later_record_ns"] - paired["earlier_record_ns"]
    ) / NS_PER_MS
    paired["stage"] = f"{earlier_name} -> {later_name}"
    return paired


def angular_rate_deg_s(
    timestamps_ns: Iterable[int],
    angles_deg: Iterable[float],
    *,
    max_gap_s: float,
) -> pd.Series:
    times = np.asarray(timestamps_ns, dtype=np.float64) / NS_PER_S
    angles = np.unwrap(np.deg2rad(np.asarray(angles_deg, dtype=np.float64)))
    dt = np.diff(times)
    rates = np.rad2deg(np.diff(angles)) / dt
    valid = np.isfinite(rates) & (dt > 0.0) & (dt <= max_gap_s)
    return pd.Series(rates[valid], dtype="float64")


def target_image_angle_rows(frame: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for source_index, row in frame.iterrows():
        detections = parse_json_list(row["detections"])
        if len(detections) != 1:
            continue
        detection = detections[0]
        mask = detection.get("mask", {})
        width = float(mask.get("width", 0.0))
        center_x = float(
            detection.get("bbox", {}).get("center", {}).get("position", {}).get("x", math.nan)
        )
        track_id = str(detection.get("id", ""))
        if not track_id or width <= 0.0 or not math.isfinite(center_x):
            continue
        angle_deg = -(center_x - 0.5 * width) * CAMERA_HORIZONTAL_FOV_DEG / width
        rows.append(
            {
                "source_index": source_index,
                "header_ns": int(row["header_stamp_sec"]) * NS_PER_S
                + int(row["header_stamp_nanosec"]),
                "track_id": track_id,
                "image_angle_deg": angle_deg,
            }
        )
    return pd.DataFrame(rows)


def image_angle_rates(frame: pd.DataFrame) -> pd.Series:
    targets = target_image_angle_rows(frame)
    if targets.empty:
        return pd.Series(dtype="float64")
    rates = []
    for _, group in targets.groupby("track_id"):
        ordered = group.sort_values("header_ns")
        rates.append(
            angular_rate_deg_s(
                ordered["header_ns"], ordered["image_angle_deg"], max_gap_s=0.5
            )
        )
    return pd.concat(rates, ignore_index=True) if rates else pd.Series(dtype="float64")


def component_rates(bag_dir: Path) -> dict[str, pd.Series]:
    rates: dict[str, pd.Series] = {}
    gimbal_path = bag_dir / "evolo_gimbal_camera_gimbal_gcu_fb.parquet"
    if gimbal_path.exists():
        gimbal = pd.read_parquet(gimbal_path, columns=["t_ns", "relative_yaw"])
        rates["Gimbal yaw rate"] = angular_rate_deg_s(
            gimbal["t_ns"], gimbal["relative_yaw"], max_gap_s=0.2
        ).abs()

    odom_path = bag_dir / "evolo_smarc_odom.parquet"
    if odom_path.exists():
        odom = pd.read_parquet(odom_path, columns=["twist_twist_angular_z"])
        rates["Observer yaw rate"] = pd.Series(
            np.rad2deg(odom["twist_twist_angular_z"].astype(float)).abs(),
            dtype="float64",
        )

    target_path = bag_dir / "yolo_target.parquet"
    if target_path.exists():
        target = pd.read_parquet(
            target_path,
            columns=[
                "header_stamp_sec",
                "header_stamp_nanosec",
                "detections",
            ],
        )
        rates["Image-plane target rate"] = image_angle_rates(target).abs()
    return rates


def sensitivity_rows(
    bag: str, rates: dict[str, pd.Series]
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    rate_rows: list[dict[str, object]] = []
    sensitivity: list[dict[str, object]] = []
    for component, values in rates.items():
        rate_rows.append(summarize_distribution(component, values, "deg/s", bag))

    for quantile in RATE_QUANTILES:
        component_quantiles = {
            name: float(finite(values).quantile(quantile))
            for name, values in rates.items()
            if not finite(values).empty
        }
        conservative_rate = sum(component_quantiles.values())
        for offset_ms in TIMING_OFFSETS_MS:
            sensitivity.append(
                {
                    "bag": bag,
                    "rate_quantile": f"p{round(100 * quantile)}",
                    "gimbal_rate_deg_s": component_quantiles.get("Gimbal yaw rate", math.nan),
                    "observer_rate_deg_s": component_quantiles.get("Observer yaw rate", math.nan),
                    "image_rate_deg_s": component_quantiles.get("Image-plane target rate", math.nan),
                    "conservative_rate_deg_s": conservative_rate,
                    "timing_offset_ms": offset_ms,
                    "induced_bearing_error_deg": conservative_rate * offset_ms / 1000.0,
                }
            )
    return rate_rows, sensitivity


def manual_check_rows(bag: str, frame: pd.DataFrame, count: int = 10) -> pd.DataFrame:
    sample = frame.head(count).copy()
    result = pd.DataFrame(
        {
            "bag": bag,
            "row_index": sample.index,
            "record_ns_raw": sample["t_ns"].astype("int64"),
            "header_sec_raw": sample["header_stamp_sec"].astype("int64"),
            "header_nanosec_raw": sample["header_stamp_nanosec"].astype("int64"),
        }
    )
    result["header_ns_recomposed"] = (
        result["header_sec_raw"] * NS_PER_S + result["header_nanosec_raw"]
    )
    result["age_ns_integer"] = result["record_ns_raw"] - result["header_ns_recomposed"]
    result["age_ms"] = result["age_ns_integer"] / NS_PER_MS
    return result


def write_plots(
    bag: str,
    output_dir: Path,
    ages: dict[str, pd.DataFrame],
    stages: list[pd.DataFrame],
    rates: dict[str, pd.Series],
) -> None:
    fig, ax = plt.subplots(figsize=(11, 6))
    for label, frame in ages.items():
        elapsed = (header_stamp_ns(frame) - header_stamp_ns(frame).min()) / NS_PER_S
        ax.scatter(elapsed, observed_message_age_ms(frame), s=3, alpha=0.45, label=label)
    ax.set(title=f"Observed message age over time — {bag}", xlabel="Header time from first sample [s]", ylabel="Bag record − header [ms]")
    ax.legend(markerscale=3)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output_dir / "observed_message_age_timeseries.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5))
    for stage in stages:
        if stage.empty:
            continue
        elapsed = (stage["header_ns"] - stage["header_ns"].min()) / NS_PER_S
        ax.scatter(elapsed, stage["delay_ms"], s=4, alpha=0.55, label=stage["stage"].iloc[0])
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set(title=f"Header-matched stage delay — {bag}", xlabel="Inherited image-header time from first match [s]", ylabel="Later record − earlier record [ms]")
    ax.legend(markerscale=3)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output_dir / "stage_delay_timeseries.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    labels = list(rates)
    positions = np.arange(len(labels))
    medians = [finite(rates[label]).quantile(0.50) for label in labels]
    p95s = [finite(rates[label]).quantile(0.95) for label in labels]
    ax.bar(positions, medians, label="median")
    ax.scatter(positions, p95s, color="tab:red", label="p95")
    ax.set_xticks(positions, labels, rotation=15, ha="right")
    ax.set(ylabel="Absolute angular rate [deg/s]", title=f"Timing-sensitivity rate inputs — {bag}")
    ax.legend()
    ax.grid(axis="y", alpha=0.2)
    fig.tight_layout()
    fig.savefig(output_dir / "angular_rate_summary.png", dpi=160)
    plt.close(fig)


def classification(inventory: pd.DataFrame) -> tuple[str, str]:
    present = set(inventory.loc[inventory["present"], "signal"])
    required_internal = {"YOLO tracking", "Selected target", "GCU feedback", "Evolo odometry"}
    if not required_internal.issubset(present):
        missing = ", ".join(sorted(required_internal - present))
        return "unusable", f"Missing internal analysis signals: {missing}."
    reasons = []
    if "Camera image" not in present:
        reasons.append("raw camera images are absent")
    if "Published bearing" not in present:
        reasons.append("published bearing output is absent")
    if "LiDAR truth" not in present:
        reasons.append("LiDAR truth is absent")
    reasons.append("GCU feedback has no device/header timestamp")
    reasons.append("WARAPS position has no measurement header")
    return "exploratory only", "; ".join(reasons) + "."


def audit_bag(bag_dir: Path, output_dir: Path) -> dict[str, object]:
    bag = bag_dir.name
    output_dir.mkdir(parents=True, exist_ok=False)

    inventory = pd.DataFrame(inventory_topic(bag, bag_dir, spec) for spec in TOPICS)
    inventory.to_csv(output_dir / "topic_inventory.csv", index=False)

    ages: dict[str, pd.DataFrame] = {}
    age_rows: list[dict[str, object]] = []
    loaded: dict[str, pd.DataFrame] = {}
    for filename, label in (
        ("yolo_detections.parquet", "YOLO detections"),
        ("yolo_tracking.parquet", "YOLO tracking"),
        ("yolo_target.parquet", "Selected target"),
        ("evolo_gimbal_camera_tracked_poi_image.parquet", "Tracked POI"),
        ("evolo_joint_states.parquet", "Gimbal joint state"),
        ("evolo_smarc_odom.parquet", "Evolo odometry"),
    ):
        path = bag_dir / filename
        if not path.exists():
            continue
        frame = load_topic(path)
        loaded[filename] = frame
        if has_flat_header(frame):
            ages[label] = frame
            age_rows.append(
                summarize_distribution(
                    f"{label} observed message age",
                    observed_message_age_ms(frame),
                    "ms",
                    bag,
                )
            )

    stage_frames: list[pd.DataFrame] = []
    stage_rows: list[dict[str, object]] = []
    stage_pairs = (
        ("yolo_detections.parquet", "yolo_tracking.parquet", "YOLO detections", "YOLO tracking"),
        ("yolo_tracking.parquet", "yolo_target.parquet", "YOLO tracking", "Selected target"),
    )
    for earlier_file, later_file, earlier_label, later_label in stage_pairs:
        if earlier_file not in loaded or later_file not in loaded:
            continue
        stage = exact_header_stage(loaded[earlier_file], loaded[later_file], earlier_label, later_label)
        stage_frames.append(stage)
        stage_rows.append(
            summarize_distribution(
                f"{earlier_label} -> {later_label} header-matched stage delay",
                stage["delay_ms"],
                "ms",
                bag,
            )
        )
        earlier_unique = header_stamp_ns(loaded[earlier_file]).nunique()
        later_unique = header_stamp_ns(loaded[later_file]).nunique()
        stage_rows[-1]["matched_headers"] = len(stage)
        stage_rows[-1]["earlier_header_coverage"] = len(stage) / earlier_unique if earlier_unique else math.nan
        stage_rows[-1]["later_header_coverage"] = len(stage) / later_unique if later_unique else math.nan

    distributions = pd.DataFrame(age_rows + stage_rows)
    distributions.to_csv(output_dir / "delay_statistics.csv", index=False)

    rates = component_rates(bag_dir)
    rate_rows, sensitivity = sensitivity_rows(bag, rates)
    pd.DataFrame(rate_rows).to_csv(output_dir / "angular_rate_statistics.csv", index=False)
    pd.DataFrame(sensitivity).to_csv(output_dir / "timing_sensitivity.csv", index=False)

    if "yolo_tracking.parquet" in loaded:
        manual_check_rows(bag, loaded["yolo_tracking.parquet"]).to_csv(
            output_dir / "manual_timestamp_checks.csv", index=False
        )

    write_plots(bag, output_dir, ages, stage_frames, rates)
    decision, reason = classification(inventory)
    return {
        "bag": bag,
        "decision": decision,
        "reason": reason,
        "inventory": inventory,
        "distributions": distributions,
        "rates": pd.DataFrame(rate_rows),
        "sensitivity": pd.DataFrame(sensitivity),
    }


def fmt(value: object, digits: int = 2) -> str:
    return "n/a" if pd.isna(value) else f"{float(value):.{digits}f}"


def write_summary(results: list[dict[str, object]], output_dir: Path) -> None:
    decisions = pd.DataFrame(
        {"bag": result["bag"], "decision": result["decision"], "reason": result["reason"]}
        for result in results
    )
    decisions.to_csv(output_dir / "bag_decisions.csv", index=False)
    all_inventory = pd.concat([result["inventory"] for result in results], ignore_index=True)
    all_distributions = pd.concat([result["distributions"] for result in results], ignore_index=True)
    all_rates = pd.concat([result["rates"] for result in results], ignore_index=True)
    all_sensitivity = pd.concat([result["sensitivity"] for result in results], ignore_index=True)
    all_inventory.to_csv(output_dir / "all_bags_topic_inventory.csv", index=False)
    all_distributions.to_csv(output_dir / "all_bags_delay_statistics.csv", index=False)
    all_rates.to_csv(output_dir / "all_bags_angular_rate_statistics.csv", index=False)
    all_sensitivity.to_csv(output_dir / "all_bags_timing_sensitivity.csv", index=False)

    lines = [
        "# Phase 1 offline timestamp and latency audit",
        "",
        "> Offline Phase 1 analysis complete; Phase 1 gate remains open pending an "
        "instrumented capture establishing camera exposure time, GCU sample timing, "
        "and synchronized truth timing.",
        "",
        "## Per-bag decisions",
        "",
        "| Bag | Decision | Evidence |",
        "| --- | --- | --- |",
    ]
    for row in decisions.to_dict(orient="records"):
        lines.append(f"| `{row['bag']}` | **{row['decision']}** | {row['reason']} |")

    lines.extend(
        [
            "",
            "## Observed message age",
            "",
            "These values are bag-record time minus ROS header time. They are not "
            "capture-to-detection latency because the physical camera-stamp event is unknown.",
            "",
            "| Bag | Signal | n | Median (ms) | IQR | p95 | p99 | Max | Negative | Outliers |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    ages = all_distributions[all_distributions["metric"].str.contains("observed message age")]
    for row in ages.to_dict(orient="records"):
        signal = row["metric"].replace(" observed message age", "")
        lines.append(
            f"| `{row['bag']}` | {signal} | {int(row['samples'])} | {fmt(row['median'])} | "
            f"{fmt(row['iqr'])} | {fmt(row['p95'])} | {fmt(row['p99'])} | "
            f"{fmt(row['maximum'])} | {int(row['negative_count'])} | {int(row['outlier_count'])} |"
        )

    lines.extend(
        [
            "",
            "## Header-matched stages",
            "",
            "| Bag | Stage | Matches | Earlier coverage | Later coverage | Median (ms) | p95 | Negative |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    stages = all_distributions[all_distributions["metric"].str.contains("header-matched")]
    for row in stages.to_dict(orient="records"):
        lines.append(
            f"| `{row['bag']}` | {row['metric'].replace(' header-matched stage delay', '')} | "
            f"{int(row['matched_headers'])} | {fmt(100 * row['earlier_header_coverage'], 1)}% | "
            f"{fmt(100 * row['later_header_coverage'], 1)}% | {fmt(row['median'])} | "
            f"{fmt(row['p95'])} | {int(row['negative_count'])} |"
        )

    lines.extend(
        [
            "",
            "## Timing sensitivity",
            "",
            "The conservative rate is the sum of same-quantile absolute observer, gimbal, "
            "and selected-target image-plane rates. The streams are not claimed to be "
            "physically synchronized.",
            "",
            "| Bag | Rate quantile | Composite rate (deg/s) | Error at 20 ms | 50 ms | 100 ms | 250 ms | 500 ms |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for bag, bag_rows in all_sensitivity.groupby("bag"):
        for quantile, quantile_rows in bag_rows.groupby("rate_quantile", sort=False):
            by_offset = quantile_rows.set_index("timing_offset_ms")
            rate = quantile_rows["conservative_rate_deg_s"].iloc[0]
            errors = [fmt(by_offset.loc[offset, "induced_bearing_error_deg"], 3) for offset in TIMING_OFFSETS_MS]
            lines.append(f"| `{bag}` | {quantile} | {fmt(rate)} | " + " | ".join(errors) + " |")

    lines.extend(
        [
            "",
            "## Evidence gaps",
            "",
            "- Camera images were not exported, and the physical event represented by the gscam header is unverified.",
            "- GCU feedback has no ROS header or device timestamp; bag-record time includes unknown polling and transport delay.",
            "- Published bearing markers and `/bounding_boxes/corrected` LiDAR truth are absent.",
            "- Tender WARAPS position is a headerless GeoPoint payload, so remote measurement time and clock offset are unavailable.",
            "- Tracked POI and published bearing code replace the inherited observation timestamp with callback/publication time.",
            "- Existing evidence cannot separate fixed delay, jitter, decoder buffering, scheduling, or recorder delay.",
            "",
            "## Interpretation",
            "",
            "All three bags retain enough internally related vision, target, gimbal, pose, and "
            "WARAPS data for exploratory timing and tracking work. None supports final RQ1.1 "
            "residual claims as-is, and no stable physical-delay correction can be validated "
            "from these exports alone. The strict Phase 1 gate therefore remains open.",
        ]
    )
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    bag_dirs = [path.expanduser().resolve() for path in args.bag_dirs]
    output_dir = args.output_dir.expanduser().resolve()
    for bag_dir in bag_dirs:
        if not bag_dir.is_dir():
            parser.error(f"bag directory does not exist: {bag_dir}")
        if output_dir == bag_dir or bag_dir in output_dir.parents:
            parser.error("output directory must not be inside a source bag directory")
    if output_dir.exists() and any(output_dir.iterdir()):
        parser.error(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    results = [audit_bag(bag_dir, output_dir / bag_dir.name) for bag_dir in bag_dirs]
    write_summary(results, output_dir)
    print(f"Wrote Phase 1 audit for {len(results)} bags to {output_dir}")


if __name__ == "__main__":
    main()
