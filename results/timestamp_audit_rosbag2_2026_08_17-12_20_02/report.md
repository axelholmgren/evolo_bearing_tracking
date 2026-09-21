# Phase 1 offline timestamp and latency audit

> Offline Phase 1 analysis complete; Phase 1 gate remains open pending an instrumented capture establishing camera exposure time, GCU sample timing, and synchronized truth timing.

## Per-bag decisions

| Bag | Decision | Evidence |
| --- | --- | --- |
| `rosbag2_2026_08_17-12_20_02` | **exploratory only** | raw camera images are absent; published bearing output is absent; LiDAR truth is absent; GCU feedback has no device/header timestamp; WARAPS position has no measurement header. |

## Observed message age

These values are bag-record time minus ROS header time. They are not capture-to-detection latency because the physical camera-stamp event is unknown.

| Bag | Signal | n | Median (ms) | IQR | p95 | p99 | Max | Negative | Outliers |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `rosbag2_2026_08_17-12_20_02` | YOLO detections | 4496 | 457.67 | 51.36 | 527.39 | 579.82 | 750.26 | 0 | 169 |
| `rosbag2_2026_08_17-12_20_02` | YOLO tracking | 4489 | 514.94 | 79.03 | 610.61 | 693.91 | 932.42 | 0 | 92 |
| `rosbag2_2026_08_17-12_20_02` | Selected target | 4489 | 518.89 | 79.54 | 616.56 | 699.42 | 1030.40 | 0 | 99 |
| `rosbag2_2026_08_17-12_20_02` | Tracked POI | 3338 | 1.74 | 2.75 | 8.74 | 15.55 | 654.10 | 0 | 220 |
| `rosbag2_2026_08_17-12_20_02` | Gimbal joint state | 3655 | 3.39 | 5.55 | 14.24 | 22.08 | 628.63 | 0 | 169 |
| `rosbag2_2026_08_17-12_20_02` | Evolo odometry | 3653 | 21.97 | 17.33 | 44.19 | 50.78 | 581.57 | 0 | 14 |

## Header-matched stages

| Bag | Stage | Matches | Earlier coverage | Later coverage | Median (ms) | p95 | Negative |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `rosbag2_2026_08_17-12_20_02` | YOLO detections -> YOLO tracking | 3181 | 70.8% | 70.9% | 61.13 | 123.23 | 264 |
| `rosbag2_2026_08_17-12_20_02` | YOLO tracking -> Selected target | 4487 | 100.0% | 100.0% | 3.06 | 10.93 | 784 |

## Timing sensitivity

The conservative rate is the sum of same-quantile absolute observer, gimbal, and selected-target image-plane rates. The streams are not claimed to be physically synchronized.

| Bag | Rate quantile | Composite rate (deg/s) | Error at 20 ms | 50 ms | 100 ms | 250 ms | 500 ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `rosbag2_2026_08_17-12_20_02` | p50 | 3.86 | 0.077 | 0.193 | 0.386 | 0.965 | 1.930 |
| `rosbag2_2026_08_17-12_20_02` | p90 | 12.80 | 0.256 | 0.640 | 1.280 | 3.199 | 6.398 |
| `rosbag2_2026_08_17-12_20_02` | p95 | 18.27 | 0.365 | 0.914 | 1.827 | 4.568 | 9.136 |

## Evidence gaps

- Camera images were not exported, and the physical event represented by the gscam header is unverified.
- GCU feedback has no ROS header or device timestamp; bag-record time includes unknown polling and transport delay.
- Published bearing markers and `/bounding_boxes/corrected` LiDAR truth are absent.
- Tender WARAPS position is a headerless GeoPoint payload, so remote measurement time and clock offset are unavailable.
- Tracked POI and published bearing code replace the inherited observation timestamp with callback/publication time.
- Existing evidence cannot separate fixed delay, jitter, decoder buffering, scheduling, or recorder delay.

## Interpretation

All three bags retain enough internally related vision, target, gimbal, pose, and WARAPS data for exploratory timing and tracking work. None supports final RQ1.1 residual claims as-is, and no stable physical-delay correction can be validated from these exports alone. The strict Phase 1 gate therefore remains open.
