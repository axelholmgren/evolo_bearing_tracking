# Phase 1 offline timestamp and latency audit

> Offline Phase 1 analysis complete; Phase 1 gate remains open pending an instrumented capture establishing camera exposure time, GCU sample timing, and synchronized truth timing.

## Per-bag decisions

| Bag | Decision | Evidence |
| --- | --- | --- |
| `rosbag2_2026_08_17-12_20_02` | **exploratory only** | raw camera images are absent; published bearing output is absent; LiDAR truth is absent; GCU feedback has no device/header timestamp; WARAPS position has no measurement header. |
| `rosbag2_2026_08_17-15_01_11` | **exploratory only** | raw camera images are absent; published bearing output is absent; LiDAR truth is absent; GCU feedback has no device/header timestamp; WARAPS position has no measurement header. |
| `rosbag2_2026_08_17-15_02_44` | **exploratory only** | raw camera images are absent; published bearing output is absent; LiDAR truth is absent; GCU feedback has no device/header timestamp; WARAPS position has no measurement header. |

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
| `rosbag2_2026_08_17-15_01_11` | YOLO detections | 722 | 462.27 | 54.44 | 529.30 | 566.28 | 1357.18 | 0 | 33 |
| `rosbag2_2026_08_17-15_01_11` | YOLO tracking | 723 | 469.34 | 64.55 | 581.21 | 693.33 | 1004.97 | 0 | 43 |
| `rosbag2_2026_08_17-15_01_11` | Selected target | 724 | 472.86 | 63.58 | 586.45 | 706.41 | 1034.89 | 0 | 46 |
| `rosbag2_2026_08_17-15_01_11` | Tracked POI | 90 | 1.50 | 2.83 | 6.74 | 7.77 | 12.90 | 0 | 1 |
| `rosbag2_2026_08_17-15_01_11` | Gimbal joint state | 885 | 1.31 | 4.31 | 12.25 | 21.96 | 702.07 | 0 | 51 |
| `rosbag2_2026_08_17-15_01_11` | Evolo odometry | 886 | 18.32 | 14.51 | 37.67 | 55.31 | 759.90 | 0 | 22 |
| `rosbag2_2026_08_17-15_02_44` | YOLO detections | 3153 | 481.47 | 53.64 | 561.09 | 608.44 | 787.24 | 0 | 104 |
| `rosbag2_2026_08_17-15_02_44` | YOLO tracking | 3159 | 529.38 | 107.49 | 651.08 | 716.61 | 1044.45 | 0 | 28 |
| `rosbag2_2026_08_17-15_02_44` | Selected target | 3160 | 534.53 | 108.88 | 658.49 | 724.70 | 1180.74 | 0 | 29 |
| `rosbag2_2026_08_17-15_02_44` | Tracked POI | 1047 | 1.87 | 2.89 | 7.54 | 12.51 | 33.43 | 0 | 42 |
| `rosbag2_2026_08_17-15_02_44` | Gimbal joint state | 4118 | 1.87 | 4.45 | 11.58 | 19.95 | 725.92 | 0 | 198 |
| `rosbag2_2026_08_17-15_02_44` | Evolo odometry | 4120 | 20.81 | 13.10 | 41.45 | 50.79 | 774.87 | 0 | 69 |

## Header-matched stages

| Bag | Stage | Matches | Earlier coverage | Later coverage | Median (ms) | p95 | Negative |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `rosbag2_2026_08_17-12_20_02` | YOLO detections -> YOLO tracking | 3181 | 70.8% | 70.9% | 61.13 | 123.23 | 264 |
| `rosbag2_2026_08_17-12_20_02` | YOLO tracking -> Selected target | 4487 | 100.0% | 100.0% | 3.06 | 10.93 | 784 |
| `rosbag2_2026_08_17-15_01_11` | YOLO detections -> YOLO tracking | 633 | 87.7% | 87.6% | 6.03 | 98.82 | 60 |
| `rosbag2_2026_08_17-15_01_11` | YOLO tracking -> Selected target | 723 | 100.0% | 99.9% | 2.46 | 10.28 | 118 |
| `rosbag2_2026_08_17-15_02_44` | YOLO detections -> YOLO tracking | 2767 | 87.8% | 87.6% | 14.78 | 121.04 | 78 |
| `rosbag2_2026_08_17-15_02_44` | YOLO tracking -> Selected target | 3159 | 100.0% | 100.0% | 3.79 | 14.68 | 415 |

## Timing sensitivity

The conservative rate is the sum of same-quantile absolute observer, gimbal, and selected-target image-plane rates. The streams are not claimed to be physically synchronized.

| Bag | Rate quantile | Composite rate (deg/s) | Error at 20 ms | 50 ms | 100 ms | 250 ms | 500 ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `rosbag2_2026_08_17-12_20_02` | p50 | 3.86 | 0.077 | 0.193 | 0.386 | 0.965 | 1.930 |
| `rosbag2_2026_08_17-12_20_02` | p90 | 12.80 | 0.256 | 0.640 | 1.280 | 3.199 | 6.398 |
| `rosbag2_2026_08_17-12_20_02` | p95 | 18.27 | 0.365 | 0.914 | 1.827 | 4.568 | 9.136 |
| `rosbag2_2026_08_17-15_01_11` | p50 | 5.64 | 0.113 | 0.282 | 0.564 | 1.411 | 2.822 |
| `rosbag2_2026_08_17-15_01_11` | p90 | 14.82 | 0.296 | 0.741 | 1.482 | 3.704 | 7.409 |
| `rosbag2_2026_08_17-15_01_11` | p95 | 19.91 | 0.398 | 0.996 | 1.991 | 4.978 | 9.957 |
| `rosbag2_2026_08_17-15_02_44` | p50 | 3.27 | 0.065 | 0.163 | 0.327 | 0.817 | 1.635 |
| `rosbag2_2026_08_17-15_02_44` | p90 | 12.61 | 0.252 | 0.630 | 1.261 | 3.152 | 6.304 |
| `rosbag2_2026_08_17-15_02_44` | p95 | 19.60 | 0.392 | 0.980 | 1.960 | 4.900 | 9.799 |

## Evidence gaps

- Camera images were not exported, and the physical event represented by the gscam header is unverified.
- GCU feedback has no ROS header or device timestamp; bag-record time includes unknown polling and transport delay.
- Published bearing markers and `/bounding_boxes/corrected` LiDAR truth are absent.
- Tender WARAPS position is a headerless GeoPoint payload, so remote measurement time and clock offset are unavailable.
- Tracked POI and published bearing code replace the inherited observation timestamp with callback/publication time.
- Existing evidence cannot separate fixed delay, jitter, decoder buffering, scheduling, or recorder delay.

## Interpretation

All three bags retain enough internally related vision, target, gimbal, pose, and WARAPS data for exploratory timing and tracking work. None supports final RQ1.1 residual claims as-is, and no stable physical-delay correction can be validated from these exports alone. The strict Phase 1 gate therefore remains open.
