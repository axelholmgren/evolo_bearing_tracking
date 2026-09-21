# LiDAR-box candidates for the hand audit

Derived locally from the offline replay of `rosbag2_2026_08_17-12_20_02`.

`timestamp_source=marker_header` means the tracker preserved a bag-time marker stamp. `delivery_clock` means the tracker provided an out-of-range wall-time stamp, so this is a replay delivery-time candidate and must not be treated as valid LiDAR synchronization evidence.
