# Interfaces — Topics, Frames, Parameters

## Topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `phone_imu_bridge` | `imu_deadreckoning`, `vio_fuser` |
| `/gps/fix` | `sensor_msgs/NavSatFix` | `phone_imu_bridge` | `gps_anchor_fuser` |
| `/imu/odom`, `/imu/path` | `nav_msgs/Odometry`, `Path` | `imu_deadreckoning` | (Phase 2.5 anchor) |
| `/camera/image_raw` | `sensor_msgs/Image` | `ipcam_bridge` | `feature_tracker`, `vo_estimator` |
| `/camera/features` | `sensor_msgs/Image` | `feature_tracker` | RViz |
| `/vio/odom`, `/vio/path` | `Odometry`, `Path` | `vo_estimator` | `vio_fuser` |
| `/vio/debug_image` | `Image` | `vo_estimator` | RViz |
| `/vio_fused/odom`, `/vio_fused/path` | `Odometry`, `Path` | `vio_fuser` | `gps_anchor_fuser` |
| `/vio_gps/odom`, `/vio_gps/path` | `Odometry`, `Path` | `gps_anchor_fuser` | downstream SLAM (future) |
| `/tag_pose` | `geometry_msgs/PoseStamped` | `tag_pose_bridge` | (not yet consumed) |

Legacy: `/gps/pose`, `/fusion/odom` (legacy `fusion_node`), `/odom` (`ekf2d`).

## Frames
`map` → `base_link` (published by `vio_fuser` with `broadcast_tf: true`). Sensor frames:
`imu_link`, `gps_link`, `camera`. No static transforms between them are published yet (see roadmap).

## Phone input
- **IMU/GPS:** SensaGram → UDP, default `bind_ip 0.0.0.0`, `port 5555`. Fields: linear acceleration (gravity removed), angular velocity, rotation vector.
- **Camera:** DroidCam MJPEG, default `http://192.168.1.114:4747/video` (override with `video_url:=`).

## Key parameters

### phone_imu_bridge
`port` (5555), `imu_topic`, `gps_topic`, `accel_scale` (9.80665 if accel is in g), `gyro_scale` (π/180 if deg/s),
`swap_xy/xz/yz`, `flip_x/y/z` (axis alignment), `use_orientation_if_available`.

### ipcam_bridge
`video_url`, `frame_id`, `publish_hz` (30), `connect_timeout_s`, `reconnect_backoff_s`.

### vo_estimator
Intrinsics `fx, fy, cx, cy` (default 525/525/320/240 — **replace with calibrated values**),
`max_corners` 500, `min_tracked` 120, `min_inlier_ratio` 0.55, `min_median_flow_px` 1.25, `step_scale` 0.05.

### vio_fuser
`use_imu_orientation`, `use_world_accel`, `accel_lpf_alpha` 0.2, `max_dt` 0.2,
ZUPT: `zupt_enable`, `zupt_accel_thresh` 0.08 m/s², `zupt_gyro_thresh` 0.06 rad/s, `zupt_count` 6,
`zupt_mode` (`damp`|`clamp`), `zupt_damp` 0.3 (launch value; code default 0.6),
`zupt_use_vo_gate`, `zupt_vo_speed_thresh` 0.03 m/s.

### gps_anchor_fuser
`gps_alpha` 0.03, `max_hacc_m` 15, `max_gps_step_m` 12, `correct_xy_only` true,
`gate_gps_when_stationary` true, `min_speed_for_gps_update` 0.25 m/s (launch; code default 0.08),
`min_dt_between_gps_updates` 0.2 s.

> Note: several launch-file values override code defaults (marked above). Treat the launch file as the source of truth.
