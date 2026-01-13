# PhoneFusion-Nav — Phase 2 (IMU-only Dead-Reckoning)

## Topics
- Subscribed: `/imu/data` (sensor_msgs/Imu)
- Published: `/imu/odom` (nav_msgs/Odometry), `/imu/path` (nav_msgs/Path)

## Packages
- `phone_imu_bridge` : UDP -> `/imu/data`
- `imu_deadreckoning`: integrates gyro+accel -> `/imu/odom`, `/imu/path`
- `phonefusion_nav_bringup`: launches both + RViz config

## Launch
(Execution later — code-first workflow.)
