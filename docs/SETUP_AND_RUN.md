# Setup & Run Guide

## Prerequisites
- Ubuntu 22.04, ROS 2 Humble (`ros-humble-desktop`)
- `python3-opencv`, `python3-numpy`, `python3-scipy`, `python3-pandas`, `python3-matplotlib`
- Optional for AprilTags: `ros-humble-apriltag-ros`, `ros-humble-camera-ros`, `ros-humble-image-proc`
- Optional for legacy C++ nodes: `libeigen3-dev`, `libapriltag-dev`, `nlohmann-json3-dev`
- Phone apps: **SensaGram** (IMU/GPS over UDP) and **DroidCam** (MJPEG camera). Phone and PC on the same Wi-Fi.

## Build
```bash
cd ~/phonefusion_nav/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install
source install/setup.bash
```

## Phone configuration
1. SensaGram: target = PC IP, UDP port `5555`; enable linear acceleration, gyroscope, rotation vector, and location.
2. DroidCam: note the URL shown (e.g. `http://192.168.1.114:4747/video`).
3. Find the PC IP with `hostname -I`; allow UDP 5555 through any firewall.

## Run
```bash
# Full Phase 4A stack
ros2 launch phonefusion_nav_bringup vio_full_stack.launch.py video_url:=http://<phone-ip>:4747/video

# With RViz
ros2 launch phonefusion_nav_bringup vio_full_stack.launch.py use_rviz:=true rviz_config:=<path>.rviz

# Phase 2 only (IMU dead reckoning)
ros2 launch phonefusion_nav_bringup phase2_imu_bringup.launch.py
```

## Sanity checks
```bash
ros2 topic hz /imu/data /vio_fused/odom /vio_gps/odom
ros2 topic echo /gps/fix --once
ros2 topic echo /vio_gps/odom --field pose.pose.position --once
```
Expected behavior: phone still → `/vio_gps/odom` position stays put; phone moving → trajectory is slowly pulled toward GPS.

## Offline analysis of recorded phone data
```bash
pip install pandas numpy scipy matplotlib
python3 phonedata/south_brunswick_public_library/analyze_static.py phonedata/south_brunswick_public_library --trim 5
```
Outputs go to `analysis_out/` (5 PNGs + `static_summary.csv`).

## Troubleshooting
| Symptom | Likely cause / fix |
|---|---|
| No `/imu/data` | PC firewall, wrong IP/port in SensaGram, phone on different network |
| IMU axes look wrong in RViz | Use `swap_*` / `flip_*` params; check `accel_scale` / `gyro_scale` units |
| `ipcam_pub` can't connect | Wrong DroidCam URL; open it in a browser first |
| VO path doesn't move | Too few features (`min_tracked`), low texture, or motion below `min_median_flow_px` |
| Pose drifts while still | Check ZUPT thresholds; ensure `zupt_use_vo_gate` and `gate_gps_when_stationary` are true |
| `ros2 launch` can't find executable | Re-source `install/setup.bash`; legacy launch files use different executable names (see roadmap) |
