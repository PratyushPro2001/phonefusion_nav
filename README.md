# PhoneFusion-Nav

Phone-based **Visual–Inertial Odometry + GPS anchoring** on **ROS 2 Humble**.

**Goal:** produce a clean, SLAM-ready odometry stream:
- **Local, smooth motion:** VO + IMU → `/vio_fused/odom`
- **Globally bounded drift:** GPS motion-gated anchor → `/vio_gps/odom`

---

## Phase 2 — IMU-Only Dead Reckoning (Complete)

### Objective
Demonstrate IMU-only dead reckoning using a smartphone as a sensor platform, and explicitly observe drift.

### Inputs (SensaGram → UDP → ROS 2)
- `linear_acceleration` (gravity removed)
- `angular_velocity`
- `orientation` quaternion

### Pipeline
Phone IMU → `phone_imu_bridge` → `/imu/data` → `imu_deadreckoning` → `/imu/odom`

### Key steps
- Axis alignment + unit normalization
- Bias calibration
- Low-pass filtering
- Integrate accel → velocity → position
- ZUPT during stationarity

---

## Phase 2.5 — GPS-Anchored IMU Odometry (Complete)

### Objective
Bound IMU drift using GPS as a slow anchor signal (no EKF).

### Inputs
- IMU odom: `/imu/odom`
- GPS fix: `/gps/fix` (`sensor_msgs/NavSatFix`)

### Output
- Fused: `/fused/odom`, `/fused/path`

### Strategy
- Convert lat/lon to local XY (tangent plane approximation)
- Low-gain offset correction (anchor) + jump rejection + accuracy gating

---

## Phase 3 — Visual–Inertial Fusion (VO + IMU, Manual) (Complete)

### Objective
Create a **stable, SLAM-ready odometry** stream from:
- **Position:** Visual Odometry (`/vio/odom`)
- **Orientation:** IMU quaternion (`/imu/data.orientation`)
- **Velocity:** accel integration (world frame)
- **Stationarity constraint:** ZUPT, gated by VO motion

### Pipeline
DroidCam MJPEG → `ipcam_bridge` → `/camera/image_raw` → `feature_tracker` → `vo_estimator` → `/vio/odom`

`/vio/odom` + `/imu/data` → `vio_fuser` → `/vio_fused/odom`, `/vio_fused/path`

### Key processing
- Normalize IMU quaternion and replace VO orientation
- Rotate IMU `linear_acceleration` into world frame and integrate to velocity
- ZUPT: damp/clamp velocity when stationary
- **VO-gated ZUPT:** prevents false stationarity during smooth constant-velocity motion

### Validated ZUPT settings (example)
```yaml
zupt_enable: true
zupt_mode: damp
zupt_damp: 0.3
zupt_accel_thresh: 0.08      # m/s^2
zupt_gyro_thresh: 0.06       # rad/s
zupt_count: 6
zupt_use_vo_gate: true
zupt_vo_speed_thresh: 0.03   # m/s
```

---

## Phase 4A — GPS Anchor for VIO (Motion-Gated) (Complete)

### Objective
Anchor **VIO** globally **without letting GPS jitter walk the pose** when the phone is stationary.

### Inputs
- VIO fused odom: `/vio_fused/odom`
- GPS fix: `/gps/fix`

### Outputs
- **SLAM-ready anchored odom:** `/vio_gps/odom`
- `/vio_gps/path`

### Why motion-gated GPS matters
GPS has meters of noise; if you apply it while stationary, your pose will drift even when the phone doesn’t move.

**Fix:** only update the GPS correction offset when motion is confidently detected.

### Core parameters
```yaml
gps_alpha: 0.03              # anchor strength (0..1). smaller = gentler
max_hacc_m: 15.0             # reject GPS if covariance implies poor accuracy
max_gps_step_m: 12.0         # reject GPS teleports/jumps
min_speed_for_gps_update: 0.25  # motion gate: only apply GPS corrections when moving
```

**Behavior check:**
- Phone still → `/vio_gps/odom` position stays stable (no jitter-walk)
- Phone moving → GPS slowly pulls trajectory toward global consistency

---

## Quickstart

### Build
```bash
cd ~/phonefusion_nav/ros2_ws
colcon build --merge-install
source install/setup.bash
```

### Launch full stack
```bash
ros2 launch phonefusion_nav_bringup vio_full_stack.launch.py
```

### Key topics
```text
/camera/image_raw
/vio/odom
/imu/data
/gps/fix
/vio_fused/odom
/vio_gps/odom
```

### Sanity checks
```bash
ros2 topic hz /vio_fused/odom
ros2 topic hz /vio_gps/odom
ros2 topic echo /gps/fix --once
ros2 topic echo /vio_gps/odom --field pose.pose.position --once
```

---

## Current Project Status
- Phase 2: IMU dead reckoning — ✅ Complete
- Phase 2.5: GPS anchoring (IMU) — ✅ Complete
- Phase 3: VIO fusion (manual) — ✅ Complete
- Phase 4A: GPS anchor for VIO (motion-gated) — ✅ Complete
- Phase 4B: SLAM integration / loop closure (future) — Planned
- EKF refactor (optional future) — Planned

