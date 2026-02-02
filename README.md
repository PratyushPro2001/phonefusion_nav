# PhoneFusion-Nav

---

# Phase 2 & Phase 2.5 — IMU Dead Reckoning and GPS Anchoring

## Phase 2: IMU-Only Dead Reckoning

### Objective

Demonstrate IMU-only dead reckoning using a smartphone as a sensor platform, and explicitly observe and analyze drift behavior.

### Sensor Inputs

Data streamed from an Android phone (via SensaGram → UDP → ROS2):

- Linear acceleration (gravity removed)
- Gyroscope (angular velocity)
- Game rotation vector (orientation quaternion)

### Pipeline

Phone IMU ↓ UDP (SensaGram) phone\_imu\_bridge ↓ sensor\_msgs/Imu (/imu/data) imu\_deadreckoning ↓ nav\_msgs/Odometry (/imu/odom) RViz (path visualization)

### Key Processing Steps

- Axis alignment and unit normalization
- Bias calibration at startup
- Low-pass filtering of acceleration
- Velocity integration
- Position integration
- Zero-Velocity Update (ZUPT) during stationary detection
- Velocity damping to prevent runaway drift

### Result

- Short-term motion tracking behaves correctly
- Long-term position drifts inevitably, even when the phone is stationary

### Engineering Insight

IMU-only dead reckoning suffers from unbounded drift due to double integration of noisy acceleration. This behavior is expected and intentionally exposed in Phase 2.

---

## Phase 2.5: GPS-Anchored IMU Odometry

### Objective

Bound long-term drift by introducing GPS as a slow correction signal, without using an EKF or SLAM framework.

### Additional Sensor Input

- GPS (NavSatFix):
  - Latitude
  - Longitude
  - Altitude
  - Horizontal accuracy (covariance)

### Added Component

gps\_anchor\_fuser (custom ROS2 node)

### Pipeline

IMU Dead Reckoning (/imu/odom) + GPS Fix (/gps/fix) ↓ gps\_anchor\_fuser ↓ Fused Odometry (/fused/odom) Fused Path (/fused/path)

### Fusion Strategy

- Convert GPS latitude/longitude into a local Cartesian frame
- Use GPS as a low-gain anchor, not a primary motion source
- Apply gradual correction using a tunable gain parameter
- Reject GPS updates when:
  - Reported accuracy is poor
  - Sudden position jumps exceed a threshold
- Preserve IMU smoothness while bounding long-term drift

### Result

- IMU trajectory remains smooth and responsive
- Long-term drift is bounded by GPS anchoring
- Demonstrates why multi-sensor fusion is required

---

## Phase 3 — Visual–Inertial Fusion (VO + IMU, No EKF)

### Objective

Fuse visual odometry with smartphone IMU data to produce a stable, SLAM-ready odometry stream, **without using EKF or robot\_localization**, in order to explicitly control observability, drift, and constraints.

### Sensor Inputs

- Visual Odometry (`/vio/odom`)
  - Position from feature tracking + motion estimation
- IMU (`/imu/data`)
  - Orientation quaternion
  - Gravity-compensated linear acceleration
  - Angular velocity

### Core Idea

Each sensor is trusted for **different state variables**:

- **Position** → Visual Odometry
- **Orientation** → IMU quaternion
- **Velocity** → IMU linear acceleration (integrated in world frame)
- **Stationarity constraint** → ZUPT, gated by VO motion

This explicit role separation enables deterministic, debuggable fusion.

---

### Fusion Pipeline

VO Odom (/vio/odom) + IMU (/imu/data) ↓ vio\_fuser (custom ROS2 node) ↓ Fused Odometry (/vio\_fused/odom) Fused Path (/vio\_fused/path)

---

### Key Processing Steps

1. **Orientation Replacement**

   - IMU quaternion replaces VO orientation
   - Quaternion normalized at runtime

2. **World-Frame Acceleration**

   - IMU linear acceleration rotated from body → world frame
   - Prevents axis-coupled velocity drift

3. **Velocity Integration**

   - World-frame acceleration integrated once (accel → velocity)
   - No position integration from IMU (VO owns position)

4. **Zero-Velocity Update (ZUPT)**

   - Stationary detection using:
     - Acceleration magnitude
     - Gyroscope magnitude
   - Velocity damped or clamped to zero when stationary

5. **VO-Gated ZUPT (Critical)**

   - ZUPT is **disabled** whenever VO reports motion
   - Prevents false stationarity during smooth constant-velocity motion

---

### Active ZUPT Parameters (Validated)

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

### Result

- Velocity is **non-zero during motion**
- Velocity **decays smoothly** when stopping
- Velocity **clamps to zero** when fully stationary
- Output odometry is stable at \~30 Hz
- `/vio_fused/odom` is directly usable by SLAM or mapping stacks

---

### Engineering Insight

- IMU-only ZUPT fails during smooth motion due to zero acceleration
- Visual Odometry restores observability
- Fusion is enforced via **physical constraints**, not probabilistic black boxes
- EKF remains a future refactor, not a dependency

---

## Current Project Status

- Phase 2: IMU dead reckoning — Complete
- Phase 2.5: GPS anchoring — Complete
- **Phase 3: Visual–Inertial Fusion (manual)** — **Complete**
- Phase 4: EKF refactor / SLAM integration — Planned

