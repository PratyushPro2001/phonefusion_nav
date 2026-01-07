
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
Phone IMU
  ↓ UDP (SensaGram)
phone_imu_bridge
  ↓ sensor_msgs/Imu (/imu/data)
imu_deadreckoning
  ↓ nav_msgs/Odometry (/imu/odom)
RViz (path visualization)

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
IMU-only dead reckoning suffers from unbounded drift due to double integration of noisy acceleration.
This behavior is expected and intentionally exposed in Phase 2.

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
gps_anchor_fuser (custom ROS2 node)

### Pipeline
IMU Dead Reckoning (/imu/odom)
             +
GPS Fix (/gps/fix)
             ↓
gps_anchor_fuser
             ↓
Fused Odometry (/fused/odom)
Fused Path (/fused/path)

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

## Why This Design Matters
- Phase 2 exposes the fundamental limitations of IMU-only localization
- Phase 2.5 shows how GPS improves global consistency but remains noisy
- The system intentionally avoids jumping directly to EKF or SLAM
- Builds intuition before introducing probabilistic fusion methods

This phased approach mirrors real-world robotics development:
observe failure → mitigate → formalize

---

## Current Project Status
- Phase 2: IMU dead reckoning — Complete
- Phase 2.5: GPS anchoring — Complete
- Phase 3: EKF / Visual-Inertial SLAM — Planned

---

