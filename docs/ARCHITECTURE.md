# Architecture

PhoneFusion-Nav turns a smartphone into a ROS 2 (Humble) sensor head and builds a
progressively better odometry stream from it. The project was deliberately built in
phases — *observe failure → mitigate → formalize* — so each phase exposes a limitation
the next one fixes.

## Phases

| Phase | Goal | Output topic(s) | Status |
|---|---|---|---|
| 2 | IMU-only dead reckoning (shows unbounded drift) | `/imu/odom`, `/imu/path` | Complete |
| 2.5 | GPS low-gain anchor on IMU odom | `/fused/odom`, `/fused/path` | Complete |
| 3 | Visual-inertial fusion (VO position + IMU orientation, VO-gated ZUPT) | `/vio_fused/odom`, `/vio_fused/path` | Complete |
| 4A | Motion-gated GPS anchor on VIO | `/vio_gps/odom`, `/vio_gps/path` | Complete |
| 4B | SLAM integration / loop closure | — | Planned |
| — | EKF refactor (replace hand-tuned fusion) | — | Planned |

## Current (Phase 4A) data flow

```
iPhone/Android ── UDP (SensaGram, :5555) ──► phone_imu_bridge ──► /imu/data  (sensor_msgs/Imu)
                                                   └────────────► /gps/fix   (sensor_msgs/NavSatFix)
Phone camera ── MJPEG (DroidCam :4747) ──► ipcam_bridge ──► /camera/image_raw
                                                   │
                                          feature_tracker ──► /camera/features (debug image)
                                                   │
                                           vo_estimator ──► /vio/odom, /vio/path
                                                                 │
/imu/data ───────────────────────────────────────────────────────┤
                                                                 ▼
                                                           vio_fuser ──► /vio_fused/odom, /vio_fused/path (+TF)
                                                                 │
/gps/fix ────────────────────────────────────────────────────────┤
                                                                 ▼
                                                        gps_anchor_fuser ──► /vio_gps/odom, /vio_gps/path
```

Launch everything with `ros2 launch phonefusion_nav_bringup vio_full_stack.launch.py`.

## Design decisions

- **Orientation from the IMU quaternion, position from VO.** VO supplies translation;
  its own rotation is replaced by the (normalized) phone orientation, which is far more stable.
- **VO-gated ZUPT.** Zero-velocity updates only fire when accel *and* gyro are quiet
  *and* VO speed is below threshold, so smooth constant-velocity motion is not mistaken for standing still.
- **Motion-gated GPS.** GPS noise is metres; applying it while stationary makes the pose
  random-walk. The correction offset is only updated when speed ≥ `min_speed_for_gps_update`,
  with accuracy and jump rejection. See `SENSOR_DATA_ANALYSIS.md` for measured numbers.
- **Visual scale is arbitrary.** `vo_estimator` advances by a fixed `step_scale` per accepted
  frame pair, so VO is not metric. IMU/GPS are what keep the output roughly metric.

## Repository layout

```
phonefusion_nav/
├── ros2_ws/src/            ← ACTIVE workspace (build this)
│   ├── phone_imu_bridge    UDP JSON → /imu/data, /gps/fix
│   ├── imu_deadreckoning   Phase 2: /imu/data → /imu/odom, /imu/path
│   ├── ipcam_bridge        MJPEG URL → /camera/image_raw
│   ├── feature_tracker     Shi-Tomasi + LK optical flow (visualization)
│   ├── vo_estimator        essential-matrix VO → /vio/odom
│   ├── vio_fuser           VO + IMU + ZUPT → /vio_fused/odom
│   ├── gps_anchor_fuser    motion-gated GPS anchor → /vio_gps/odom
│   ├── tag_pose_bridge     AprilTag detections → /tag_pose
│   ├── phone_vio_lite      earlier single-node VO prototype (superseded)
│   └── phonefusion_nav_bringup   launch files (vio_full_stack, vision_tag_bringup, AprilTag camera)
├── src/                    ← LEGACY 10-day-sprint scaffold (C++ stubs + early Python nodes)
│   ├── ekf2d               yaw-only "EKF" (the only non-stub C++ node)
│   ├── fusion_node, gps_bridge, odom_plotter, imu_orientation_viz   early Python nodes
│   └── apriltag_pose, phone_cam_bridge, tiny_planner, pure_pursuit_controller   stubs (~280 B each)
├── phonefusion_nav_bringup/   legacy launch (full_system, demo_turtlesim, phase2_imu_bringup)
├── phonedata/              recorded sensor logs + analysis script
├── config/, test/, .github/workflows/   empty placeholders
└── docs/                   this documentation
```
