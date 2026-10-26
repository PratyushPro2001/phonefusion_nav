# Status & Roadmap

_Last updated: 2026-10-26_

## Status
| Phase | State |
|---|---|
| 2 – IMU dead reckoning | ✅ Complete |
| 2.5 – GPS-anchored IMU odom | ✅ Complete |
| 3 – VO + IMU fusion (manual) | ✅ Complete |
| 4A – Motion-gated GPS anchor for VIO | ✅ Complete |
| 4B – SLAM / loop closure | ⏳ Planned |
| EKF refactor | ⏳ Planned |
| Static sensor characterization (iPhone log) | ✅ Done (see `SENSOR_DATA_ANALYSIS.md`) |

## Known gaps & inconsistencies found during review
**Docs**
1. `Phase2_Phase2_5_Documentation.md` lists Phase 3 as "EKF / Visual-Inertial SLAM — Planned"; `README.md` says Phase 3 (manual VIO) is complete. Align them (this index supersedes the old note).
2. `PhoneFusion-Nav_FULL_README.md` describes the original 6-node **C++** plan; most of that is not what was built (the working stack is Python in `ros2_ws`). Mark it as "original plan / historical".
3. `PHONEFUSION_PHASE2.md` says "Execution later — code-first workflow"; Phase 2 is now complete.

**Repo hygiene**
4. `LICENSE` was empty (README claims MIT) — a proper MIT file is provided.
5. `config/*.yaml`, `test/*.cpp`, `.github/workflows/ci.yaml` are empty. CI is therefore a no-op.
6. Stub C++ nodes (`apriltag_pose`, `phone_cam_bridge`, `tiny_planner`, `pure_pursuit_controller`) only print a message.
7. Two launch trees: legacy `phonefusion_nav_bringup/` (top level) and `ros2_ws/src/phonefusion_nav_bringup/` (active; has `COLCON_IGNORE` and no `package.xml`/`CMakeLists.txt` visible, only `setup.py`). Verify it builds under `colcon`, or the `ros2 launch phonefusion_nav_bringup …` command in the README will fail.
8. Legacy `full_system.launch.py` references `phone_imu_bridge` executable `imu_bridge` (C++ plan), while the active package exposes `phone_imu_bridge_node`.
9. `vision_tag_bringup.launch.py` is duplicated in two places.
10. `ros2_ws/src/phonefusion_nav/` and `launch/__pycache__` are leftovers; `.DS_Store` files exist in several folders (already in `.gitignore`).
11. `phonedata/` holds a 65 MB zip plus ~190 MB of CSVs. Add `phonedata/` (or `*.zip`, `*.csv`) to `.gitignore` or use Git LFS.
12. `phone_vio_lite` is superseded by `ipcam_bridge → vo_estimator`; archive or delete.
13. Parameter defaults in code differ from launch values (`zupt_damp`, `min_speed_for_gps_update`). Document or unify.
14. VO scale is arbitrary (`step_scale`); camera intrinsics are placeholders (525/525/320/240). Calibrate the phone camera.
15. No static TF between `imu_link`, `camera`, `gps_link`, `base_link`.
16. Maintainer name/email in packages are placeholders in the starter-kit templates.

## Roadmap
**Next (short term)**
- [ ] Clean up items 4–13 above; commit docs.
- [ ] Record walking datasets (straight line, closed loop) as bags; compute drift metrics (`TESTING.md`).
- [ ] Camera calibration → real intrinsics in `vo_estimator`.
- [ ] Unit tests for ZUPT and GPS gating; enable CI.

**Medium term**
- [ ] Publish static TF tree and add RViz config for the Phase 4A stack.
- [ ] Use the barometer for a height constraint; evaluate raw (uncalibrated) gyro bias estimation online.
- [ ] Replace hand-tuned fusion with an EKF/ESKF (position, velocity, orientation, biases).

**Long term (Phase 4B)**
- [ ] Feed `/vio_gps/odom` into a SLAM backend (e.g. RTAB-Map / ORB-SLAM3 / slam_toolbox with visual odometry) with loop closure.
- [ ] AprilTag landmarks as absolute resets (`tag_pose_bridge` already outputs `/tag_pose`).
- [ ] Demo video + recorded bag for the portfolio README.
