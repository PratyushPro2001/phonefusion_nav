# Testing & Validation Plan

Current automated coverage is a placeholder (`test/*.cpp` are empty; CI workflow is empty). Python packages
include ament lint tests (flake8, pep257, copyright). This plan defines what "working" means.

## Automated
1. **Lint:** `colcon test --packages-select <pkg>` for all Python packages.
2. **Unit tests to add**
   - `gps_anchor_fuser`: lat/lon → local XY conversion; jump rejection (>`max_gps_step_m`); accuracy rejection (>`max_hacc_m`); no update when speed < gate.
   - `vio_fuser`: ZUPT triggers after `zupt_count` quiet samples; does *not* trigger when VO speed > threshold; quaternion normalization.
   - `phone_imu_bridge`: parse sample SensaGram packets; unit scaling and axis swap/flip.
3. **CI:** enable `.github/workflows/ci.yaml` to build `ros2_ws` on Humble and run `colcon test`.

## Manual acceptance tests
| # | Scenario | Pass criterion |
|---|---|---|
| T1 | Phone flat & still, 60 s | `/vio_fused/odom` position change < 0.05 m; `/vio_gps/odom` change < 0.1 m |
| T2 | Walk 20 m straight | Path heading consistent with compass; GPS offset converges, no jumps > `max_gps_step_m` |
| T3 | Walk a closed loop | Start/end gap reported (drift metric) |
| T4 | Cover camera / low texture | VO holds last pose, no NaNs; fused odom stays finite |
| T5 | Weak GPS (indoors) | GPS updates rejected, odom continues from VIO |
| T6 | Network drop (UDP stops 5 s) | Nodes stay alive, resume on reconnect |

## Recording & replay
```bash
ros2 bag record -o bags/run01 /imu/data /gps/fix /camera/image_raw /vio/odom /vio_fused/odom /vio_gps/odom
ros2 bag play bags/run01
```
Record a bag per test above so tuning can be repeated without the phone. Keep large bags out of git (see roadmap).

## Metrics to log
- Static drift (m/min) with and without ZUPT
- Closed-loop end error (m) for IMU-only, VIO, VIO+GPS
- GPS rejection rate (% fixes rejected)
- Topic rates (`ros2 topic hz`) and end-to-end latency camera→`/vio_fused/odom`
