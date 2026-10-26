# Live demo: phone → EKF → Foxglove (runbook)

## What it does
iPhone (Sensor Logger HTTP push) → `sensor_logger_bridge` → `phone_ekf` → Foxglove 3D view.
EKF inputs: IMU (iOS orientation + gravity-free acceleration), barometer pressure, compass bearing.
Outputs: `/ekf/odom`, `/ekf/pose`, `/ekf/path` (rolling trail), TF `map → base_link`, `/ekf/status` (JSON health).

## Start (3 terminals)
```bash
# 1) container (Mac terminal)
cd ~/Documents/phonefusion_nav
docker compose -f docker/compose.yml run --rm --service-ports ros

# 2) inside the container: build once
cd /ws/ros2_ws && colcon build --merge-install --packages-select sensor_logger_bridge phone_ekf && source install/setup.bash

# 3) run everything
ros2 launch phone_ekf live_demo.launch.py
```
Phone: Push URL `http://<mac-ip>:8000/data` (find with `ipconfig getifaddr en0`), start a recording.
**Hold the phone still for ~2 s at the start** so the compass can align heading and the bias is estimated.

## Foxglove
1. Install the Foxglove desktop app (or use Chrome; Safari blocks `ws://` from an https page).
2. Open connection → *Foxglove WebSocket* → `ws://localhost:8765`.
3. Add a **3D** panel. Display frame: `map`. Turn on `/ekf/path`, `/ekf/pose` and the TF `base_link`. Set the grid to ~1 m.
4. Add a **Plot** panel: `/ekf/odom.pose.pose.position.z` (height from barometer), `/ekf/odom.twist.twist.linear.x`.
5. Add a **Raw Messages** panel on `/ekf/status` for filter health.
Reset the trail any time: `ros2 service call /ekf/reset std_srvs/srv/Trigger`.

## Backup if the live link fails
```bash
cd /ws/ros2_ws/src/phone_ekf
PYTHONPATH=/ws/ros2_ws/src/sensor_logger_bridge:. python3 -m phone_ekf.run_offline /ws/phonedata/<capture>.jsonl
```
Writes `<capture>_ekf.png/.csv` (top view, height, speed). Or replay a recorded run into the live stack with `replay_capture`.

## What is honest to say about it
- **Orientation** comes from the iOS fused attitude. The EKF does not re-estimate it; it estimates the yaw *offset* (`psi`) between iOS' arbitrary heading and magnetic north, and corrects drift with the compass (3-sigma gated so indoor magnetic distortion is rejected rather than followed).
- **Height** is observable from the barometer (±~0.3 m).
- **Horizontal position** is double-integrated acceleration. It is bounded only by ZUPT when the phone is still, so it **will drift while you move** — expect a correct *shape* for a few metres, not metre-accurate tracking. This is exactly why GPS (outdoors) or camera (indoors) is the next measurement to add.
- **Known weakness:** a perfectly smooth constant-velocity slide looks "still" to an IMU-only detector (ZUPT fires). Normal hand motion or walking has enough acceleration to avoid this.
- Tests: `python3 -m pytest ros2_ws/src/phone_ekf/test` — simulated walks (end-point error ≈0.1 m, with idealised attitude), stationary 5 min (<5 cm), bad-compass rejection, barometer height, and a run on a real capture. **Not yet validated on real walking data**; the simulation proves the code, not real-world accuracy.

## Health checks (for your ROS supervisor tool)
`/ekf/status` JSON: `still`, `compass_aligned`, update counts (`zupt`, `compass`, `compass_rej`, `baro`, `baro_rej`), `last_nis` (innovation consistency; values ≫ 9 for the compass mean heading data is being rejected), `accel_bias`, `psi_deg`.
Topic rates: `/imu/data` ≈100 Hz, `/gps/fix` ≈1 Hz, `/barometer/pressure` ≈1 Hz, `/ekf/odom` ≈100 Hz.
