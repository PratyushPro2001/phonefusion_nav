# Sensor Logger Bridge (iPhone → ROS 2)

Replaces the old SensaGram/UDP `phone_imu_bridge` for iPhone data. Package: `ros2_ws/src/sensor_logger_bridge`.

## Verified payload format
From a real capture on an iPhone 16e (`phonedata/iphone16e_http_push_sample.jsonl`, 15 batches, 4.9 s):

```json
{"messageId":199,"sessionId":"…","deviceId":"…",
 "payload":[{"name":"gyroscope","time":1792991306277069800,"values":{"x":…,"y":…,"z":…}}, …]}
```

| Sensor name | Fields | Unit / note | Rate |
|---|---|---|---|
| `accelerometer` | x,y,z | m/s², **gravity removed** | 100 Hz |
| `accelerometeruncalibrated` | x,y,z | **g**, gravity included (|a|≈0.993) | 100 Hz |
| `gyroscope`, `gyroscopeuncalibrated` | x,y,z | rad/s | 100 Hz |
| `orientation` | qx,qy,qz,qw, yaw,pitch,roll | unit quaternion (norm 1±1e-7), rad | 100 Hz |
| `gravity` | x,y,z | m/s², derived from orientation (|g| exactly 9.80665) | 100 Hz |
| `magnetometeruncalibrated` | x,y,z | µT, includes hard-iron offset (|B|≈430 µT) | ≈97 Hz |
| `magnetometer`, `compass` | `magneticBearing` only | deg — **no xyz in the calibrated stream** | 100 Hz |
| `location` | latitude, longitude, altitude, horizontalAccuracy, verticalAccuracy, speed, bearing, … | `altitude` is ellipsoidal; accuracies in m; −1 = invalid | ≈1 Hz |
| `barometer` | pressure, relativeAltitude | hPa, m | ≈1 Hz |

Things that matter for estimation:
- `time` is an **integer nanosecond epoch on the phone clock**. Python ints keep it exact; do not convert to float32.
- gyro, accel and orientation share the **identical timestamp** in each 100 Hz tick; uncalibrated sensors are 2–5 ms later. The bridge joins on exact equality.
- Batches (100 ms setting) arrive as 57–180 samples. The **first message after a push starts can hold a backlog** (one batch had 1024 samples spanning ~3 s). The bridge handles this: stamps come from sample times, not arrival.
- Batch arrival is jittery, so `time_mode: offset` (default) estimates `offset = min(arrival − newest_sample_time)` over the last 300 batches and stamps `phone_time + offset`. Alternatives: `phone` (raw phone clock; fine if both clocks are NTP-synced) and `arrival` (not recommended).

## Topics published
`/imu/data`, `/gps/fix` (covariance = reported accuracy²; fixes with accuracy < 0 are dropped), `/mag/uncalibrated` (Tesla),
`/barometer/pressure` (Pa), `/barometer/relative_altitude`.
IMU covariances come from `static_summary.csv`: `gyro_std 0.0007 rad/s`, `accel_std 0.008 m/s²` (parameters); orientation std is a placeholder.
Axes stay in the iOS device frame (x right, y up, z out of the screen).

## Run
```bash
docker compose -f docker/compose.yml build            # once
docker compose -f docker/compose.yml run --rm --service-ports ros
# inside the container
cd ros2_ws && colcon build --merge-install --packages-select sensor_logger_bridge && source install/setup.bash
ros2 launch sensor_logger_bridge sensor_logger_bridge.launch.py record_path:=/ws/phonedata/walk01.jsonl
```
Phone: Push URL `http://192.168.1.252:8000/data` (use your current Mac IP), batch period 100 ms, "Skip Writing" off (keeps a copy on the phone).

## Test without the phone
```bash
ros2 run sensor_logger_bridge replay_capture /ws/phonedata/iphone16e_http_push_sample.jsonl --speed 1
ros2 topic hz /imu/data          # ≈100 Hz
ros2 topic echo /gps/fix --once
```
Unit tests (no ROS needed): `cd ros2_ws/src/sensor_logger_bridge && python3 -m pytest test/test_parser.py`.

## Not done yet
- Camera: the app can send base64 JPEG (“Send Images”). The payload format for images is not captured yet; record one batch with Camera enabled and I'll add `/camera/image_raw`.
- Frame conversion to ROS REP-103 (x forward, y left, z up) and `imu_link → base_link` static TF.
- Orientation covariance is a placeholder; compare iOS orientation with your own filter.
- Live test of the ROS node itself (written but not run: rclpy was not available where it was developed).
