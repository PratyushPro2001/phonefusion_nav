# Sensor Data Analysis — Static Recording (South Brunswick Public Library)

Source: `phonedata/south_brunswick_public_library/` (exported from the **Sensor Logger** app),
analyzed by `analyze_static.py` → `analysis_out/` (5 plots + `static_summary.csv`).

## Recording
| Item | Value |
|---|---|
| Device | iPhone 16e, iOS 26.6.2, Sensor Logger 1.68 |
| Start | 2026-10-25 18:25:53 (America/New_York) |
| Duration | ≈ 1853 s (≈ 31 min), phone stationary |
| IMU rate | ≈ 100 Hz (99.9 Hz measured), GPS ≈ 1 Hz (1860 fixes), barometer present |
| Sensors | Accelerometer, Gravity, Gyroscope, Orientation, Magnetometer, Compass, Barometer, Location (+ uncalibrated accel/gyro/mag). `Annotation.csv` is empty. |

Purpose: characterize noise, bias and drift with no motion, to set realistic ZUPT thresholds, filter gains
and GPS gates.

## Key numbers (from `static_summary.csv`)
| Sensor | Std (per axis) | Mean / bias | Takeaway |
|---|---|---|---|
| Accelerometer (gravity removed) | 0.007–0.008 m/s² | x 0.0002, y −0.0015, z 0.0207 m/s² | Residual bias ≈ 0.02 m/s². Double-integrated, 0.02 m/s² gives ≈ 0.5·0.02·t² ≈ 9 m after 30 s. This is why IMU-only drifts. |
| Gyroscope (calibrated) | 0.0007 rad/s | ≈ 0 (≤ 1e-5) | Already bias-corrected by iOS. |
| Gyroscope (uncalibrated) | 0.0007 rad/s | x 0.0025, y 0.0027, z 0.0007 rad/s | Raw bias ≈ 0.15 °/s on x/y — matters if you use raw data. |
| Gravity vector | z std 3e-5 | (0.139, 0.044, −9.806) m/s² | Phone ≈ flat, slightly tilted; magnitude ≈ 9.806. |
| Magnetometer | x 0.80, y 0.24, z 0.41 µT | (24.7, −24.7, −73.5) µT → |B| ≈ 81 µT | Expected ≈ 50 µT for this area: indoor magnetic distortion, so compass heading is unreliable inside. |
| GPS horizontal accuracy | mean 3.4 m, max 118 m | first fix 19.7 m | Fix quality varies a lot indoors/at start. |
| GPS position wander | lat span ≈ 18 m, std ≈ 6.6 m; lon std ≈ 1.7 m | — | A stationary phone still "moves" by metres → justifies motion-gated GPS. |

## Implications for the pipeline
- **ZUPT thresholds** (`zupt_accel_thresh 0.08`, `zupt_gyro_thresh 0.06`) sit ≈ 10× above the measured noise
  std (0.008 m/s², 0.0007 rad/s). They are safe against false motion; they could be tightened (e.g. 0.04 / 0.02) if ZUPT misses slow motion.
- **GPS gate** (`max_hacc_m 15`, `max_gps_step_m 12`, `min_speed_for_gps_update 0.25`) is consistent with the observed ±18 m static wander and 118 m worst-case accuracy.
- **Do not trust magnetic heading indoors**; keep using the game rotation vector (no magnetometer) for orientation, as the project already does.
- **Barometer** data exists but is unused — a possible height constraint later.

## Caveats / things to fix in `analyze_static.py`
- Uncalibrated accelerometer values are in **g** (z ≈ −0.998), but the summary labels the unit `m/s^2`. Fix the label or convert.
- `analysis_out/summary_text.txt` is empty — the script doesn't populate it (or the run failed after the CSV step).
- The raw `.zip` export (65 MB) and ~190 MB of CSVs live in the repo folder; see roadmap on keeping data out of git.
- This is a **static** dataset only. It cannot validate VO, GPS anchoring under motion, or loop-closure error; record walking datasets for that (see `TESTING.md`).
