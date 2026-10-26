#!/usr/bin/env python3
"""
Static-recording analysis for Sensor Logger CSV exports.

Usage:
    python analyze_static.py /path/to/folder            # defaults: trim 5 s each end
    python analyze_static.py /path/to/folder --trim 10  # cut 10 s at start/end
    python analyze_static.py /path/to/folder --show     # also open plot windows

Outputs (saved into <folder>/analysis_out/):
    1_timeseries.png, 2_drift.png, 3_psd.png, 4_allan.png, 5_orientation.png
    static_summary.csv   (bias, std, variance per sensor axis)
Needs: pandas, numpy, scipy, matplotlib
"""
import argparse
import os

import numpy as np
import pandas as pd
import matplotlib

import matplotlib.pyplot as plt
from scipy.signal import welch

G0 = 9.80665
AXES = ["x", "y", "z"]


# ----------------------------------------------------------------- loading
def load(folder, name, trim):
    path = os.path.join(folder, name + ".csv")
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    df = pd.read_csv(path)
    if df.empty:
        return None
    if "seconds_elapsed" in df.columns:
        t = df["seconds_elapsed"].astype(float).to_numpy()
    elif "time" in df.columns:
        raw = df["time"].astype(float).to_numpy()
        scale = 1e9 if raw[0] > 1e15 else (1e3 if raw[0] > 1e11 else 1.0)
        t = (raw - raw[0]) / scale
    else:
        return None
    df = df.copy()
    df["t"] = t - t[0]
    tmax = df["t"].iloc[-1]
    if trim > 0 and tmax > 4 * trim:
        df = df[(df["t"] >= trim) & (df["t"] <= tmax - trim)].reset_index(drop=True)
    return df


def rate_info(t):
    dt = np.diff(t)
    dt = dt[dt > 0]
    fs = 1.0 / np.median(dt)
    return fs, dt.std(), dt.max()


def allan_dev(x, fs):
    n = len(x)
    max_m = n // 9
    if max_m < 2:
        return np.array([]), np.array([])
    ms = np.unique(np.logspace(0, np.log10(max_m), 40).astype(int))
    taus, adev = [], []
    for m in ms:
        k = n // m
        y = x[: k * m].reshape(k, m).mean(axis=1)
        adev.append(np.sqrt(0.5 * np.mean(np.diff(y) ** 2)))
        taus.append(m / fs)
    return np.array(taus), np.array(adev)


# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder", nargs="?", default=".")
    ap.add_argument("--trim", type=float, default=5.0,
                    help="seconds to cut from start and end (phone being placed/picked up)")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")

    out = os.path.join(args.folder, "analysis_out")
    os.makedirs(out, exist_ok=True)

    sensors = {
        "Accelerometer": "m/s^2", "AccelerometerUncalibrated": "m/s^2",
        "Gyroscope": "rad/s", "GyroscopeUncalibrated": "rad/s",
        "Magnetometer": "uT", "MagnetometerUncalibrated": "uT",
        "Gravity": "m/s^2",
    }
    data, rows = {}, []

    print("=" * 70)
    print(f"Folder: {os.path.abspath(args.folder)}   (trim {args.trim}s each end)")
    print("=" * 70)

    for name, unit in sensors.items():
        df = load(args.folder, name, args.trim)
        if df is None or not all(a in df.columns for a in AXES):
            print(f"[skip] {name}: missing or empty")
            continue
        data[name] = df
        fs, dt_std, dt_max = rate_info(df["t"].to_numpy())
        dur = df["t"].iloc[-1] - df["t"].iloc[0]
        print(f"\n{name}  [{unit}]  {len(df)} samples, {dur/60:.1f} min, "
              f"~{fs:.1f} Hz, dt jitter {dt_std*1000:.2f} ms, max gap {dt_max*1000:.0f} ms")
        for a in AXES:
            v = df[a].to_numpy()
            rows.append(dict(sensor=name, axis=a, unit=unit, mean=v.mean(), std=v.std(ddof=1),
                             variance=v.var(ddof=1), peak_to_peak=np.ptp(v), fs_hz=fs))
            print(f"   {a}: mean {v.mean():+.6f}   std {v.std(ddof=1):.6f}   "
                  f"var (R start) {v.var(ddof=1):.3e}   p2p {np.ptp(v):.5f}")

        if name.startswith("Accelerometer") or name == "Gravity":
            mag = np.linalg.norm(df[AXES].to_numpy(), axis=1)
            g_ref = G0 if mag.mean() > 5 else 1.0
            unit_note = "m/s^2" if g_ref == G0 else "g (units look like g, not m/s^2!)"
            ax = max(AXES, key=lambda a: abs(df[a].mean()))
            bias_ax = df[ax].mean() - np.sign(df[ax].mean()) * g_ref
            print(f"   |a| mean {mag.mean():.5f} (expect {g_ref:.5f}, {unit_note}); "
                  f"gravity on axis '{ax}', bias there ~ {bias_ax:+.5f}")

    if not data:
        print("\nNo sensor files found. Check the folder path.")
        return

    pd.DataFrame(rows).to_csv(os.path.join(out, "static_summary.csv"), index=False)

    groups = [
        ("Accelerometer", "AccelerometerUncalibrated", "m/s^2"),
        ("Gyroscope", "GyroscopeUncalibrated", "rad/s"),
        ("Magnetometer", "MagnetometerUncalibrated", "uT"),
    ]

    # ---- 1. time series (calibrated vs uncalibrated overlay)
    fig, axs = plt.subplots(3, 3, figsize=(15, 9), squeeze=False)
    for r, (cal, unc, unit) in enumerate(groups):
        for c, a in enumerate(AXES):
            for nm, style in ((unc, dict(alpha=0.6, lw=0.6)), (cal, dict(alpha=0.9, lw=0.6))):
                if nm in data:
                    axs[r][c].plot(data[nm]["t"], data[nm][a], label=nm, **style)
            axs[r][c].set_title(f"{cal.replace('ometer','')} {a} [{unit}]")
            axs[r][c].set_xlabel("s")
            axs[r][c].grid(alpha=0.3)
        axs[r][0].legend(fontsize=7)
    fig.suptitle("Static recording: raw time series")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "1_timeseries.png"), dpi=120)

    # ---- 2. drift (10 s rolling mean)
    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    for r, (cal, unc, unit) in enumerate(groups):
        for nm, ls in ((cal, "-"), (unc, "--")):
            if nm not in data:
                continue
            df = data[nm]
            fs, _, _ = rate_info(df["t"].to_numpy())
            w = max(int(fs * 10), 2)
            for a in AXES:
                axs[r].plot(df["t"], df[a].rolling(w, min_periods=w).mean(), ls=ls,
                            label=f"{nm} {a}")
        axs[r].set_title(f"{cal}: 10 s rolling mean (drift / warm-up)  [{unit}]")
        axs[r].legend(fontsize=6, ncol=3)
        axs[r].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "2_drift.png"), dpi=120)

    # ---- 3. power spectral density (noise colour, vibration peaks)
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
    for i, (cal, _, unit) in enumerate(groups):
        if cal not in data:
            continue
        df = data[cal]
        fs, _, _ = rate_info(df["t"].to_numpy())
        for a in AXES:
            v = df[a].to_numpy()
            f, p = welch(v - v.mean(), fs=fs, nperseg=min(4096, max(len(v) // 4, 64)))
            axs[i].semilogy(f, p, label=a)
        axs[i].set_title(f"{cal} PSD  [({unit})^2/Hz]")
        axs[i].set_xlabel("Hz")
        axs[i].legend()
        axs[i].grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "3_psd.png"), dpi=120)

    # ---- 4. Allan deviation (accel + gyro)
    fig, axs = plt.subplots(1, 2, figsize=(13, 5))
    print("\nAllan deviation (white-noise level at tau~1 s, and minimum = bias-instability scale):")
    for i, (cal, unc, unit) in enumerate(groups[:2]):
        for nm, ls in ((cal, "-"), (unc, "--")):
            if nm not in data:
                continue
            df = data[nm]
            fs, _, _ = rate_info(df["t"].to_numpy())
            for a in AXES:
                taus, adev = allan_dev(df[a].to_numpy(), fs)
                if len(taus) == 0:
                    continue
                axs[i].loglog(taus, adev, ls=ls, label=f"{nm} {a}")
                n1 = np.interp(1.0, taus, adev) if taus[0] <= 1.0 <= taus[-1] else np.nan
                print(f"   {nm:27s} {a}: sigma(1s) {n1:.3e}   min {adev.min():.3e} "
                      f"at tau {taus[adev.argmin()]:.1f}s  ({unit})")
        axs[i].set_title(f"{cal} Allan deviation [{unit}]")
        axs[i].set_xlabel("tau [s]")
        axs[i].grid(alpha=0.3, which="both")
        axs[i].legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "4_allan.png"), dpi=120)

    # ---- 5. orientation (should be flat when static)
    ori = load(args.folder, "Orientation", args.trim)
    if ori is not None and all(c in ori.columns for c in ("roll", "pitch", "yaw")):
        fig, axs = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
        print("\nOrientation (assuming radians, shown in degrees):")
        for i, c in enumerate(("roll", "pitch", "yaw")):
            deg = np.degrees(ori[c].to_numpy())
            axs[i].plot(ori["t"], deg)
            axs[i].set_ylabel(f"{c} [deg]")
            axs[i].grid(alpha=0.3)
            print(f"   {c}: mean {deg.mean():+.4f}  std {deg.std(ddof=1):.5f}  "
                  f"p2p {np.ptp(deg):.4f} deg")
        fig.tight_layout()
        fig.savefig(os.path.join(out, "5_orientation.png"), dpi=120)

    # ---- extras: barometer and GPS noise at rest
    baro = load(args.folder, "Barometer", args.trim)
    if baro is not None and "pressure" in baro.columns:
        print(f"\nBarometer: pressure mean {baro['pressure'].mean():.3f}  "
              f"std {baro['pressure'].std(ddof=1):.4f} (hPa)")
    loc = load(args.folder, "Location", 0)
    if loc is not None and {"latitude", "longitude"} <= set(loc.columns) and len(loc) > 5:
        lat0 = np.radians(loc["latitude"].mean())
        n_m = loc["latitude"].std(ddof=1) * 111_320
        e_m = loc["longitude"].std(ddof=1) * 111_320 * np.cos(lat0)
        print(f"\nGPS at rest ({len(loc)} fixes): north std {n_m:.2f} m, east std {e_m:.2f} m "
              f"-> GPS R starts near {max(n_m, e_m)**2:.2f} m^2")

    print(f"\nSaved plots + static_summary.csv to: {os.path.abspath(out)}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
