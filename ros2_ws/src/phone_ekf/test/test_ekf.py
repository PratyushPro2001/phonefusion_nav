import json
import os
import sys

import numpy as np

from phone_ekf.ekf import EkfConfig, PhoneEKF, Rz, quat_to_R, R_to_quat, wrap

RNG = np.random.default_rng(1)
HZ = 100.0


def simulate(ekf, duration, a_world_fn, R_world_true, psi_true, bias, a_noise=0.05, gyro=None,
             bearing_noise_deg=5.0, bearing_fn=None, baro_fn=None, t0=0.0):
    """Drive the EKF with synthetic sensor data. Phone orientation is R_world_true (phone->ENU);
    iOS reports R_ios = Rz(-psi_true) @ R_world_true. Returns true position array."""
    R_ios = Rz(-psi_true) @ R_world_true
    p_true, v_true = np.zeros(3), np.zeros(3)
    hist = []
    n = int(duration * HZ)
    for i in range(n):
        t = t0 + i / HZ
        a_w = a_world_fn(t - t0)
        p_true += v_true / HZ + 0.5 * a_w / HZ ** 2
        v_true += a_w / HZ
        a_user = R_world_true.T @ a_w + bias + RNG.normal(0, a_noise, 3)
        w = RNG.normal(0, 0.01, 3) if gyro is None else gyro(t - t0)
        ekf.imu(t, R_ios, a_user, w)
        if i % 1 == 0 and bearing_fn is not None:
            ekf.compass(bearing_fn(t - t0, R_world_true) + RNG.normal(0, bearing_noise_deg))
        if i % 100 == 0 and baro_fn is not None:
            ekf.baro(baro_fn(t - t0))
        hist.append(p_true.copy())
    return np.array(hist)


def true_bearing(t, Rw):
    axis = Rw[:, 1] if abs(Rw[2, 1]) < abs(Rw[2, 2]) else -Rw[:, 2]
    theta = np.arctan2(axis[1], axis[0])
    return np.degrees(np.pi / 2 - theta) % 360


def profile(t):
    """still 3 s, accelerate 1 s to 1.2 m/s east, walk ~8 s with gait, brake 1 s, still 5 s.
    Gait = zero-mean 1.8 Hz / 3.6 Hz accelerations (what a real step looks like to an IMU). Without it,
    constant-velocity motion is indistinguishable from standing still for an IMU-only detector."""
    a = np.zeros(3)
    if 3 <= t < 4:
        a += [1.2, 0, 0]
    if 12 <= t < 13:
        a += [-1.2, 0, 0]
    if 3 <= t < 13:
        a += [1.5 * np.sin(2 * np.pi * 1.8 * t), 0.3 * np.sin(2 * np.pi * 1.8 * t), 3.0 * np.sin(2 * np.pi * 3.6 * t)]
    return a


def walking_gyro(t):
    s = 0.3 if 3 <= t < 13 else 0.01
    return RNG.normal(0, s, 3)


def test_quat_roundtrip():
    R = Rz(0.7) @ np.array([[1, 0, 0], [0, 0.8, -0.6], [0, 0.6, 0.8]])
    assert np.allclose(quat_to_R(*R_to_quat(R)), R, atol=1e-9)


def test_straight_walk_end_position_and_heading():
    ekf = PhoneEKF()
    Rw = Rz(0.4)                       # phone flat, top edge pointing 0.4 rad CCW of East
    psi_true = 1.1
    bias = np.array([0.02, -0.015, 0.02])
    sim = simulate(ekf, 18.0, profile, Rw, psi_true, bias, gyro=walking_gyro, bearing_fn=true_bearing)
    err_end = np.linalg.norm(ekf.p[:2] - sim[-1][:2])
    assert 9.0 < sim[-1][0] < 12.0                                # truth: ~10 m east
    assert abs(wrap(ekf.psi - psi_true)) < np.radians(5)          # compass alignment found psi
    assert err_end < 1.0, err_end                                 # < 1 m on ~11 m
    assert np.linalg.norm(ekf.v) < 0.05                           # ZUPT holds v = 0 after stopping


def test_stationary_5_minutes_no_drift():
    ekf = PhoneEKF()
    simulate(ekf, 300.0, lambda t: np.zeros(3), Rz(0.2), 0.5, np.array([0.02, 0.01, -0.02]),
             bearing_fn=true_bearing)
    assert np.linalg.norm(ekf.p) < 0.05, ekf.p


def test_naive_integration_would_explode():
    """Same bias, no filter: shows what the EKF is protecting against (>~9 m in 30 s)."""
    bias = np.array([0.02, 0, 0])
    t = 30.0
    assert 0.5 * bias[0] * t * t > 8.0


def test_bad_compass_is_rejected():
    ekf = PhoneEKF()
    Rw, psi_true = Rz(0.3), 0.8
    simulate(ekf, 5.0, lambda t: np.zeros(3), Rw, psi_true, np.zeros(3), bearing_fn=true_bearing)
    before = ekf.psi
    # 2 s of magnetically disturbed heading: 90 deg off
    simulate(ekf, 2.0, lambda t: np.zeros(3), Rw, psi_true, np.zeros(3),
             bearing_fn=lambda t, R: true_bearing(t, R) + 90.0, t0=5.0)
    assert abs(wrap(ekf.psi - before)) < np.radians(8)
    assert ekf.stats['compass_rej'] > 100


def test_barometer_height():
    ekf = PhoneEKF()
    P0 = 101325.0
    # +1.5 m climb between t=5 s and t=15 s: pressure falls ~12 Pa/m
    def baro(t):
        h = 1.5 * np.clip((t - 5) / 10, 0, 1)
        return P0 * (1 - h / 44330.0) ** (1 / 0.190284) + RNG.normal(0, 1.0)
    simulate(ekf, 25.0, lambda t: np.zeros(3), Rz(0), 0, np.zeros(3), baro_fn=baro)
    assert abs(ekf.p[2] - 1.5) < 0.25, ekf.p[2]


def test_real_capture_runs_clean():
    here = os.path.dirname(__file__)
    root = os.path.abspath(os.path.join(here, '..', '..', 'sensor_logger_bridge'))
    sys.path.insert(0, root)
    from sensor_logger_bridge.parser import ImuAssembler, iter_samples
    f = os.path.join(root, 'test', 'data', 'sample.jsonl')
    asm, ticks, comp = ImuAssembler(), [], {}
    for line in open(f):
        samples = list(iter_samples(json.loads(line)))
        ticks += asm.feed(samples)
        comp.update({t: v['magneticBearing'] for n, t, v in samples if n == 'compass'})
    ekf = PhoneEKF(EkfConfig(compass_init_n=20))
    t0 = ticks[0]['t_ns']
    for d in ticks:
        o = d['orient']
        R = quat_to_R(o['qx'], o['qy'], o['qz'], o['qw'])
        a = [d['accel'][k] for k in 'xyz']
        g = [d['gyro'][k] for k in 'xyz']
        ekf.imu((d['t_ns'] - t0) / 1e9, R, a, g)
        if d['t_ns'] in comp:
            ekf.compass(comp[d['t_ns']])
    assert np.all(np.isfinite(ekf.x)) and np.all(np.isfinite(ekf.P))
    assert np.linalg.norm(ekf.p) < 1.0
