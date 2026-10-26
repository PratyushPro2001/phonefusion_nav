"""Phone EKF: IMU (iOS orientation + user acceleration) + barometer + compass bearing. No ROS imports.

World frame: ENU (x East, y North, z Up).
State x (10): p(3) world position [m], v(3) world velocity [m/s],
              ba(3) accelerometer bias in the phone frame [m/s^2], psi yaw correction [rad].
Attitude is NOT estimated here: roll/pitch/yaw come from the iOS orientation quaternion R_ios
(phone -> z-up reference frame, convention verified on real data).  The EKF estimates psi, the rotation
about Z that aligns iOS' arbitrary reference heading with magnetic north, and tracks its drift:
    R_world = Rz(psi) @ R_ios
Propagation (100 Hz):  a_w = R_world (a_user - ba);  p += v dt + a_w dt^2/2;  v += a_w dt
Updates:
  * ZUPT      v = 0                         (when still: low gyro and low user-acceleration)
  * still-bias a_user = ba                  (true acceleration is 0 when still)
  * barometer relative height -> p_z
  * compass   heading of the phone axis that points horizontally -> psi   (3-sigma gated)
Honest limitation: horizontal position comes from double integration and is only bounded by ZUPT;
it WILL drift while moving.  Height (baro) and heading (compass) are observable.
"""
from collections import deque
from dataclasses import dataclass
import numpy as np

G = 9.80665
NP, NV, NB, NPSI = slice(0, 3), slice(3, 6), slice(6, 9), 9
N = 10


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def Rz(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_to_R(x, y, z, w):
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = (t + 1.0) ** 0.5 * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = (1.0 + R[i, i] - R[j, j] - R[k, k]) ** 0.5 * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


@dataclass
class EkfConfig:
    accel_std: float = 0.008          # m/s^2   measured (static_summary.csv)
    tilt_std_deg: float = 0.5         # attitude error -> gravity leak g*sin(tilt) into horizontal accel
    ba_walk: float = 2e-4             # m/s^2/sqrt(s)  accel-bias random walk (placeholder until Allan fit)
    psi_walk_deg: float = 0.05        # deg/sqrt(s)   yaw drift of the iOS reference
    z_walk: float = 0.1               # m/sqrt(s)  unmodelled vertical motion (stairs/lift look 'still' to ZUPT)
    ba_init_std: float = 0.05
    psi_init_std: float = 0.2         # rad after compass alignment
    max_dt: float = 0.2
    # stationarity detector
    still_window: int = 30            # samples (0.3 s at 100 Hz)
    still_gyro: float = 0.08          # rad/s  (mean |w|)
    still_accel: float = 0.15         # m/s^2  (mean |a_user|)
    zupt_every: int = 5               # apply at 20 Hz
    zupt_std: float = 0.03            # m/s
    still_bias_std: float = 0.08      # m/s^2  measurement noise of "a_user = ba"
    # barometer
    baro_std: float = 0.3             # m
    baro_init_n: int = 3
    # compass
    use_compass: bool = True
    compass_std_deg: float = 15.0
    compass_init_n: int = 100
    min_horizontal: float = 0.35      # axis must have >= this horizontal component
    gate_chi2: float = 9.0            # 3-sigma, 1 dof


class PhoneEKF:
    def __init__(self, cfg=None):
        self.cfg = cfg or EkfConfig()
        self.reset()

    def reset(self):
        c = self.cfg
        self.x = np.zeros(N)
        self.P = np.diag([1e-4] * 3 + [1e-2] * 3 + [c.ba_init_std ** 2] * 3 + [np.pi ** 2])
        self.t = None
        self.R_ios = np.eye(3)
        self.a_user = np.zeros(3)
        self._gyro_win = deque(maxlen=c.still_window)
        self._acc_win = deque(maxlen=c.still_window)
        self._k = 0
        self.still = False
        self.p0 = None
        self._baro_init = []
        self._psi_init = []
        self.aligned = False
        self.stats = {'zupt': 0, 'bias': 0, 'baro': 0, 'baro_rej': 0, 'compass': 0, 'compass_rej': 0}
        self.last_nis = {'compass': 0.0, 'baro': 0.0}

    # ----------------------------------------------------------------- accessors
    @property
    def p(self): return self.x[NP]
    @property
    def v(self): return self.x[NV]
    @property
    def ba(self): return self.x[NB]
    @property
    def psi(self): return float(self.x[NPSI])

    def R_world(self):
        return Rz(self.psi) @ self.R_ios

    # ----------------------------------------------------------------- core update
    def _update(self, z, h, H, R, gate=None, key=None):
        y = np.atleast_1d(z - h)
        H = np.atleast_2d(H)
        R = np.atleast_2d(R)
        S = H @ self.P @ H.T + R
        Sinv = np.linalg.inv(S)
        nis = float(y @ Sinv @ y)
        if key:
            self.last_nis[key] = nis
        if gate is not None and nis > gate:
            return False
        K = self.P @ H.T @ Sinv
        self.x = self.x + K @ y
        I_KH = np.eye(N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.x[NPSI] = wrap(self.x[NPSI])
        return True

    # ----------------------------------------------------------------- IMU
    def imu(self, t, R_ios, a_user, gyro):
        """t seconds (monotonic), R_ios 3x3 phone->ref, a_user m/s^2 (gravity removed, phone frame), gyro rad/s."""
        c = self.cfg
        self.R_ios = R_ios
        self.a_user = np.asarray(a_user, float)
        if self.t is None:
            self.t = t
            return
        dt = t - self.t
        if dt <= 0:
            return
        self.t = t
        if dt > c.max_dt:          # data gap: do not integrate across it
            self.x[NV] = 0.0
            return
        Rw = self.R_world()
        a_w = Rw @ (self.a_user - self.ba)
        p, v = self.p.copy(), self.v.copy()
        self.x[NP] = p + v * dt + 0.5 * a_w * dt * dt
        self.x[NV] = v + a_w * dt

        F = np.eye(N)
        F[NP, NV] = np.eye(3) * dt
        F[NP, NB] = -0.5 * Rw * dt * dt
        F[NV, NB] = -Rw * dt
        dRa = np.array([-a_w[1], a_w[0], 0.0])
        F[NP, NPSI] = 0.5 * dRa * dt * dt
        F[NV, NPSI] = dRa * dt
        q = c.accel_std ** 2 + (G * np.radians(c.tilt_std_deg)) ** 2
        Q = np.zeros((N, N))
        Q[NP, NP] = np.eye(3) * 0.25 * q * dt ** 4
        Q[NV, NV] = np.eye(3) * q * dt ** 2
        Q[NP, NV] = Q[NV, NP] = np.eye(3) * 0.5 * q * dt ** 3
        Q[NB, NB] = np.eye(3) * c.ba_walk ** 2 * dt
        Q[NPSI, NPSI] = np.radians(c.psi_walk_deg) ** 2 * dt
        Q[2, 2] += c.z_walk ** 2 * dt
        self.P = F @ self.P @ F.T + Q

        # stationarity -> ZUPT + bias update
        self._gyro_win.append(float(np.linalg.norm(gyro)))
        self._acc_win.append(float(np.linalg.norm(self.a_user)))
        self._k += 1
        full = len(self._gyro_win) == c.still_window
        self.still = bool(full and np.mean(self._gyro_win) < c.still_gyro and np.mean(self._acc_win) < c.still_accel)
        if self.still and self._k % c.zupt_every == 0:
            H = np.zeros((3, N)); H[:, NV] = np.eye(3)
            self._update(np.zeros(3), self.v, H, np.eye(3) * c.zupt_std ** 2)
            self.stats['zupt'] += 1
            Hb = np.zeros((3, N)); Hb[:, NB] = np.eye(3)
            self._update(self.a_user, self.ba, Hb, np.eye(3) * c.still_bias_std ** 2)
            self.stats['bias'] += 1

    # ----------------------------------------------------------------- barometer
    def baro(self, pressure_pa):
        c = self.cfg
        if self.p0 is None:
            self._baro_init.append(pressure_pa)
            if len(self._baro_init) >= c.baro_init_n:
                self.p0 = float(np.mean(self._baro_init))
            return
        h = 44330.0 * (1.0 - (pressure_pa / self.p0) ** 0.190284)
        H = np.zeros((1, N)); H[0, 2] = 1.0
        ok = self._update(np.array([h]), np.array([self.x[2]]), H, np.array([[c.baro_std ** 2]]),
                          gate=c.gate_chi2 * 4, key='baro')   # looser gate: height can really change
        self.stats['baro' if ok else 'baro_rej'] += 1

    # ----------------------------------------------------------------- compass
    def _heading_axis(self):
        R = self.R_ios
        if abs(R[2, 1]) < abs(R[2, 2]):
            axis = R[:, 1]        # phone top edge (flat or tilted)
        else:
            axis = -R[:, 2]       # rear camera direction (held upright)
        return axis

    def compass(self, bearing_deg):
        """bearing_deg: magnetic heading, clockwise from north, of the phone's pointing axis."""
        c = self.cfg
        if not c.use_compass:
            return
        axis = self._heading_axis()
        if np.hypot(axis[0], axis[1]) < c.min_horizontal:
            return
        theta_axis = np.arctan2(axis[1], axis[0])                 # CCW from ref-x
        theta_mag = np.pi / 2 - np.radians(bearing_deg)           # ENU: CCW from East
        raw = wrap(theta_mag - theta_axis)                        # = psi if everything agrees
        if not self.aligned:
            self._psi_init.append(raw)
            if len(self._psi_init) >= c.compass_init_n:
                psi0 = float(np.angle(np.mean(np.exp(1j * np.array(self._psi_init)))))
                self.x[NPSI] = psi0
                self.P[NPSI, :] = 0.0; self.P[:, NPSI] = 0.0
                self.P[NPSI, NPSI] = c.psi_init_std ** 2
                self.aligned = True
            return
        H = np.zeros((1, N)); H[0, NPSI] = 1.0
        y = wrap(raw - self.psi)
        ok = self._update(np.array([y]), np.array([0.0]), H, np.array([[np.radians(c.compass_std_deg) ** 2]]),
                          gate=c.gate_chi2, key='compass')
        self.stats['compass' if ok else 'compass_rej'] += 1
