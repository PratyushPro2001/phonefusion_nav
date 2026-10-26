"""Pure-Python parsing of Sensor Logger HTTP-push payloads (no ROS imports).

Payload (verified on iPhone 16e, Sensor Logger 1.68):
    {"messageId": int, "sessionId": str, "deviceId": str,
     "payload": [ {"name": str, "time": <int ns since epoch, PHONE clock>,
                   "values": {...}, "accuracy": int (some sensors)}, ... ]}

Units seen in the real capture:
    accelerometer             x,y,z  m/s^2, gravity removed
    accelerometeruncalibrated x,y,z  g, gravity INCLUDED
    gyroscope / ...uncalibrated x,y,z rad/s
    gravity                   x,y,z  m/s^2 (derived from orientation, |g| = 9.80665 exactly)
    orientation               qx,qy,qz,qw + yaw,pitch,roll (rad)
    magnetometeruncalibrated  x,y,z  uT (includes hard-iron offset)
    magnetometer / compass    only {magneticBearing} (deg) -- no xyz
    location                  lat/lon deg, altitude (ellipsoidal) m, accuracies m, speed m/s
    barometer                 pressure hPa, relativeAltitude m
Timestamps of gyro/accel/orientation within one 100 Hz tick are identical.
"""
from collections import deque
import json

IMU_KEYS = ('gyroscope', 'accelerometer', 'orientation')


def iter_samples(msg):
    """Yield (name, t_ns, values) for every sample in a decoded message."""
    items = msg.get('payload', []) if isinstance(msg, dict) else msg
    for it in items:
        if not isinstance(it, dict) or 'time' not in it or 'name' not in it:
            continue
        yield str(it['name']).lower(), int(it['time']), it.get('values', {})


def parse_body(body):
    """bytes/str -> decoded message dict (raises ValueError on bad JSON)."""
    return json.loads(body)


class ImuAssembler:
    """Join gyroscope + accelerometer + orientation samples that share a timestamp."""

    def __init__(self, max_pending_ns=500_000_000):
        self._pending = {}
        self._max_pending_ns = max_pending_ns
        self.dropped = 0

    def feed(self, samples):
        """samples: iterable of (name, t_ns, values). Returns list of complete IMU dicts, time-sorted."""
        newest = 0
        for name, t, v in samples:
            if name in IMU_KEYS:
                self._pending.setdefault(t, {})[name] = v
                newest = max(newest, t)
        out = []
        for t in sorted(self._pending):
            d = self._pending[t]
            if all(k in d for k in IMU_KEYS):
                out.append({'t_ns': t, 'gyro': d['gyroscope'],
                            'accel': d['accelerometer'], 'orient': d['orientation']})
                del self._pending[t]
        if newest:  # drop incomplete ticks that are too old to ever complete
            for t in [t for t in self._pending if newest - t > self._max_pending_ns]:
                del self._pending[t]
                self.dropped += 1
        return out


class ClockOffset:
    """Map phone timestamps onto the PC clock.

    offset = min over recent batches of (arrival_time - newest_sample_time). The minimum is the
    least-delayed batch, so network jitter is removed while the PC clock stays the reference.
    """

    def __init__(self, window=300):
        self._d = deque(maxlen=window)

    def update(self, recv_ns, newest_phone_ns):
        self._d.append(recv_ns - newest_phone_ns)

    @property
    def offset_ns(self):
        return min(self._d) if self._d else 0

    def to_pc_ns(self, phone_ns):
        return phone_ns + self.offset_ns


def quat_normalized(v):
    x, y, z, w = (float(v[k]) for k in ('qx', 'qy', 'qz', 'qw'))
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    return x / n, y / n, z / n, w / n
