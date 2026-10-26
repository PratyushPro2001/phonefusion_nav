"""Run the EKF on a recorded .jsonl capture (no ROS, no phone) and save the trajectory plot + CSV.

    python3 -m phone_ekf.run_offline capture.jsonl [out_prefix]
Backup plan for a demo: shows the same filter on a recording.
"""
import csv
import json
import sys

import numpy as np

from phone_ekf.ekf import EkfConfig, PhoneEKF, quat_to_R


def load(path):
    from sensor_logger_bridge.parser import ImuAssembler, iter_samples
    asm, ticks, comp, baro = ImuAssembler(), [], {}, []
    for line in open(path):
        if not line.strip():
            continue
        samples = list(iter_samples(json.loads(line)))
        ticks += asm.feed(samples)
        for n, t, v in samples:
            if n == 'compass' and 'magneticBearing' in v:
                comp[t] = v['magneticBearing']
            elif n == 'barometer':
                baro.append((t, v['pressure'] * 100.0))
    return ticks, comp, sorted(baro)


def run(path, cfg=None):
    ticks, comp, baro = load(path)
    ekf = PhoneEKF(cfg or EkfConfig())
    t0, bi, rows = ticks[0]['t_ns'], 0, []
    for d in ticks:
        o = d['orient']
        ekf.imu((d['t_ns'] - t0) / 1e9, quat_to_R(o['qx'], o['qy'], o['qz'], o['qw']),
                [d['accel'][k] for k in 'xyz'], [d['gyro'][k] for k in 'xyz'])
        if d['t_ns'] in comp:
            ekf.compass(comp[d['t_ns']])
        while bi < len(baro) and baro[bi][0] <= d['t_ns']:
            ekf.baro(baro[bi][1]); bi += 1
        rows.append(((d['t_ns'] - t0) / 1e9, *ekf.p, *ekf.v, int(ekf.still), ekf.psi))
    return ekf, np.array(rows)


def main():
    path = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else path.rsplit('.', 1)[0] + '_ekf'
    ekf, rows = run(path)
    with open(out + '.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'still', 'psi'])
        w.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    ax[0].plot(rows[:, 1], rows[:, 2]); ax[0].plot(*rows[0, 1:3], 'go'); ax[0].plot(*rows[-1, 1:3], 'rx')
    ax[0].set_aspect('equal'); ax[0].set_title('top view (E, N) [m]'); ax[0].grid(True)
    ax[1].plot(rows[:, 0], rows[:, 3]); ax[1].set_title('height z [m]'); ax[1].grid(True)
    ax[2].plot(rows[:, 0], np.linalg.norm(rows[:, 4:7], axis=1)); ax[2].plot(rows[:, 0], rows[:, 7] * 0.2, alpha=.4)
    ax[2].set_title('speed [m/s] (flat band = still)'); ax[2].grid(True)
    fig.tight_layout(); fig.savefig(out + '.png', dpi=110)
    print('saved', out + '.csv', out + '.png', ' stats:', ekf.stats)


if __name__ == '__main__':
    main()
