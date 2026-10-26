import json
import os
import time
import urllib.request

from sensor_logger_bridge.http_ingest import IngestServer
from sensor_logger_bridge.parser import ClockOffset, ImuAssembler, iter_samples, quat_normalized
from sensor_logger_bridge.replay import replay

DATA = os.path.join(os.path.dirname(__file__), 'data', 'sample.jsonl')


def load():
    return [json.loads(l) for l in open(DATA) if l.strip()]


def test_iter_samples_names_and_ns_ints():
    s = list(iter_samples(load()[0]))
    names = {n for n, _, _ in s}
    assert {'gyroscope', 'accelerometer', 'orientation', 'gravity'} <= names
    assert all(isinstance(t, int) and t > 1.7e18 for _, t, _ in s)


def test_imu_assembly_rate_and_monotonic():
    a, out = ImuAssembler(), []
    for m in load():
        out += a.feed(iter_samples(m))
    assert len(out) > 40
    ts = [o['t_ns'] for o in out]
    assert ts == sorted(ts)
    dts = [(b - a_) / 1e6 for a_, b in zip(ts, ts[1:])]
    assert 9.0 < sorted(dts)[len(dts) // 2] < 11.0       # 100 Hz
    assert a.dropped == 0


def test_tick_split_across_batches():
    a = ImuAssembler()
    t = 10**18
    assert a.feed([('gyroscope', t, {'x': 0}), ('accelerometer', t, {'x': 0})]) == []
    out = a.feed([('orientation', t, {'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1})])
    assert len(out) == 1


def test_quaternion_normalized():
    for m in load():
        for n, _, v in iter_samples(m):
            if n == 'orientation':
                q = quat_normalized(v)
                assert abs(sum(c * c for c in q) - 1) < 1e-9
                return


def test_clock_offset_ignores_backlog():
    c = ClockOffset()
    c.update(1_000_000_000 + 3_000_000_000, 1_000_000_000)   # delayed batch (3 s)
    c.update(2_000_000_000 + 50_000_000, 2_000_000_000)       # 50 ms
    c.update(3_000_000_000 + 80_000_000, 3_000_000_000)
    assert c.offset_ns == 50_000_000


def test_http_roundtrip_with_replay(tmp_path):
    rec = str(tmp_path / 'rec.jsonl')
    srv = IngestServer('127.0.0.1', 0, record_path=rec).start()
    try:
        replay(DATA, f'http://127.0.0.1:{srv.port}/data', speed=0)
        time.sleep(0.2)
        assert srv.received == len(load())
        assert srv.queue.qsize() == len(load())
        assert len(open(rec).read().splitlines()) == len(load())
    finally:
        srv.stop()


def test_auth_rejects_without_token():
    srv = IngestServer('127.0.0.1', 0, auth_token='secret').start()
    try:
        req = urllib.request.Request(f'http://127.0.0.1:{srv.port}/data', data=b'{}')
        try:
            urllib.request.urlopen(req)
            assert False
        except urllib.error.HTTPError as e:
            assert e.code == 401
    finally:
        srv.stop()
