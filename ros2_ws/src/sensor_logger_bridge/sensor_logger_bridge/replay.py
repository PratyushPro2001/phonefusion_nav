"""Replay a captured .jsonl (one raw POST body per line) to the bridge, no phone needed.

    ros2 run sensor_logger_bridge replay_capture phonedata/iphone16e_http_push_sample.jsonl
    ros2 run sensor_logger_bridge replay_capture FILE --url http://localhost:8000/data --speed 1
"""
import argparse
import json
import time
import urllib.request


def replay(path, url, speed=1.0, loop=False):
    while True:
        prev = None
        with open(path, 'rb') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                msg = json.loads(line)
                ts = [i['time'] for i in msg.get('payload', []) if 'time' in i]
                newest = max(ts) / 1e9 if ts else None
                if prev is not None and newest is not None and speed > 0:
                    time.sleep(max(0.0, (newest - prev) / speed))
                prev = newest if newest is not None else prev
                req = urllib.request.Request(url, data=line, headers={'Content-Type': 'application/json'})
                urllib.request.urlopen(req, timeout=5).read()
        if not loop:
            break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('file')
    ap.add_argument('--url', default='http://localhost:8000/data')
    ap.add_argument('--speed', type=float, default=1.0, help='1=real time, 0=as fast as possible')
    ap.add_argument('--loop', action='store_true')
    a = ap.parse_args()
    replay(a.file, a.url, a.speed, a.loop)


if __name__ == '__main__':
    main()
