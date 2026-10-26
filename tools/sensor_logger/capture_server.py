#!/usr/bin/env python3
"""Capture Sensor Logger HTTP-push payloads to disk (stdlib only).

Run on the Mac:   python3 tools/sensor_logger/capture_server.py
Phone push URL:   http://<mac-ip>:8000/data

Writes every raw POST body as one line to captures/payloads_<timestamp>.jsonl
and prints a short summary per batch (sensor names, counts, first sample).
Stop with Ctrl+C.
"""
import json, os, sys, time
from collections import Counter
from http.server import BaseHTTPRequestHandler, HTTPServer

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
os.makedirs("captures", exist_ok=True)
OUT = os.path.join("captures", time.strftime("payloads_%Y%m%d_%H%M%S.jsonl"))
state = {"n": 0, "printed_first": False, "t0": time.time()}


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        with open(OUT, "ab") as f:
            f.write(body.replace(b"\n", b" ") + b"\n")
        state["n"] += 1
        try:
            msg = json.loads(body)
            items = msg.get("payload", msg) if isinstance(msg, dict) else msg
            names = Counter(i.get("name", "?") for i in items if isinstance(i, dict))
            print(f"#{state['n']:4d} {len(body):7d} B  {dict(names)}")
            if not state["printed_first"]:
                state["printed_first"] = True
                short = json.dumps(msg, indent=2)
                print("---- first payload (truncated) ----")
                print(short[:2500])
                print("-----------------------------------")
        except Exception as e:
            print(f"#{state['n']} non-JSON or unexpected body ({e}); saved raw")
        self.send_response(200)
        self.end_headers()

    def log_message(self, *a):
        pass


print(f"Listening on 0.0.0.0:{PORT}  ->  saving to {OUT}")
print("Phone URL should be http://<this-mac-ip>:%d/data" % PORT)
try:
    HTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
except KeyboardInterrupt:
    print(f"\nDone: {state['n']} batches in {time.time()-state['t0']:.0f}s -> {OUT}")
