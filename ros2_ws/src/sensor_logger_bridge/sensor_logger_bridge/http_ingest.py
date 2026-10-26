"""Tiny threaded HTTP receiver. No ROS dependency, so it can be tested anywhere."""
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class IngestServer:
    def __init__(self, host='0.0.0.0', port=8000, auth_token='', record_path='', max_queue=2000):
        self.queue = queue.Queue(maxsize=max_queue)
        self.auth_token = auth_token
        self.record_path = record_path
        self.rejected = 0
        self.overflow = 0
        self.received = 0
        self._rec_lock = threading.Lock()
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                recv_ns = time.time_ns()
                n = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(n)
                if outer.auth_token and self.headers.get('Authorization', '') != outer.auth_token:
                    outer.rejected += 1
                    self.send_response(401)
                    self.end_headers()
                    return
                outer.received += 1
                if outer.record_path:
                    with outer._rec_lock, open(outer.record_path, 'ab') as f:
                        f.write(body.replace(b'\n', b' ') + b'\n')
                try:
                    outer.queue.put_nowait((recv_ns, body))
                except queue.Full:
                    outer.overflow += 1
                self.send_response(200)
                self.end_headers()

            def log_message(self, *a):
                pass

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.port = self.httpd.server_address[1]
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
