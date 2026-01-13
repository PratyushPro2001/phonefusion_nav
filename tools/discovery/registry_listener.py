#!/usr/bin/env python3
import socket, json, time, os
from pathlib import Path

PORT = 37020                  # discovery port
REG_PATH = Path.home() / "phonefusion_nav" / "runtime" / "registry.json"

def main():
    REG_PATH.parent.mkdir(parents=True, exist_ok=True)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", PORT))

    print(f"[registry] listening on UDP :{PORT}")
    print(f"[registry] writing -> {REG_PATH}")

    while True:
        data, addr = s.recvfrom(4096)
        now = time.time()
        try:
            msg = json.loads(data.decode("utf-8", errors="ignore"))
        except Exception:
            continue

        # expected msg fields (defaults ok)
        rec = {
            "ts": now,
            "from_ip": addr[0],
            "device_name": msg.get("device_name", "phone"),
            "imu_udp_port": int(msg.get("imu_udp_port", 5555)),
            "ipcam_http_port": int(msg.get("ipcam_http_port", 4747)),
            "ipcam_path": msg.get("ipcam_path", "/video"),
            "notes": msg.get("notes", "")
        }

        REG_PATH.write_text(json.dumps(rec, indent=2))
        ipcam_url = f"http://{rec['from_ip']}:{rec['ipcam_http_port']}{rec['ipcam_path']}"
        print(f"[registry] updated: {rec['device_name']} @ {rec['from_ip']} | IMU:{rec['imu_udp_port']} | IPCAM:{ipcam_url}")

if __name__ == "__main__":
    main()
