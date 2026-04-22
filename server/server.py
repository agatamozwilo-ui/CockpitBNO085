import json
import re
import socket
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

HOST = "0.0.0.0"
HTTP_PORT = 8000
UDP_PORT = 5005

latest = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "source": "none",
    "updated_ms": 0,
}
lock = threading.Lock()

line_re = re.compile(
    r"ROLL\s*[:=]\s*([-+]?\d*\.?\d+).+?PITCH\s*[:=]\s*([-+]?\d*\.?\d+).+?YAW\s*[:=]\s*([-+]?\d*\.?\d+)",
    re.IGNORECASE,
)


def update_latest_from_text(text, addr):
    m = line_re.search(text)
    if not m:
        return False

    roll = float(m.group(1))
    pitch = float(m.group(2))
    yaw = float(m.group(3))

    with lock:
        latest["roll"] = roll
        latest["pitch"] = pitch
        latest["yaw"] = yaw
        latest["source"] = f"{addr[0]}:{addr[1]}"
        latest["updated_ms"] = int(time.time() * 1000)

    return True


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(Path(__file__).parent), **kwargs)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/latest":
            with lock:
                data = dict(latest)
            blob = json.dumps(data).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(blob)))
            self.end_headers()
            self.wfile.write(blob)
            return

        if parsed.path == "/":
            self.path = "/index.html"

        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/ingest":
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length) if length > 0 else b""
            text = body.decode("utf-8", errors="ignore").strip()
            ok = update_latest_from_text(text, self.client_address)
            self.send_response(200 if ok else 400)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"OK")
            return

        self.send_response(404)
        self.end_headers()


def handle_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, UDP_PORT))
    print(f"UDP listening on {HOST}:{UDP_PORT}")

    while True:
        payload, addr = sock.recvfrom(1024)
        text = payload.decode("utf-8", errors="ignore").strip()

        update_latest_from_text(text, addr)



def main():
    t = threading.Thread(target=handle_udp, daemon=True)
    t.start()

    server = ThreadingHTTPServer((HOST, HTTP_PORT), Handler)
    print(f"HTTP server running: http://localhost:{HTTP_PORT}")
    print("Open this URL on any device in your LAN (replace localhost with this PC's IP)")
    server.serve_forever()


if __name__ == "__main__":
    main()
