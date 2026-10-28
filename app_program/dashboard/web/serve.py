"""Serve only the local dashboard files. No dependencies or firmware changes."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class DashboardHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


if __name__ == "__main__":
    root = Path(__file__).resolve().parent
    handler = partial(DashboardHandler, directory=str(root))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", 8765), handler)
    except OSError as error:
        raise SystemExit(f"Cannot start dashboard on port 8765: {error}. Check whether it is already running.")
    print("CSC11 dashboard: http://127.0.0.1:8765/", flush=True)
    print("Keep this terminal open. Ctrl+C stops the webpage server.", flush=True)
    print("Broker WebSocket address: ws://127.0.0.1:9001/mqtt", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
