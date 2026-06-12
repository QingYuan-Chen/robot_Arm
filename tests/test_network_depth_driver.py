from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


def test_network_mjpeg_driver_reads_depth_png():
    from rebotarm_vision.camera.network_mjpeg_driver import (
        NetworkMjpegConfig,
        NetworkMjpegDriver,
    )

    depth_mm = np.array(
        [
            [0, 1000, 1001],
            [1200, 1300, 1400],
        ],
        dtype=np.uint16,
    )
    ok, encoded = cv2.imencode(".png", depth_mm)
    assert ok

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/depth.png":
                payload = encoded.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            self.send_response(404)
            self.end_headers()

        def log_message(self, _format, *args):
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        driver = NetworkMjpegDriver(
            NetworkMjpegConfig(
                snapshot_url="",
                stream_url="",
                frame_timeout_ms=1000,
                depth_url=f"http://127.0.0.1:{server.server_port}/depth.png",
            )
        )
        _color, received_depth = driver.get_frame()

        assert received_depth is not None
        assert received_depth.dtype == np.uint16
        assert received_depth.shape == (2, 3)
        assert received_depth.tolist() == depth_mm.tolist()
        assert "depth_ok" in driver.last_debug_message
    finally:
        server.shutdown()
        server.server_close()
