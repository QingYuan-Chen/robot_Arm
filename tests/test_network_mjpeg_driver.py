from __future__ import annotations

import contextlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread

import cv2
import numpy as np


def _jpeg_bytes(image_bgr: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".jpg", image_bgr)
    assert ok
    return encoded.tobytes()


@contextlib.contextmanager
def _snapshot_server(payload: bytes):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != "/snapshot.jpg":
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, _format, *args):
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}/snapshot.jpg"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_network_mjpeg_driver_reads_snapshot_as_bgr_frame():
    from rebotarm_vision.camera.network_mjpeg_driver import NetworkMjpegConfig, NetworkMjpegDriver

    image = np.zeros((24, 32, 3), dtype=np.uint8)
    image[:, :, 0] = 20
    image[:, :, 1] = 80
    image[:, :, 2] = 200

    with _snapshot_server(_jpeg_bytes(image)) as snapshot_url:
        driver = NetworkMjpegDriver(
            NetworkMjpegConfig(
                snapshot_url=snapshot_url,
                stream_url="",
                frame_timeout_ms=1000,
            )
        )
        driver.open()
        assert driver.warmup(1)

        color_bgr, depth_mm = driver.get_frame()

    assert depth_mm is None
    assert color_bgr.shape == image.shape
    assert color_bgr.dtype == np.uint8
    assert color_bgr.std() > 0
    np.testing.assert_allclose(color_bgr.mean(axis=(0, 1)), image.mean(axis=(0, 1)), atol=6)
