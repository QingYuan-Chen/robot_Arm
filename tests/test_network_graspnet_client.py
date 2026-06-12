from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def test_network_graspnet_client_fetches_json_payload():
    from rebotarm_vision.network_graspnet_client import NetworkGraspNetClient, NetworkGraspNetConfig

    payload = {
        "frame_id": "camera_depth_frame",
        "source": "windows_graspnet_baseline",
        "backend_configured": True,
        "candidates": [],
    }

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format, *args):
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        client = NetworkGraspNetClient(
            NetworkGraspNetConfig(
                candidates_url=f"http://127.0.0.1:{server.server_port}/graspnet_candidates.json",
                timeout_ms=1000,
            )
        )

        result = client.fetch()

        assert result["source"] == "windows_graspnet_baseline"
        assert client.last_debug_message == "ok candidates=0"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
