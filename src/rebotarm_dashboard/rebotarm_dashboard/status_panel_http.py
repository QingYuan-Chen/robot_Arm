from __future__ import annotations

import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

from .status_panel_api import dispatch_post_request, is_allowed_post_path
from .status_panel_state import TeleopStatusStore, encode_sse_event
from .web_robot_assets import rewrite_package_mesh_uris, safe_mesh_path


def _write_response(handler: BaseHTTPRequestHandler, status: int, body: bytes, content_type: str) -> None:
    handler.send_response(status)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _json_bytes(payload: dict) -> bytes:
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def create_status_panel_server(
    *,
    host: str,
    port: int,
    node: object,
    store: TeleopStatusStore,
    html_page: str,
    urdf_path: Path,
    mesh_dir: Path,
    sse_interval_sec: float,
) -> ThreadingHTTPServer:
    class StatusPanelRequestHandler(BaseHTTPRequestHandler):
        def handle(self):  # noqa: D401
            try:
                super().handle()
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                return

        def do_GET(self):  # noqa: N802
            url = urlsplit(self.path)
            route = url.path
            if route == "/":
                _write_response(self, 200, html_page.encode("utf-8"), "text/html; charset=utf-8")
                return
            if route == "/api/status":
                _write_response(self, 200, _json_bytes(store.snapshot_dict()), "application/json")
                return
            if route == "/api/config":
                _write_response(self, 200, _json_bytes(node._panel_config()), "application/json")  # noqa: SLF001
                return
            if route == "/api/teach_record_info":
                query = parse_qs(url.query)
                record_path = query.get("path", query.get("record_path", [""]))[0]
                _write_response(
                    self,
                    200,
                    _json_bytes(node._teach_record_info(record_path or None)),  # noqa: SLF001
                    "application/json",
                )
                return
            if route == "/api/teach_records":
                _write_response(self, 200, _json_bytes(node._teach_records()), "application/json")  # noqa: SLF001
                return
            if route == "/api/teach_trajectory":
                query = parse_qs(url.query)
                record_path = query.get("path", [""])[0]
                max_points = int(query.get("max_points", ["500"])[0])
                _write_response(
                    self,
                    200,
                    _json_bytes(node._teach_trajectory(record_path or None, max_points=max_points)),  # noqa: SLF001
                    "application/json",
                )
                return
            if route == "/robot/urdf":
                body = rewrite_package_mesh_uris(urdf_path.read_text(encoding="utf-8")).encode("utf-8")
                _write_response(self, 200, body, "application/xml; charset=utf-8")
                return
            if route.startswith("/robot/meshes/"):
                name = unquote(route.removeprefix("/robot/meshes/"))
                path = safe_mesh_path(mesh_dir, name)
                if path is None:
                    self.send_response(404)
                    self.end_headers()
                    return
                _write_response(self, 200, path.read_bytes(), "model/stl")
                return
            if route == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(encode_sse_event(store.snapshot_dict()).encode("utf-8"))
                        self.wfile.flush()
                        time.sleep(sse_interval_sec)
                except Exception:
                    return
            self.send_response(404)
            self.end_headers()

        def do_POST(self):  # noqa: N802
            if not is_allowed_post_path(self.path):
                self.send_response(404)
                self.end_headers()
                return
            try:
                def read_payload() -> dict:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw = self.rfile.read(length) if length > 0 else b"{}"
                    payload = json.loads(raw.decode("utf-8"))
                    if not isinstance(payload, dict):
                        raise ValueError("payload must be an object")
                    return payload

                result = dispatch_post_request(node, self.path, read_payload)
                status = 200 if result.get("accepted") else 400
            except Exception as exc:
                result = {"accepted": False, "message": f"invalid web execute request: {exc}"}
                status = 400
            _write_response(self, status, _json_bytes(result), "application/json")

        def log_message(self, *_args):
            return

    return ThreadingHTTPServer((host, port), StatusPanelRequestHandler)
