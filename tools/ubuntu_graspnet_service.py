from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib
import json
import os
from pathlib import Path
import sys
import time
from typing import Any

from rebotarm_vision.graspnet_service_contract import (
    CONTRACT_VERSION,
    ContractError,
    build_inference_response,
    decode_inference_request,
)


class StaleInputError(RuntimeError):
    def __init__(self, request, detail: str) -> None:
        super().__init__(detail)
        self.request = request


class GraspNetService:
    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str,
        device: str,
        backend_module: str,
        max_input_age_ms: int = 1000,
    ) -> None:
        self.model_root = str(model_root).strip()
        self.checkpoint_path = str(checkpoint_path).strip()
        self.device = str(device).strip()
        self.backend_module = str(backend_module).strip()
        self.max_input_age_ms = max(0, int(max_input_age_ms))
        self.backend = None
        self.backend_error = "model_root_or_checkpoint_unset"
        if self.model_root and self.checkpoint_path:
            try:
                if not Path(self.model_root).is_dir():
                    raise FileNotFoundError(f"model root not found: {self.model_root}")
                if not Path(self.checkpoint_path).is_file():
                    raise FileNotFoundError(f"checkpoint not found: {self.checkpoint_path}")
                module = importlib.import_module(self.backend_module)
                self.backend = module.GraspNetBaselineInference(
                    model_root=self.model_root,
                    checkpoint_path=self.checkpoint_path,
                    device=self.device,
                )
                self.backend_error = ""
            except Exception as exc:
                self.backend_error = f"{type(exc).__name__}: {exc}"

    @property
    def configured(self) -> bool:
        return self.backend is not None

    def health(self) -> dict[str, Any]:
        return {
            "status": "ready" if self.configured else "unconfigured",
            "contract_version": CONTRACT_VERSION,
            "backend_configured": self.configured,
            "backend_error": self.backend_error,
            "device": self.device,
            "max_input_age_ms": self.max_input_age_ms,
        }

    def infer(self, payload: Any) -> dict[str, Any]:
        request = decode_inference_request(payload)
        if self.max_input_age_ms > 0:
            age_ns = time.time_ns() - request.sent_at_unix_ns
            limit_ns = self.max_input_age_ms * 1_000_000
            if age_ns > limit_ns:
                raise StaleInputError(request, f"input age {age_ns / 1e6:.1f} ms exceeds limit")
            if age_ns < -limit_ns:
                raise StaleInputError(request, f"input timestamp is {-age_ns / 1e6:.1f} ms in the future")
        if self.backend is None:
            raise RuntimeError("GraspNet backend is not configured")
        candidates = self.backend.infer(
            color_bgr=request.color_bgr,
            depth_mm=request.depth_m,
            detections=[request.detection],
            camera_info=request.camera_info,
            max_grasps=request.max_grasps,
        )
        return build_inference_response(request, list(candidates))


def make_handler(service: GraspNetService):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path != "/health":
                self._json(404, {"error": "not_found"})
                return
            self._json(200, service.health())

        def do_POST(self) -> None:
            if self.path != "/infer":
                self._json(404, {"error": "not_found"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 64 * 1024 * 1024:
                    raise ContractError("request body size is invalid")
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                result = service.infer(payload)
            except ContractError as exc:
                self._json(400, {"error": "invalid_contract", "detail": str(exc)})
                return
            except json.JSONDecodeError as exc:
                self._json(400, {"error": "invalid_json", "detail": str(exc)})
                return
            except StaleInputError as exc:
                self._json(
                    409,
                    {
                        "contract_version": CONTRACT_VERSION,
                        "source": "ubuntu_local_graspnet",
                        "backend_configured": service.configured,
                        "stale": True,
                        "timestamp_ns": exc.request.timestamp_ns,
                        "frame_id": exc.request.frame_id,
                        "candidates": [],
                        "error": "stale_input",
                        "detail": str(exc),
                    },
                )
                return
            except RuntimeError as exc:
                self._json(503, {"error": "backend_unavailable", "detail": str(exc), **service.health()})
                return
            except Exception as exc:
                self._json(500, {"error": "inference_failed", "detail": f"{type(exc).__name__}: {exc}"})
                return
            self._json(200, result)

        def log_message(self, format: str, *args) -> None:
            print(f"graspnet_service {self.address_string()} {format % args}", flush=True)

        def _json(self, status: int, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Local Ubuntu GraspNet inference service")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--model-root", default=os.environ.get("GRASPNET_MODEL_ROOT", ""))
    parser.add_argument("--checkpoint-path", default=os.environ.get("GRASPNET_CHECKPOINT_PATH", ""))
    parser.add_argument("--device", default=os.environ.get("GRASPNET_DEVICE", "cuda:0"))
    parser.add_argument("--backend-module", default="graspnet_baseline_inference")
    parser.add_argument("--max-input-age-ms", type=int, default=1000)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    service = GraspNetService(
        model_root=args.model_root,
        checkpoint_path=args.checkpoint_path,
        device=args.device,
        backend_module=args.backend_module,
        max_input_age_ms=args.max_input_age_ms,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    print(json.dumps({"listen": f"http://{args.host}:{args.port}", **service.health()}), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
