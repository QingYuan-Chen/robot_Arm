from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
    import numpy as np
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: opencv-python/numpy. Install them with: python -m pip install opencv-python numpy"
    ) from exc


class CameraState:
    def __init__(
        self,
        camera_index: int,
        width: int,
        height: int,
        fps: int,
        jpeg_quality: int,
        model_path: str,
        yolo_device: str,
        conf_threshold: float,
        iou_threshold: float,
        yolo_classes: list[str],
        allowed_classes: list[str],
        detection_fps: float,
        backend: str,
        capture_source: str,
        depth_width: int,
        depth_height: int,
        depth_fps: int,
        depth_downsample_filter: int,
        graspnet_candidates_path: str,
    ):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.model_path = model_path
        self.yolo_device = yolo_device
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.yolo_classes = yolo_classes
        self.allowed_classes = {name.strip().lower() for name in allowed_classes if name.strip()}
        self.detection_period = 1.0 / max(detection_fps, 0.1)
        self.backend = backend
        self.capture_source = capture_source
        self.depth_width = depth_width
        self.depth_height = depth_height
        self.depth_fps = depth_fps
        self.depth_downsample_filter = max(1, int(depth_downsample_filter))
        self.graspnet_candidates_path = graspnet_candidates_path
        self.lock = threading.Lock()
        self.latest_jpeg: bytes | None = None
        self.latest_annotated_jpeg: bytes | None = None
        self.latest_depth_png: bytes | None = None
        self.latest_camera_info: dict = {
            "color_frame_id": "windows_gemini_color_frame",
            "depth_frame_id": "camera_depth_frame",
            "color_width": width,
            "color_height": height,
            "color_fps": fps,
            "color_format": "requested",
            "depth_width": depth_width,
            "depth_height": depth_height,
            "depth_fps": depth_fps,
            "depth_format": "requested",
            "depth_downsample_filter": self.depth_downsample_filter,
            "fx": 0.0,
            "fy": 0.0,
            "cx": 0.0,
            "cy": 0.0,
            "depth_scale_m": 0.001,
            "source": capture_source,
            "timestamp": 0.0,
        }
        self.latest_detections: dict = {
            "frame_id": "windows_gemini_color_frame",
            "image_width": width,
            "image_height": height,
            "detections": [],
            "source": "windows_yolo",
            "model": model_path,
            "timestamp": 0.0,
        }
        self.latest_graspnet_candidates: dict = {
            "frame_id": "camera_depth_frame",
            "source": "windows_graspnet_baseline",
            "backend_configured": False,
            "candidates": [],
            "timestamp": 0.0,
        }
        self.latest_error = "starting"
        self.detector_error = ""
        self.camera_debug = "not_opened"
        self.running = True

    def current_graspnet_candidates(self) -> dict:
        if self.graspnet_candidates_path:
            try:
                with open(self.graspnet_candidates_path, "r", encoding="utf-8") as handle:
                    payload = json.load(handle)
                if isinstance(payload, dict):
                    with self.lock:
                        self.latest_graspnet_candidates = payload
                    return payload
            except Exception as exc:
                return {
                    "frame_id": "camera_depth_frame",
                    "source": "windows_graspnet_baseline",
                    "backend_configured": False,
                    "candidates": [],
                    "error": f"{type(exc).__name__}: {exc}",
                    "timestamp": time.time(),
                }
        with self.lock:
            return dict(self.latest_graspnet_candidates)

    def run(self) -> None:
        model = self._load_model()
        if self.capture_source == "orbbec":
            self._run_orbbec(model)
            return
        self._run_opencv(model)

    def _run_opencv(self, model) -> None:
        capture = self._open_capture()
        if not capture.isOpened():
            self.latest_error = f"failed to open camera index {self.camera_index}"
            return

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        period = 1.0 / max(self.fps, 1)
        last_detection_at = 0.0

        try:
            black_frame_count = 0
            while self.running:
                ok, frame = capture.read()
                if not ok or frame is None:
                    self.latest_error = "camera read failed"
                    capture.release()
                    time.sleep(0.5)
                    capture = self._open_capture()
                    black_frame_count = 0
                    continue
                if float(frame.std()) < 1.0:
                    black_frame_count += 1
                    self.latest_error = f"camera frame looks blank, std={float(frame.std()):.2f}"
                    if black_frame_count >= 5:
                        capture.release()
                        time.sleep(0.5)
                        capture = self._open_capture()
                        black_frame_count = 0
                    time.sleep(0.1)
                    continue
                black_frame_count = 0
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if not ok:
                    self.latest_error = "jpeg encode failed"
                    continue
                with self.lock:
                    self.latest_jpeg = encoded.tobytes()
                    self.latest_error = ""
                now = time.monotonic()
                if model is not None and now - last_detection_at >= self.detection_period:
                    self._run_detection(model, frame)
                    last_detection_at = now
                time.sleep(period)
        finally:
            capture.release()

    def _run_orbbec(self, model) -> None:
        try:
            from pyorbbecsdk import Config, OBAlignMode, OBFormat, OBSensorType, Pipeline
        except Exception as exc:
            self.latest_error = f"pyorbbecsdk unavailable: {type(exc).__name__}: {exc}"
            self.camera_debug = "orbbec_import_failed"
            return

        pipeline = None
        try:
            pipeline = Pipeline()
            config = Config()

            color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            color_profile = None
            for fmt in (OBFormat.MJPG, OBFormat.RGB):
                try:
                    color_profile = color_profiles.get_video_stream_profile(
                        self.width,
                        self.height,
                        fmt,
                        self.fps,
                    )
                    break
                except Exception:
                    pass
            if color_profile is None:
                color_profile = color_profiles.get_default_video_stream_profile()
            config.enable_stream(color_profile)
            self._record_video_profile("color", color_profile)

            depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = None
            if self.depth_width > 0 and self.depth_height > 0:
                for fmt in (OBFormat.Y16, OBFormat.Y14):
                    try:
                        depth_profile = depth_profiles.get_video_stream_profile(
                            self.depth_width,
                            self.depth_height,
                            fmt,
                            self.depth_fps,
                        )
                        break
                    except Exception:
                        pass
            if depth_profile is None:
                depth_profile = depth_profiles.get_default_video_stream_profile()
            config.enable_stream(depth_profile)
            self._record_video_profile("depth", depth_profile)
            config.set_align_mode(OBAlignMode.HW_MODE)
            pipeline.start(config)
            self.camera_debug = "orbbec opened=True align=HW_MODE"

            try:
                self._record_camera_param(pipeline.get_camera_param())
            except Exception as exc:
                self.detector_error = f"camera info unavailable: {type(exc).__name__}: {exc}"

            period = 1.0 / max(self.fps, 1)
            last_detection_at = 0.0
            while self.running:
                frames = pipeline.wait_for_frames(1000)
                if frames is None:
                    self.latest_error = "orbbec wait_for_frames timeout"
                    time.sleep(0.1)
                    continue

                color_bgr = self._orbbec_color_to_bgr(frames)
                depth_mm = self._orbbec_depth_to_mm(frames)
                if color_bgr is None and depth_mm is None:
                    self.latest_error = "orbbec empty frameset"
                    continue

                if color_bgr is not None:
                    self._store_color_frame(color_bgr)
                    now = time.monotonic()
                    if model is not None and now - last_detection_at >= self.detection_period:
                        self._run_detection(model, color_bgr)
                        last_detection_at = now
                if depth_mm is not None:
                    self._store_depth_frame(depth_mm)
                if color_bgr is not None or depth_mm is not None:
                    self.latest_error = ""
                time.sleep(period)
        except Exception as exc:
            self.latest_error = f"orbbec failed: {type(exc).__name__}: {exc}"
            self.camera_debug = "orbbec opened=False"
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass

    def _orbbec_color_to_bgr(self, frames):
        try:
            from pyorbbecsdk import OBFormat

            color_frame = frames.get_color_frame()
            if color_frame is None:
                return None
            width = color_frame.get_width()
            height = color_frame.get_height()
            raw = np.frombuffer(bytes(color_frame.get_data()), dtype=np.uint8)
            fmt = color_frame.get_format()
            with self.lock:
                self.latest_camera_info.update(
                    {
                        "actual_color_width": int(width),
                        "actual_color_height": int(height),
                        "actual_color_format": str(fmt),
                    }
                )
            if fmt == OBFormat.MJPG:
                return cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if fmt == OBFormat.RGB:
                return cv2.cvtColor(raw.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
            return raw.reshape(height, width, 3)
        except Exception as exc:
            self.latest_error = f"orbbec color decode failed: {type(exc).__name__}: {exc}"
            return None

    def _orbbec_depth_to_mm(self, frames):
        try:
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                return None
            width = depth_frame.get_width()
            height = depth_frame.get_height()
            depth = np.frombuffer(bytes(depth_frame.get_data()), dtype=np.uint16).reshape(height, width)
            with self.lock:
                self.latest_camera_info.update(
                    {
                        "actual_depth_width": int(width),
                        "actual_depth_height": int(height),
                        "actual_depth_format": str(depth_frame.get_format()),
                    }
                )
            if self.depth_downsample_filter > 1:
                step = self.depth_downsample_filter
                depth = depth[::step, ::step]
            return depth
        except Exception as exc:
            self.latest_error = f"orbbec depth decode failed: {type(exc).__name__}: {exc}"
            return None

    def _record_video_profile(self, prefix: str, profile) -> None:
        try:
            with self.lock:
                self.latest_camera_info.update(
                    {
                        f"{prefix}_width": int(profile.get_width()),
                        f"{prefix}_height": int(profile.get_height()),
                        f"{prefix}_fps": int(profile.get_fps()),
                        f"{prefix}_format": str(profile.get_format()),
                    }
                )
        except Exception as exc:
            with self.lock:
                self.latest_camera_info[f"{prefix}_profile_error"] = f"{type(exc).__name__}: {exc}"

    def _record_camera_param(self, camera_param) -> None:
        color_intrinsic = self._intrinsic_to_dict(getattr(camera_param, "rgb_intrinsic", None))
        depth_intrinsic = self._intrinsic_to_dict(getattr(camera_param, "depth_intrinsic", None))
        color_distortion = self._distortion_to_dict(getattr(camera_param, "rgb_distortion", None))
        depth_distortion = self._distortion_to_dict(getattr(camera_param, "depth_distortion", None))
        depth_to_color = self._transform_to_dict(getattr(camera_param, "transform", None))

        update = {
            "sdk_camera_param_available": True,
            "color_intrinsic": color_intrinsic,
            "depth_intrinsic": depth_intrinsic,
            "color_distortion": color_distortion,
            "depth_distortion": depth_distortion,
            "depth_to_color": depth_to_color,
            "timestamp": time.time(),
        }
        if color_intrinsic:
            update.update(
                {
                    "fx": color_intrinsic.get("fx", 0.0),
                    "fy": color_intrinsic.get("fy", 0.0),
                    "cx": color_intrinsic.get("cx", 0.0),
                    "cy": color_intrinsic.get("cy", 0.0),
                    "color_width": int(color_intrinsic.get("width", self.width) or self.width),
                    "color_height": int(color_intrinsic.get("height", self.height) or self.height),
                }
            )
        if depth_intrinsic:
            update.update(
                {
                    "depth_fx": depth_intrinsic.get("fx", 0.0),
                    "depth_fy": depth_intrinsic.get("fy", 0.0),
                    "depth_cx": depth_intrinsic.get("cx", 0.0),
                    "depth_cy": depth_intrinsic.get("cy", 0.0),
                    "depth_intrinsic_width": int(depth_intrinsic.get("width", 0) or 0),
                    "depth_intrinsic_height": int(depth_intrinsic.get("height", 0) or 0),
                }
            )
        with self.lock:
            self.latest_camera_info.update(update)

    @staticmethod
    def _intrinsic_to_dict(intrinsic) -> dict:
        if intrinsic is None:
            return {}
        result = {}
        for key in ("fx", "fy", "cx", "cy", "width", "height"):
            if hasattr(intrinsic, key):
                value = getattr(intrinsic, key)
                result[key] = int(value) if key in ("width", "height") else float(value)
        return result

    @staticmethod
    def _distortion_to_dict(distortion) -> dict:
        if distortion is None:
            return {}
        result = {}
        for key in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2"):
            if hasattr(distortion, key):
                result[key] = float(getattr(distortion, key))
        return result

    @staticmethod
    def _transform_to_dict(transform) -> dict:
        if transform is None:
            return {}

        def values_from_attr(*names):
            for name in names:
                if hasattr(transform, name):
                    value = getattr(transform, name)
                    try:
                        import numpy as _np

                        array = _np.asarray(value, dtype=float).reshape(-1)
                        return [float(item) for item in array]
                    except Exception:
                        try:
                            return [float(item) for item in value]
                        except TypeError:
                            return [float(value)]
            return []

        rotation = values_from_attr("rot", "rotation", "r")
        translation = values_from_attr("trans", "translation", "t")
        return {
            "rotation": rotation,
            "translation": translation,
        }

    def _store_color_frame(self, frame) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.latest_error = "jpeg encode failed"
            return
        with self.lock:
            self.latest_jpeg = encoded.tobytes()

    def _store_depth_frame(self, depth_mm) -> None:
        depth_mm = np.ascontiguousarray(depth_mm.astype(np.uint16, copy=False))
        ok, encoded = cv2.imencode(".png", depth_mm)
        if not ok:
            self.latest_error = "depth png encode failed"
            return
        with self.lock:
            self.latest_depth_png = encoded.tobytes()
            self.latest_camera_info.update(
                {
                    "depth_width": int(depth_mm.shape[1]),
                    "depth_height": int(depth_mm.shape[0]),
                    "depth_nonzero": int(np.count_nonzero(depth_mm)),
                    "timestamp": time.time(),
                }
            )

    def _open_capture(self):
        backend_map = {
            "dshow": cv2.CAP_DSHOW,
            "msmf": cv2.CAP_MSMF,
            "any": cv2.CAP_ANY,
        }
        backend_id = backend_map.get(self.backend.lower())
        if backend_id is None:
            raise RuntimeError(f"unsupported backend: {self.backend}")
        capture = cv2.VideoCapture(self.camera_index, backend_id)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.camera_debug = f"backend={self.backend} opened={capture.isOpened()}"
        return capture

    def _load_model(self):
        if not self.model_path:
            self.detector_error = "yolo disabled"
            return None
        try:
            from ultralytics import YOLO

            model = YOLO(self.model_path)
            if self.yolo_classes and ("world" in self.model_path.lower() or "yoloe" in self.model_path.lower()):
                model.set_classes(self.yolo_classes)
            self.detector_error = ""
            return model
        except Exception as exc:
            self.detector_error = f"failed to load YOLO: {type(exc).__name__}: {exc}"
            return None

    def _run_detection(self, model, frame) -> None:
        try:
            results = model.predict(
                frame,
                verbose=False,
                device=self.yolo_device,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
            )
            detections = self._results_to_json(results)
            annotated = frame.copy()
            for det in detections:
                x1, y1, x2, y2 = [int(round(v)) for v in det["bbox_xyxy"]]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{det["class_name"]} {det["confidence"]:.2f}'
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(y1 - 8, 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            ok, encoded = cv2.imencode(
                ".jpg",
                annotated,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            with self.lock:
                self.latest_detections = {
                    "frame_id": "windows_gemini_color_frame",
                    "image_width": int(frame.shape[1]),
                    "image_height": int(frame.shape[0]),
                    "detections": detections,
                    "source": "windows_yolo",
                    "model": self.model_path,
                    "timestamp": time.time(),
                }
                if ok:
                    self.latest_annotated_jpeg = encoded.tobytes()
        except Exception as exc:
            self.detector_error = f"detection failed: {type(exc).__name__}: {exc}"

    def _results_to_json(self, results) -> list[dict]:
        detections = []
        for result in results:
            names = getattr(result, "names", {})
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for index in range(len(boxes)):
                box = boxes[index]
                xyxy = box.xyxy[0].detach().cpu().numpy().reshape(-1)
                cls_id = int(box.cls[0].detach().cpu().numpy().reshape(-1)[0])
                conf = float(box.conf[0].detach().cpu().numpy().reshape(-1)[0])
                class_name = str(names.get(cls_id, cls_id))
                if self.allowed_classes and class_name.lower() not in self.allowed_classes:
                    continue
                detections.append(
                    {
                        "class_name": class_name,
                        "confidence": conf,
                        "bbox_xyxy": [float(v) for v in xyxy[:4]],
                        "obb": self._obb_to_json(result, index),
                        "mask": self._mask_to_json(result, index),
                    }
                )
        return detections

    def _obb_to_json(self, result, index: int) -> dict | None:
        obb = getattr(result, "obb", None)
        if obb is None:
            return None
        try:
            xywhr = obb.xywhr[index].detach().cpu().numpy().reshape(-1)
            points = obb.xyxyxyxy[index].detach().cpu().numpy()
        except Exception:
            return None
        if xywhr.size < 5:
            return None
        points = np.asarray(points, dtype=np.float32)
        if points.ndim == 3 and points.shape[0] == 1:
            points = points[0]
        if points.ndim == 1 and points.size == 8:
            points = points.reshape(4, 2)
        if points.shape != (4, 2):
            points_xy = []
        else:
            points_xy = [float(value) for value in points.reshape(-1)]
        return {
            "cx": float(xywhr[0]),
            "cy": float(xywhr[1]),
            "w": float(xywhr[2]),
            "h": float(xywhr[3]),
            "theta": float(xywhr[4]),
            "points_xy": points_xy,
        }

    def _mask_to_json(self, result, index: int) -> dict | None:
        masks = getattr(result, "masks", None)
        if masks is None:
            return None
        polygons = getattr(masks, "xy", None)
        if polygons is None or index >= len(polygons):
            return None
        polygon = np.asarray(polygons[index], dtype=np.float32)
        if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2:
            return None
        return {"polygon_xy": [float(value) for value in polygon.reshape(-1)]}


def make_handler(state: CameraState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/health"):
                if state.latest_jpeg:
                    detail = "ok"
                else:
                    detail = state.latest_error
                if state.detector_error:
                    detail = f"{detail}; detector={state.detector_error}"
                detail = f"{detail}; camera={state.camera_debug}"
                if state.latest_depth_png:
                    detail = f"{detail}; depth=ok"
                self._send_text(detail)
                return
            if self.path == "/snapshot.jpg":
                self._send_snapshot()
                return
            if self.path == "/video.mjpg":
                self._send_stream(annotated=False)
                return
            if self.path == "/annotated.mjpg":
                self._send_stream(annotated=True)
                return
            if self.path == "/detections.json":
                self._send_detections()
                return
            if self.path == "/depth.png":
                self._send_depth()
                return
            if self.path == "/camera_info.json":
                self._send_camera_info()
                return
            if self.path == "/graspnet_candidates.json":
                self._send_graspnet_candidates()
                return
            self.send_response(404)
            self.end_headers()

        def _send_text(self, text: str) -> None:
            payload = text.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _current_jpeg(self) -> bytes | None:
            with state.lock:
                return state.latest_jpeg

        def _send_snapshot(self) -> None:
            payload = self._current_jpeg()
            if payload is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _current_annotated_jpeg(self) -> bytes | None:
            with state.lock:
                return state.latest_annotated_jpeg or state.latest_jpeg

        def _current_detections(self) -> dict:
            with state.lock:
                return dict(state.latest_detections)

        def _current_depth_png(self) -> bytes | None:
            with state.lock:
                return state.latest_depth_png

        def _current_camera_info(self) -> dict:
            with state.lock:
                return dict(state.latest_camera_info)

        def _send_detections(self) -> None:
            payload = json.dumps(self._current_detections()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _send_depth(self) -> None:
            payload = self._current_depth_png()
            if payload is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _send_camera_info(self) -> None:
            payload = json.dumps(self._current_camera_info()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _send_graspnet_candidates(self) -> None:
            payload = json.dumps(state.current_graspnet_candidates()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _send_stream(self, annotated: bool) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while state.running:
                payload = self._current_annotated_jpeg() if annotated else self._current_jpeg()
                if payload is None:
                    time.sleep(0.1)
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break
                time.sleep(1.0 / max(state.fps, 1))

        def log_message(self, _format, *args):
            return

    return Handler


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve a Windows camera as MJPEG for the ROS2 VM.")
    parser.add_argument("--camera-index", type=int, default=3)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--capture-source", choices=["opencv", "orbbec"], default="opencv")
    parser.add_argument("--backend", choices=["dshow", "msmf", "any"], default="dshow")
    parser.add_argument("--depth-width", type=int, default=1280)
    parser.add_argument("--depth-height", type=int, default=720)
    parser.add_argument("--depth-fps", type=int, default=30)
    parser.add_argument("--depth-downsample-filter", type=int, default=1)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--model-path", default=r"D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main\tools\yolo26s-seg.pt")
    parser.add_argument("--yolo-device", default="0")
    parser.add_argument("--conf-threshold", type=float, default=0.25)
    parser.add_argument("--iou-threshold", type=float, default=0.45)
    parser.add_argument("--detection-fps", type=float, default=15.0)
    parser.add_argument(
        "--classes",
        default="bottle",
        help="Comma-separated open-vocabulary classes for YOLOE/world models.",
    )
    parser.add_argument(
        "--allowed-classes",
        default="bottle",
        help="Comma-separated detection class names to publish. Empty string publishes all classes.",
    )
    parser.add_argument(
        "--graspnet-candidates-path",
        default="",
        help="Optional JSON file written by a Windows GraspNet process and served at /graspnet_candidates.json.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    classes = [item.strip() for item in args.classes.split(",") if item.strip()]
    allowed_classes = [item.strip() for item in args.allowed_classes.split(",") if item.strip()]
    state = CameraState(
        args.camera_index,
        args.width,
        args.height,
        args.fps,
        args.jpeg_quality,
        args.model_path,
        args.yolo_device,
        args.conf_threshold,
        args.iou_threshold,
        classes,
        allowed_classes,
        args.detection_fps,
        args.backend,
        args.capture_source,
        args.depth_width,
        args.depth_height,
        args.depth_fps,
        args.depth_downsample_filter,
        args.graspnet_candidates_path,
    )
    camera_thread = threading.Thread(target=state.run, daemon=True)
    camera_thread.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    print(f"Serving snapshots at http://{args.host}:{args.port}/snapshot.jpg")
    print(f"Serving MJPEG at    http://{args.host}:{args.port}/video.mjpg")
    print(f"Serving detections at http://{args.host}:{args.port}/detections.json")
    print(f"Serving annotated at  http://{args.host}:{args.port}/annotated.mjpg")
    print(f"Serving depth at      http://{args.host}:{args.port}/depth.png")
    print(f"Serving camera info at http://{args.host}:{args.port}/camera_info.json")
    print(f"Serving GraspNet at  http://{args.host}:{args.port}/graspnet_candidates.json")
    print("Use Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        server.shutdown()
        server.server_close()
        camera_thread.join(timeout=2)


if __name__ == "__main__":
    main()
