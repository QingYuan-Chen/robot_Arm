from __future__ import annotations

import argparse
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any
from urllib.request import Request, urlopen

import cv2
import numpy as np

DEFAULT_WORKSPACE_MIN_DEPTH_M = 0.05
DEFAULT_WORKSPACE_MAX_DEPTH_M = 1.5


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run Windows-side GraspNet baseline inference and write candidates JSON for Ubuntu ROS2."
    )
    parser.add_argument("--server-url", default="http://127.0.0.1:8081")
    parser.add_argument("--output-path", default=r"D:\tmp\graspnet_candidates.json")
    parser.add_argument("--model-root", default="")
    parser.add_argument("--checkpoint-path", default="")
    parser.add_argument("--backend-module", default="graspnet_baseline_inference")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--max-grasps", type=int, default=20)
    parser.add_argument("--poll-hz", type=float, default=2.0)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--once", action="store_true")
    parser.add_argument(
        "--manual-trigger",
        action="store_true",
        help="Wait for 'y' + Enter before each inference. Use 'q' + Enter to quit.",
    )
    parser.add_argument("--open3d-visualize", action="store_true")
    parser.add_argument("--visualize-top-n", type=int, default=5)
    parser.add_argument("--visualize-max-points", type=int, default=30000)
    parser.add_argument("--visualize-every-n", type=int, default=10)
    parser.add_argument("--visualize-point-size", type=float, default=4.0)
    parser.add_argument("--visualize-axis-size", type=float, default=0.05)
    parser.add_argument("--visualize-zoom", type=float, default=0.28)
    parser.add_argument("--visualize-crop-radius-m", type=float, default=0.0)
    return parser


def atomic_write_json(path: str | Path, payload: dict[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=str(target.parent), delete=False) as handle:
        json.dump(payload, handle, ensure_ascii=False)
        tmp_name = handle.name
    try:
        last_error: PermissionError | None = None
        for attempt in range(20):
            try:
                os.replace(tmp_name, target)
                return
            except PermissionError as exc:
                last_error = exc
                time.sleep(min(0.05 * (attempt + 1), 0.5))
        if last_error is not None:
            raise last_error
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def write_backend_missing_payload(path: str | Path, *, reason: str) -> None:
    atomic_write_json(
        path,
        {
            "frame_id": "camera_depth_frame",
            "source": "windows_graspnet_baseline",
            "backend_configured": False,
            "candidates": [],
            "error": str(reason),
            "timestamp": time.time(),
        },
    )


def load_backend(*, model_root: str, checkpoint_path: str, backend_module: str, device: str):
    if model_root:
        root = str(Path(model_root))
        if root not in sys.path:
            sys.path.insert(0, root)
    module = importlib.import_module(backend_module)
    runner_cls = getattr(module, "GraspNetBaselineInference")
    return runner_cls(model_root=model_root, checkpoint_path=checkpoint_path, device=device)


def fetch_json(server_url: str, endpoint: str, timeout_ms: int) -> dict[str, Any]:
    url = f"{server_url.rstrip('/')}/{endpoint.lstrip('/')}"
    request = Request(url, headers={"User-Agent": "rebotarm_graspnet_bridge/0.1"})
    with urlopen(request, timeout=max(int(timeout_ms), 1) / 1000.0) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"{endpoint} returned non-object JSON")
    return payload


def fetch_image(server_url: str, endpoint: str, timeout_ms: int, flags: int) -> np.ndarray:
    url = f"{server_url.rstrip('/')}/{endpoint.lstrip('/')}"
    request = Request(url, headers={"User-Agent": "rebotarm_graspnet_bridge/0.1"})
    with urlopen(request, timeout=max(int(timeout_ms), 1) / 1000.0) as response:
        data = response.read()
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), flags)
    if image is None:
        raise RuntimeError(f"{endpoint} decode failed")
    return image


class Open3DGraspVisualizer:
    def __init__(
        self,
        *,
        top_n: int = 5,
        max_points: int = 30000,
        point_size: float = 4.0,
        axis_size: float = 0.05,
        zoom: float = 0.28,
        crop_radius_m: float = 0.0,
        window_name: str = "GraspNet candidates",
    ) -> None:
        self._o3d = importlib.import_module("open3d")
        self._top_n = max(0, int(top_n))
        self._max_points = max(1, int(max_points))
        self._point_size = max(1.0, float(point_size))
        self._axis_size = max(0.005, float(axis_size))
        self._zoom = min(max(float(zoom), 0.02), 2.0)
        self._crop_radius_m = max(0.0, float(crop_radius_m))
        self._window_name = window_name
        self._vis = None
        self._GraspGroup = self._load_grasp_group()

    def update(
        self,
        *,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
        camera_info: dict[str, Any],
        candidates: list[dict[str, Any]],
    ) -> None:
        if self._vis is None:
            self._vis = self._o3d.visualization.Visualizer()
            self._vis.create_window(window_name=self._window_name, width=1280, height=720)
            if hasattr(self._vis, "get_render_option"):
                render_options = self._vis.get_render_option()
                render_options.point_size = self._point_size
                render_options.background_color = np.asarray([0.02, 0.02, 0.02], dtype=np.float64)
        focus = self._candidate_focus(candidates)
        cloud = self._build_point_cloud(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            camera_info=camera_info,
            focus=focus,
        )
        selected_candidates = list(candidates)[: self._top_n]
        geometries = [cloud]
        gripper_geometries = self._build_official_gripper_geometries(selected_candidates)
        if gripper_geometries:
            geometries.extend(gripper_geometries)
        else:
            for index, candidate in enumerate(selected_candidates):
                geometries.append(self._build_gripper_lines(candidate))
                geometries.append(self._build_candidate_axes(candidate, index=index))
        self._vis.clear_geometries()
        for geometry in geometries:
            self._vis.add_geometry(geometry)
        self._fit_view(cloud, focus=focus)
        self._vis.poll_events()
        self._vis.update_renderer()

    def poll_events(self) -> None:
        if self._vis is None:
            return
        self._vis.poll_events()
        self._vis.update_renderer()

    def _load_grasp_group(self):
        try:
            return importlib.import_module("graspnetAPI").GraspGroup
        except Exception:
            return None

    def _build_official_gripper_geometries(self, candidates: list[dict[str, Any]]) -> list[Any]:
        if self._GraspGroup is None or not candidates:
            return []
        rows = []
        for candidate in candidates:
            row = self._candidate_to_graspnet_row(candidate)
            if row is not None:
                rows.append(row)
        if not rows:
            return []
        try:
            grasp_group = self._GraspGroup(np.asarray(rows, dtype=np.float64))
            return self._flatten_geometries(grasp_group.to_open3d_geometry_list())
        except Exception:
            return []

    @staticmethod
    def _candidate_to_graspnet_row(candidate: dict[str, Any]) -> np.ndarray | None:
        try:
            score = float(candidate.get("score", candidate.get("confidence", 0.0)) or 0.0)
            width = max(float(candidate.get("width_m", candidate.get("width", 0.04)) or 0.04), 0.001)
            height = max(float(candidate.get("height_m", candidate.get("height", 0.02)) or 0.02), 0.001)
            depth = max(float(candidate.get("depth_m", candidate.get("depth", 0.04)) or 0.04), 0.001)
            rotation = np.asarray(candidate.get("rotation_matrix", candidate.get("rotation")), dtype=np.float64).reshape(3, 3)
            translation = np.asarray(
                candidate.get("translation_xyz", candidate.get("translation")),
                dtype=np.float64,
            ).reshape(3)
            object_id = float(candidate.get("object_id", -1))
            return np.concatenate(
                [
                    np.asarray([score, width, height, depth], dtype=np.float64),
                    rotation.reshape(9),
                    translation,
                    np.asarray([object_id], dtype=np.float64),
                ]
            )
        except Exception:
            return None

    @staticmethod
    def _flatten_geometries(geometries) -> list[Any]:
        flattened = []
        for geometry in geometries:
            if isinstance(geometry, (list, tuple)):
                flattened.extend(geometry)
            else:
                flattened.append(geometry)
        return flattened

    def _build_point_cloud(
        self,
        *,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
        camera_info: dict[str, Any],
        focus: np.ndarray | None,
    ):
        depth = np.asarray(depth_mm, dtype=np.float32) * 0.001
        height, width = depth.shape[:2]
        fx = float(camera_info.get("fx", camera_info.get("depth_fx", 1.0)) or 1.0)
        fy = float(camera_info.get("fy", camera_info.get("depth_fy", 1.0)) or 1.0)
        cx = float(camera_info.get("cx", camera_info.get("depth_cx", width * 0.5)) or width * 0.5)
        cy = float(camera_info.get("cy", camera_info.get("depth_cy", height * 0.5)) or height * 0.5)
        min_depth_m = float(camera_info.get("workspace_min_depth_m", DEFAULT_WORKSPACE_MIN_DEPTH_M))
        max_depth_m = float(camera_info.get("workspace_max_depth_m", DEFAULT_WORKSPACE_MAX_DEPTH_M))
        valid_v, valid_u = np.nonzero(np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m))
        if len(valid_u) > self._max_points:
            indices = np.linspace(0, len(valid_u) - 1, self._max_points, dtype=np.int64)
            valid_u = valid_u[indices]
            valid_v = valid_v[indices]
        z = depth[valid_v, valid_u]
        x = (valid_u.astype(np.float32) - cx) * z / fx
        y = (valid_v.astype(np.float32) - cy) * z / fy
        points = np.column_stack((x, y, z)).astype(np.float64)
        colors = np.ones_like(points, dtype=np.float64) * 0.55
        color = np.asarray(color_bgr)
        if color.ndim == 3 and color.shape[:2] == depth.shape[:2]:
            colors = color[valid_v, valid_u, :3].astype(np.float64)[:, ::-1] / 255.0
        if focus is not None and self._crop_radius_m > 0.0 and len(points) > 0:
            distances = np.linalg.norm(points - focus.reshape(1, 3), axis=1)
            keep = distances <= self._crop_radius_m
            if int(np.count_nonzero(keep)) >= 50:
                points = points[keep]
                colors = colors[keep]
        cloud = self._o3d.geometry.PointCloud()
        cloud.points = self._o3d.utility.Vector3dVector(points)
        cloud.colors = self._o3d.utility.Vector3dVector(colors)
        return cloud

    @staticmethod
    def _candidate_focus(candidates: list[dict[str, Any]]) -> np.ndarray | None:
        translations = []
        for candidate in candidates[:10]:
            value = candidate.get("translation_xyz", candidate.get("translation"))
            if value is None:
                continue
            try:
                translations.append(np.asarray(value, dtype=np.float64).reshape(3))
            except Exception:
                continue
        if not translations:
            return None
        return np.mean(np.vstack(translations), axis=0)

    def _build_gripper_lines(self, candidate: dict[str, Any]):
        translation = np.asarray(candidate.get("translation_xyz", candidate.get("translation")), dtype=np.float64).reshape(3)
        rotation = np.asarray(candidate.get("rotation_matrix", candidate.get("rotation")), dtype=np.float64).reshape(3, 3)
        width = max(float(candidate.get("width_m", candidate.get("width", 0.04)) or 0.04), 0.01)
        depth = max(float(candidate.get("depth_m", candidate.get("depth", 0.04)) or 0.04), 0.01)
        score = float(candidate.get("score", candidate.get("confidence", 0.0)) or 0.0)
        half_width = width * 0.5
        # Local axes follow the common parallel-jaw convention: X approach, Y jaw opening.
        local = np.asarray(
            [
                [0.0, -half_width, 0.0],
                [0.0, half_width, 0.0],
                [depth, -half_width, 0.0],
                [depth, half_width, 0.0],
            ],
            dtype=np.float64,
        )
        points = local @ rotation.T + translation.reshape(1, 3)
        lines = np.asarray([[0, 1], [0, 2], [1, 3]], dtype=np.int32)
        color = np.asarray([[1.0 - min(score, 1.0), min(score, 1.0), 0.0] for _ in range(len(lines))])
        line_set = self._o3d.geometry.LineSet()
        line_set.points = self._o3d.utility.Vector3dVector(points)
        line_set.lines = self._o3d.utility.Vector2iVector(lines)
        line_set.colors = self._o3d.utility.Vector3dVector(color)
        return line_set

    def _build_candidate_axes(self, candidate: dict[str, Any], *, index: int):
        translation = np.asarray(candidate.get("translation_xyz", candidate.get("translation")), dtype=np.float64).reshape(3)
        rotation = np.asarray(candidate.get("rotation_matrix", candidate.get("rotation")), dtype=np.float64).reshape(3, 3)
        scale = self._axis_size * (1.0 if index == 0 else 0.75)
        points = np.vstack(
            [
                translation,
                translation + rotation[:, 0] * scale,
                translation + rotation[:, 1] * scale,
                translation + rotation[:, 2] * scale,
            ]
        )
        line_set = self._o3d.geometry.LineSet()
        line_set.points = self._o3d.utility.Vector3dVector(points)
        line_set.lines = self._o3d.utility.Vector2iVector(np.asarray([[0, 1], [0, 2], [0, 3]], dtype=np.int32))
        line_set.colors = self._o3d.utility.Vector3dVector(
            np.asarray(
                [
                    [1.0, 0.05, 0.05],  # local X: approach direction
                    [0.05, 1.0, 0.05],  # local Y: jaw opening direction
                    [0.10, 0.35, 1.0],  # local Z
                ],
                dtype=np.float64,
            )
        )
        return line_set

    def _fit_view(self, cloud, *, focus: np.ndarray | None) -> None:
        if self._vis is None:
            return
        if not hasattr(self._vis, "get_view_control"):
            return
        points = np.asarray(cloud.points)
        if points.size == 0:
            return
        center = points.mean(axis=0)
        view = self._vis.get_view_control()
        if view is None:
            return
        view.set_lookat(center.tolist())
        view.set_front([0.0, 0.0, -1.0])
        view.set_up([0.0, -1.0, 0.0])
        view.set_zoom(self._zoom)


def build_graspnet_payload(
    *,
    backend,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    detections_payload: dict[str, Any],
    camera_info: dict[str, Any],
    max_grasps: int,
) -> dict[str, Any]:
    detections = list(detections_payload.get("detections", []))
    if not detections:
        return {
            "frame_id": str(camera_info.get("depth_frame_id", "camera_depth_frame")),
            "source": "windows_graspnet_baseline",
            "backend_configured": True,
            "candidates": [],
            "reason": "no detections",
            "timestamp": time.time(),
        }
    raw = backend.infer(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detections=detections,
        camera_info=camera_info,
        max_grasps=max_grasps,
    )
    candidates = []
    for item in list(raw)[: max(0, int(max_grasps))]:
        if not isinstance(item, dict):
            continue
        candidate = dict(item)
        candidate.setdefault("source", "windows_graspnet_baseline")
        candidate.setdefault("class_name", str(detections[0].get("class_name", "")))
        candidates.append(candidate)
    return {
        "frame_id": str(camera_info.get("depth_frame_id", "camera_depth_frame")),
        "source": "windows_graspnet_baseline",
        "backend_configured": True,
        "candidates": candidates,
        "timestamp": time.time(),
    }


def resolve_visualize_every_n(args) -> int:
    if bool(getattr(args, "manual_trigger", False)):
        return 1
    return max(int(getattr(args, "visualize_every_n", 1) or 1), 1)


def run_once(args, backend, visualizer: Open3DGraspVisualizer | None = None, *, iteration: int = 1) -> dict[str, Any]:
    color_bgr = fetch_image(args.server_url, "/snapshot.jpg", args.timeout_ms, cv2.IMREAD_COLOR)
    depth_mm = fetch_image(args.server_url, "/depth.png", args.timeout_ms, cv2.IMREAD_UNCHANGED)
    if depth_mm.ndim == 3:
        depth_mm = depth_mm[:, :, 0]
    detections = fetch_json(args.server_url, "/detections.json", args.timeout_ms)
    camera_info = fetch_json(args.server_url, "/camera_info.json", args.timeout_ms)
    payload = build_graspnet_payload(
        backend=backend,
        color_bgr=color_bgr,
        depth_mm=depth_mm.astype(np.uint16, copy=False),
        detections_payload=detections,
        camera_info=camera_info,
        max_grasps=int(args.max_grasps),
    )
    atomic_write_json(args.output_path, payload)
    every_n = resolve_visualize_every_n(args)
    if visualizer is not None and int(iteration) % every_n == 0:
        visualizer.update(
            color_bgr=color_bgr,
            depth_mm=depth_mm.astype(np.uint16, copy=False),
            camera_info=camera_info,
            candidates=list(payload.get("candidates", [])),
        )
    return payload


def sleep_with_visualizer(period_sec: float, visualizer: Open3DGraspVisualizer | None) -> None:
    deadline = time.monotonic() + max(float(period_sec), 0.0)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        if visualizer is not None:
            visualizer.poll_events()
        time.sleep(min(remaining, 0.03 if visualizer is not None else remaining))


def wait_for_manual_trigger(
    *,
    input_func=input,
    print_func=print,
    prompt: str = "Press y + Enter to run one GraspNet inference, or q + Enter to quit: ",
) -> bool:
    while True:
        response = str(input_func(prompt)).strip().lower()
        if response in {"y", "yes"}:
            return True
        if response in {"q", "quit", "exit"}:
            print_func("manual trigger quit")
            return False
        print_func("skipped: enter y to infer, or q to quit")


def run_bridge_loop(
    args,
    backend,
    visualizer: Open3DGraspVisualizer | None = None,
    *,
    input_func=input,
    print_func=print,
) -> None:
    period = 1.0 / max(float(args.poll_hz), 0.1)
    iteration = 0
    while True:
        if bool(getattr(args, "manual_trigger", False)):
            if not wait_for_manual_trigger(input_func=input_func, print_func=print_func):
                return
        try:
            iteration += 1
            payload = run_once(args, backend, visualizer=visualizer, iteration=iteration)
            print_func(f"wrote {len(payload.get('candidates', []))} grasp candidates to {args.output_path}")
        except Exception as exc:
            write_backend_missing_payload(args.output_path, reason=f"{type(exc).__name__}: {exc}")
            print_func(f"graspnet bridge failed: {type(exc).__name__}: {exc}")
        if args.once:
            return
        if not bool(getattr(args, "manual_trigger", False)):
            sleep_with_visualizer(period, visualizer)


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        backend = load_backend(
            model_root=args.model_root,
            checkpoint_path=args.checkpoint_path,
            backend_module=args.backend_module,
            device=args.device,
        )
    except Exception as exc:
        write_backend_missing_payload(args.output_path, reason=f"{type(exc).__name__}: {exc}")
        raise SystemExit(
            "GraspNet backend unavailable. Create a module exposing "
            "GraspNetBaselineInference(...).infer(...) and pass --backend-module/--model-root."
        ) from exc

    visualizer = None
    if bool(args.open3d_visualize):
        try:
            visualizer = Open3DGraspVisualizer(
                top_n=int(args.visualize_top_n),
                max_points=int(args.visualize_max_points),
                point_size=float(args.visualize_point_size),
                axis_size=float(args.visualize_axis_size),
                zoom=float(args.visualize_zoom),
                crop_radius_m=float(args.visualize_crop_radius_m),
            )
            print(
                "Open3D visualization enabled: "
                f"top_n={args.visualize_top_n}, max_points={args.visualize_max_points}, "
                f"every_n={resolve_visualize_every_n(args)}"
            )
        except Exception as exc:
            print(f"Open3D visualization disabled: {type(exc).__name__}: {exc}")

    run_bridge_loop(args, backend, visualizer=visualizer)


if __name__ == "__main__":
    main()
