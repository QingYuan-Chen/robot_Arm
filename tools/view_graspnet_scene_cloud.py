from __future__ import annotations

import argparse
from collections import deque
from typing import Any, Iterable

import numpy as np

from graspnet_baseline_inference import build_scene_cloud
from windows_graspnet_baseline_bridge import Open3DGraspVisualizer


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="View the full XYZ+RGB scene cloud reconstructed before GraspNet sampling"
    )
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--candidate-topic", default="/grasp/graspnet_candidates")
    parser.add_argument("--max-input-skew-ms", type=float, default=100.0)
    parser.add_argument("--top-n", type=int, default=10)
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Maximum displayed scene points; 0 keeps the complete depth-gated cloud.",
    )
    parser.add_argument("--point-size", type=float, default=4.0)
    parser.add_argument("--axis-size", type=float, default=0.05)
    parser.add_argument("--zoom", type=float, default=0.28)
    parser.add_argument("--crop-radius-m", type=float, default=0.0)
    return parser


def _stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _color_bgr(message) -> np.ndarray:
    if str(message.encoding).lower() != "bgr8":
        raise ValueError(f"expected bgr8 color image, got {message.encoding!r}")
    height = int(message.height)
    width = int(message.width)
    row_bytes = width * 3
    step = int(getattr(message, "step", row_bytes) or row_bytes)
    if step < row_bytes:
        raise ValueError("color image step is smaller than its packed row size")
    expected = height * step
    data = np.frombuffer(message.data, dtype=np.uint8, count=expected)
    if data.size != expected:
        raise ValueError("color image payload size does not match dimensions")
    return data.reshape(height, step)[:, :row_bytes].reshape(height, width, 3).copy()


def _depth_mm(message) -> np.ndarray:
    encoding = str(message.encoding).lower()
    if encoding not in {"mono16", "16uc1"}:
        raise ValueError(f"expected mono16/16UC1 depth image, got {message.encoding!r}")
    height = int(message.height)
    width = int(message.width)
    row_bytes = width * 2
    step = int(getattr(message, "step", row_bytes) or row_bytes)
    if step < row_bytes or step % 2:
        raise ValueError("depth image step must contain complete 16-bit rows")
    expected = height * step
    packed_bytes = np.frombuffer(message.data, dtype=np.uint8, count=expected)
    if packed_bytes.size != expected:
        raise ValueError("depth image payload size does not match dimensions")
    packed_bytes = packed_bytes.reshape(height, step)[:, :row_bytes].copy()
    dtype = np.dtype(">u2" if bool(message.is_bigendian) else "<u2")
    return packed_bytes.view(dtype).reshape(height, width).astype(np.uint16, copy=True)


def _camera_info(message) -> dict[str, float]:
    values = np.asarray(message.k, dtype=np.float64)
    if values.size < 9 or not np.isfinite(values[:9]).all():
        raise ValueError("CameraInfo K matrix must contain 9 finite values")
    if values[0] <= 0.0 or values[4] <= 0.0:
        raise ValueError("CameraInfo focal lengths must be positive")
    return {
        "fx": float(values[0]),
        "fy": float(values[4]),
        "cx": float(values[2]),
        "cy": float(values[5]),
        "depth_scale_m": 0.001,
    }


def quaternion_xyzw_to_rotation_matrix(quaternion: Iterable[float]) -> np.ndarray:
    values = np.asarray(tuple(quaternion), dtype=np.float64).reshape(4)
    if not np.isfinite(values).all():
        raise ValueError("candidate quaternion must be finite")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("candidate quaternion must have non-zero norm")
    x, y, z, w = values / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def candidate_array_to_visualizer_candidates(message, *, top_n: int) -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    for candidate in list(message.candidates)[: max(0, int(top_n))]:
        position = candidate.pose.position
        orientation = candidate.pose.orientation
        try:
            rotation = quaternion_xyzw_to_rotation_matrix(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
            translation = np.asarray(
                (position.x, position.y, position.z),
                dtype=np.float64,
            )
            score = float(candidate.confidence)
            width = float(candidate.jaw_width)
            height = float(candidate.object_length)
            if not np.isfinite(translation).all() or not np.isfinite((score, width, height)).all():
                raise ValueError("candidate fields must be finite")
        except (TypeError, ValueError):
            continue
        converted.append(
            {
                "score": score,
                "width_m": width,
                "height_m": height,
                # GraspCandidate.msg does not carry GraspNet's depth field.
                # Open3DGraspVisualizer intentionally supplies its 0.04 m fallback.
                "rotation_matrix": rotation.tolist(),
                "translation_xyz": translation.tolist(),
                "object_id": -1,
            }
        )
    return converted


def _closest_timestamped_frame(frames, target_timestamp_ns: int):
    if target_timestamp_ns <= 0 or not frames:
        return None
    return min(frames, key=lambda item: abs(int(item[0]) - int(target_timestamp_ns)))


def match_candidate_rgbd_frames(
    *,
    candidate_stamp_ns: int,
    color_frames,
    depth_frames,
    max_skew_ns: int,
):
    color_frame = _closest_timestamped_frame(color_frames, candidate_stamp_ns)
    depth_frame = _closest_timestamped_frame(depth_frames, candidate_stamp_ns)
    if color_frame is None or depth_frame is None:
        return None
    color_stamp_ns = int(color_frame[0])
    depth_stamp_ns = int(depth_frame[0])
    skew = max(
        abs(color_stamp_ns - int(candidate_stamp_ns)),
        abs(depth_stamp_ns - int(candidate_stamp_ns)),
        abs(color_stamp_ns - depth_stamp_ns),
    )
    if skew > max(0, int(max_skew_ns)):
        return None
    return color_frame, depth_frame


def match_candidate_sensor_frames(
    *,
    candidate_stamp_ns: int,
    color_frames,
    depth_frames,
    camera_info_frames,
    max_skew_ns: int,
):
    color_frame = _closest_timestamped_frame(color_frames, candidate_stamp_ns)
    depth_frame = _closest_timestamped_frame(depth_frames, candidate_stamp_ns)
    camera_info_frame = _closest_timestamped_frame(camera_info_frames, candidate_stamp_ns)
    if color_frame is None or depth_frame is None or camera_info_frame is None:
        return None
    timestamps = [
        int(candidate_stamp_ns),
        int(color_frame[0]),
        int(depth_frame[0]),
        int(camera_info_frame[0]),
    ]
    if max(timestamps) - min(timestamps) > max(0, int(max_skew_ns)):
        return None
    return color_frame, depth_frame, camera_info_frame


def overlay_geometry_is_consistent(
    *,
    candidate_message,
    depth_message,
    camera_info_message,
    depth_shape: tuple[int, int],
) -> bool:
    frame_ids = {
        str(candidate_message.header.frame_id).strip(),
        str(depth_message.header.frame_id).strip(),
        str(camera_info_message.header.frame_id).strip(),
    }
    if "" in frame_ids or len(frame_ids) != 1:
        return False
    height, width = map(int, depth_shape)
    if int(camera_info_message.height) != height or int(camera_info_message.width) != width:
        return False
    try:
        _camera_info(camera_info_message)
    except (TypeError, ValueError):
        return False
    return True


def _candidate_stamp_ns(message) -> int:
    stamp_ns = _stamp_ns(message)
    if stamp_ns > 0 or not message.candidates:
        return stamp_ns
    return _stamp_ns(message.candidates[0])


def shutdown_rclpy_if_live(rclpy_module) -> None:
    if rclpy_module.ok():
        rclpy_module.shutdown()


def cleanup_viewer_runtime(*, visualizer, node, rclpy_module) -> None:
    try:
        if visualizer is not None:
            visualizer.close()
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            shutdown_rclpy_if_live(rclpy_module)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.max_input_skew_ms < 0.0:
        raise ValueError("--max-input-skew-ms must be non-negative")
    if args.top_n < 0:
        raise ValueError("--top-n must be non-negative")
    if args.max_points < 0:
        raise ValueError("--max-points must be non-negative")
    if args.point_size <= 0.0:
        raise ValueError("--point-size must be positive")
    if args.axis_size <= 0.0:
        raise ValueError("--axis-size must be positive")
    if args.zoom <= 0.0:
        raise ValueError("--zoom must be positive")
    if args.crop_radius_m < 0.0:
        raise ValueError("--crop-radius-m must be non-negative")

    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rebotarm_msgs.msg import GraspCandidateArray
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    try:
        node = rclpy.create_node("graspnet_scene_cloud_viewer")
    except BaseException:
        shutdown_rclpy_if_live(rclpy)
        raise
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    color_frames = deque(maxlen=32)
    depth_frames = deque(maxlen=32)
    camera_info_frames = deque(maxlen=32)
    pending_candidates = None

    def on_color(message: Image) -> None:
        color_frames.append((_stamp_ns(message), message))

    def on_depth(message: Image) -> None:
        depth_frames.append((_stamp_ns(message), message))

    def on_camera_info(message: CameraInfo) -> None:
        camera_info_frames.append((_stamp_ns(message), message))

    def on_candidates(message: GraspCandidateArray) -> None:
        nonlocal pending_candidates
        pending_candidates = message

    subscriptions = [
        node.create_subscription(Image, args.color_topic, on_color, qos),
        node.create_subscription(Image, args.depth_topic, on_depth, qos),
        node.create_subscription(CameraInfo, args.camera_info_topic, on_camera_info, qos),
        node.create_subscription(GraspCandidateArray, args.candidate_topic, on_candidates, 10),
    ]
    del subscriptions

    visualizer = None
    try:
        display_max_points = int(args.max_points) if int(args.max_points) > 0 else 2**31 - 1
        visualizer = Open3DGraspVisualizer(
            top_n=int(args.top_n),
            max_points=display_max_points,
            point_size=float(args.point_size),
            axis_size=float(args.axis_size),
            zoom=float(args.zoom),
            crop_radius_m=float(args.crop_radius_m),
            window_name="GraspNet XYZ+RGB scene + raw grasp poses",
        )
        renderer_status = visualizer.gripper_renderer_status
        print(renderer_status, flush=True)
        last_candidate_stamp_ns = -1
        max_skew_ns = int(float(args.max_input_skew_ms) * 1_000_000)

        print(
            "Waiting for timestamp-matched RGB, depth, CameraInfo, and raw GraspNet "
            "candidates; displaying the same 0.05-1.5 m full scene reconstructed "
            "before GraspNet sampling with Top-N raw grasp poses.",
            flush=True,
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            visualizer.poll_events()
            if pending_candidates is None:
                continue
            candidate_stamp_ns = _candidate_stamp_ns(pending_candidates)
            if candidate_stamp_ns <= 0 or candidate_stamp_ns == last_candidate_stamp_ns:
                continue
            matched = match_candidate_sensor_frames(
                candidate_stamp_ns=candidate_stamp_ns,
                color_frames=color_frames,
                depth_frames=depth_frames,
                camera_info_frames=camera_info_frames,
                max_skew_ns=max_skew_ns,
            )
            if matched is None:
                continue
            (color_stamp, color_message), (depth_stamp, depth_message), (
                camera_info_stamp,
                camera_info_message,
            ) = matched

            color_bgr = _color_bgr(color_message)
            depth_mm = _depth_mm(depth_message)
            if color_bgr.shape[:2] != depth_mm.shape:
                raise ValueError(
                    f"aligned RGB/depth size mismatch: {color_bgr.shape[:2]} vs {depth_mm.shape}"
                )
            if not overlay_geometry_is_consistent(
                candidate_message=pending_candidates,
                depth_message=depth_message,
                camera_info_message=camera_info_message,
                depth_shape=depth_mm.shape,
            ):
                print(
                    "Skipping candidate frame: candidate/depth/CameraInfo frame, dimensions, "
                    "or intrinsics are inconsistent.",
                    flush=True,
                )
                last_candidate_stamp_ns = candidate_stamp_ns
                continue
            camera_info = _camera_info(camera_info_message)
            points, colors = build_scene_cloud(
                color_bgr=color_bgr,
                depth_mm=depth_mm,
                camera_info=camera_info,
            )
            candidates = candidate_array_to_visualizer_candidates(
                pending_candidates,
                top_n=int(args.top_n),
            )
            visualizer.update(
                color_bgr=color_bgr,
                depth_mm=depth_mm,
                camera_info=camera_info,
                candidates=candidates,
                scene_points=points,
                scene_colors=colors,
            )
            current_renderer_status = visualizer.gripper_renderer_status
            if current_renderer_status != renderer_status:
                renderer_status = current_renderer_status
                print(renderer_status, flush=True)
            last_candidate_stamp_ns = candidate_stamp_ns
            z_range = "empty"
            if len(points):
                z_range = f"[{points[:, 2].min():.3f},{points[:, 2].max():.3f}]m"
            top_score = float(candidates[0]["score"]) if candidates else float("nan")
            sensor_skew_ms = (
                max(candidate_stamp_ns, color_stamp, depth_stamp, camera_info_stamp)
                - min(candidate_stamp_ns, color_stamp, depth_stamp, camera_info_stamp)
            ) / 1e6
            print(
                f"scene_points={len(points)} z={z_range} raw_candidates="
                f"{len(pending_candidates.candidates)} displayed={len(candidates)} "
                f"top_score={top_score:.6f} candidate_rgbd_skew_ms="
                f"{sensor_skew_ms:.1f}",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_viewer_runtime(
            visualizer=visualizer,
            node=node,
            rclpy_module=rclpy,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
