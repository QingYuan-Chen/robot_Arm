from __future__ import annotations

from collections import deque
import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rebotarm_msgs.msg import Detection2DArray, GraspCandidateArray


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _color_array(msg: Image) -> np.ndarray:
    if msg.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported color encoding: {msg.encoding}")
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    if msg.encoding == "bgr8":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def _depth_array(msg: Image) -> np.ndarray:
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    return np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))


def _pose_matrix(candidate) -> np.ndarray:
    pose = candidate.pose
    qx, qy, qz, qw = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    norm = float(np.linalg.norm([qx, qy, qz, qw]))
    if norm < 1e-9:
        raise ValueError("candidate quaternion is zero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]
    matrix[:3, 3] = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
    if not np.isfinite(matrix).all():
        raise ValueError("candidate pose is not finite")
    return matrix


def _scene_cloud(color, depth, info, max_points: int, detection=None):
    depth_m = depth.astype(np.float32) * 0.001
    valid = np.isfinite(depth_m) & (depth_m >= 0.15) & (depth_m <= 1.5)
    if detection is not None:
        target_mask = np.zeros(depth.shape, dtype=np.uint8)
        polygon = list(getattr(detection, "mask_polygon_xy", []))
        if bool(getattr(detection, "has_mask", False)) and len(polygon) >= 6:
            vertices = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
            cv2.fillPoly(target_mask, [np.round(vertices).astype(np.int32)], 1)
        else:
            x_min = max(0, min(depth.shape[1] - 1, int(detection.x_min)))
            y_min = max(0, min(depth.shape[0] - 1, int(detection.y_min)))
            x_max = max(x_min + 1, min(depth.shape[1], int(detection.x_max)))
            y_max = max(y_min + 1, min(depth.shape[0], int(detection.y_max)))
            target_mask[y_min:y_max, x_min:x_max] = 1
        valid &= target_mask.astype(bool)
    v, u = np.nonzero(valid)
    if len(u) == 0:
        return np.empty((0, 3)), np.empty((0, 3))
    if len(u) > max_points:
        indices = np.linspace(0, len(u) - 1, max_points, dtype=np.int64)
        u, v = u[indices], v[indices]
    z = depth_m[v, u]
    x = (u.astype(np.float32) - float(info.k[2])) * z / float(info.k[0])
    y = (v.astype(np.float32) - float(info.k[5])) * z / float(info.k[4])
    points = np.column_stack((x, y, z)).astype(np.float64)
    colors = color[v, u].astype(np.float32) / 255.0
    return points, colors.astype(np.float64)


def _gripper_geometries(o3d, matrix, jaw_width: float, color):
    width = float(np.clip(jaw_width, 0.006, 0.2))
    finger_length = 0.06
    thickness = 0.004
    geometries = []
    bars = [
        (finger_length, thickness, thickness, (-finger_length, -width / 2 - thickness, -thickness / 2)),
        (finger_length, thickness, thickness, (-finger_length, width / 2, -thickness / 2)),
        (thickness, width + 2 * thickness, thickness, (-finger_length - thickness, -width / 2 - thickness, -thickness / 2)),
    ]
    for sx, sy, sz, offset in bars:
        mesh = o3d.geometry.TriangleMesh.create_box(width=sx, height=sy, depth=sz)
        mesh.translate(offset)
        mesh.transform(matrix)
        mesh.paint_uniform_color(color)
        mesh.compute_vertex_normals()
        geometries.append(mesh)
    return geometries


class Open3DGraspWindow:
    def __init__(self, *, point_size: float = 2.0):
        import open3d as o3d

        self.o3d = o3d
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        if not self.vis.create_window("GraspNet | RGB-D cloud + raw grasp candidates", 1280, 800):
            raise RuntimeError("Open3D window creation failed; run from a desktop session")
        options = self.vis.get_render_option()
        options.point_size = float(point_size)
        options.background_color = np.array([0.035, 0.045, 0.055])
        self.fitted = False

    def update(self, points, colors, candidates):
        self.vis.clear_geometries()
        cloud = self.o3d.geometry.PointCloud()
        cloud.points = self.o3d.utility.Vector3dVector(points)
        cloud.colors = self.o3d.utility.Vector3dVector(colors)
        self.vis.add_geometry(cloud, reset_bounding_box=False)
        for index, candidate in enumerate(candidates[:5]):
            try:
                matrix = _pose_matrix(candidate)
            except ValueError:
                continue
            color = [0.15, 1.0, 0.3] if index == 0 else [1.0, 0.3, 0.12]
            for geometry in _gripper_geometries(
                self.o3d, matrix, float(candidate.jaw_width), color
            ):
                self.vis.add_geometry(geometry, reset_bounding_box=False)
            if index == 0:
                axes = self.o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.035)
                axes.transform(matrix)
                self.vis.add_geometry(axes, reset_bounding_box=False)
        if not self.fitted:
            self.vis.reset_view_point(True)
            view = self.vis.get_view_control()
            focus = np.median(points, axis=0) if len(points) else [0.0, 0.0, 0.5]
            view.set_lookat(focus)
            view.set_front([0.0, 0.0, -1.0])
            view.set_up([0.0, -1.0, 0.0])
            view.set_zoom(0.7)
            self.fitted = True
        self.vis.update_renderer()

    def poll(self):
        alive = self.vis.poll_events()
        self.vis.update_renderer()
        return alive

    def close(self):
        self.vis.destroy_window()


class GraspNetOpen3DViewer(Node):
    def __init__(self, window=None):
        super().__init__("rebotarm_graspnet_open3d_viewer")
        self.declare_parameter("input_color_topic", "/camera/color/image_raw")
        self.declare_parameter("input_depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("input_candidates_topic", "/grasp/graspnet_candidates")
        self.declare_parameter("max_points", 30000)
        self.declare_parameter("max_frame_age_sec", 1.5)
        self.declare_parameter("point_size", 2.0)
        value = lambda name: self.get_parameter(name).value
        self.window = window or Open3DGraspWindow(point_size=float(value("point_size")))
        self.colors = deque(maxlen=8)
        self.depths = deque(maxlen=8)
        self.info = None
        self.detection = None
        self.pending = None
        self.lock = threading.Lock()
        self.create_subscription(Image, str(value("input_color_topic")), self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, str(value("input_depth_topic")), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(value("input_camera_info_topic")), self._on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/grasp/detections", self._on_detections, 10)
        self.create_subscription(GraspCandidateArray, str(value("input_candidates_topic")), self._on_candidates, 10)
        self.get_logger().info("Open3D raw GraspNet viewer ready; no RViz, motion, or hardware interfaces")

    def _on_color(self, message):
        try:
            self.colors.append((_stamp_ns(message.header.stamp), _color_array(message)))
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_depth(self, message):
        try:
            self.depths.append((_stamp_ns(message.header.stamp), _depth_array(message)))
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_info(self, message):
        self.info = message

    def _on_detections(self, message):
        detections = list(message.detections)
        self.detection = max(detections, key=lambda item: float(item.confidence)) if detections else None

    def _on_candidates(self, message):
        if not message.candidates:
            with self.lock:
                self.pending = None
            return
        stamp = _stamp_ns(message.header.stamp)
        with self.lock:
            self.pending = (stamp, message)

    def render_pending(self):
        with self.lock:
            pending, self.pending = self.pending, None
        if pending is not None and self.info is not None:
            stamp, candidates = pending
            color = min(self.colors, key=lambda item: abs(item[0] - stamp), default=None)
            depth = min(self.depths, key=lambda item: abs(item[0] - stamp), default=None)
            max_age_ns = int(float(self.get_parameter("max_frame_age_sec").value) * 1e9)
            if color and depth and abs(color[0] - stamp) <= max_age_ns and abs(depth[0] - stamp) <= max_age_ns:
                try:
                    points, colors = _scene_cloud(
                        color[1],
                        depth[1],
                        self.info,
                        int(self.get_parameter("max_points").value),
                        self.detection,
                    )
                    self.window.update(points, colors, list(candidates.candidates))
                    best = candidates.candidates[0]
                    self.get_logger().info(
                        "best grasp: "
                        f"frame={candidates.header.frame_id} "
                        f"score={float(best.confidence):.4f} "
                        f"jaw={float(best.jaw_width):.4f}m "
                        f"position=({float(best.pose.position.x):.4f}, "
                        f"{float(best.pose.position.y):.4f}, "
                        f"{float(best.pose.position.z):.4f}) "
                        f"quaternion=({float(best.pose.orientation.x):.4f}, "
                        f"{float(best.pose.orientation.y):.4f}, "
                        f"{float(best.pose.orientation.z):.4f}, "
                        f"{float(best.pose.orientation.w):.4f})"
                    )
                except (ValueError, IndexError) as exc:
                    self.get_logger().warn(f"Open3D grasp frame rejected: {exc}")
        return self.window.poll()

    def destroy_node(self):
        self.window.close()
        return super().destroy_node()


def main(args=None):
    if not __import__("os").environ.get("DISPLAY"):
        raise RuntimeError("Open3D viewer requires a desktop DISPLAY")
    rclpy.init(args=args)
    node = GraspNetOpen3DViewer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok() and node.render_pending():
            pass
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
