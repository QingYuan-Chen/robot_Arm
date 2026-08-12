"""MuJoCo offscreen RGB-D rendering and ground-truth object annotations."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Callable, Sequence

import numpy as np


@dataclass(frozen=True)
class VirtualCameraConfig:
    camera_name: str = "fixed_camera"
    frame_id: str = "mujoco_fixed_camera_optical_frame"
    parent_body_name: str = "base_link"
    parent_frame_id: str = "base_link"
    width: int = 640
    height: int = 480
    rate_hz: float = 15.0
    max_depth_m: float = 2.0
    annotation_bodies: tuple[str, ...] = ("bottle",)

    def __post_init__(self) -> None:
        camera_name = str(self.camera_name).strip()
        frame_id = str(self.frame_id).strip().strip("/")
        parent_body_name = str(self.parent_body_name).strip()
        parent_frame_id = str(self.parent_frame_id).strip().strip("/")
        if not camera_name or any(character.isspace() for character in camera_name):
            raise ValueError("virtual camera name must be non-empty and contain no whitespace")
        if not frame_id or any(character.isspace() for character in frame_id):
            raise ValueError("virtual camera frame_id must be non-empty and contain no whitespace")
        if not parent_body_name or any(
            character.isspace() for character in parent_body_name
        ):
            raise ValueError(
                "virtual camera parent_body_name must be non-empty and contain no whitespace"
            )
        if not parent_frame_id or any(
            character.isspace() for character in parent_frame_id
        ):
            raise ValueError(
                "virtual camera parent_frame_id must be non-empty and contain no whitespace"
            )
        if parent_frame_id == frame_id:
            raise ValueError("virtual camera parent and child frame IDs must differ")
        if isinstance(self.width, bool) or not 16 <= int(self.width) <= 4096:
            raise ValueError("virtual camera width must be in [16, 4096]")
        if isinstance(self.height, bool) or not 16 <= int(self.height) <= 4096:
            raise ValueError("virtual camera height must be in [16, 4096]")
        rate_hz = float(self.rate_hz)
        max_depth_m = float(self.max_depth_m)
        if not math.isfinite(rate_hz) or not 0.0 < rate_hz <= 120.0:
            raise ValueError("virtual camera rate_hz must be finite and in (0, 120]")
        if not math.isfinite(max_depth_m) or not 0.0 < max_depth_m <= 65.535:
            raise ValueError("virtual camera max_depth_m must be finite and in (0, 65.535]")
        bodies = tuple(str(name).strip() for name in self.annotation_bodies)
        if not bodies or any(not name or any(char.isspace() for char in name) for name in bodies):
            raise ValueError("annotation_bodies must contain non-empty MuJoCo body names")
        if len(set(bodies)) != len(bodies):
            raise ValueError("annotation_bodies must not contain duplicates")
        object.__setattr__(self, "camera_name", camera_name)
        object.__setattr__(self, "frame_id", frame_id)
        object.__setattr__(self, "parent_body_name", parent_body_name)
        object.__setattr__(self, "parent_frame_id", parent_frame_id)
        object.__setattr__(self, "width", int(self.width))
        object.__setattr__(self, "height", int(self.height))
        object.__setattr__(self, "rate_hz", rate_hz)
        object.__setattr__(self, "max_depth_m", max_depth_m)
        object.__setattr__(self, "annotation_bodies", bodies)


@dataclass(frozen=True)
class PinholeIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class VirtualCameraExtrinsics:
    parent_frame_id: str
    child_frame_id: str
    translation_xyz: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class VirtualObjectAnnotation:
    class_name: str
    x_min: int
    y_min: int
    x_max: int
    y_max: int

    @property
    def center_u(self) -> int:
        return int(round((self.x_min + self.x_max) / 2.0))

    @property
    def center_v(self) -> int:
        return int(round((self.y_min + self.y_max) / 2.0))

    @property
    def mask_polygon_xy(self) -> tuple[float, ...]:
        return (
            float(self.x_min),
            float(self.y_min),
            float(self.x_max),
            float(self.y_min),
            float(self.x_max),
            float(self.y_max),
            float(self.x_min),
            float(self.y_max),
        )


@dataclass(frozen=True)
class VirtualCameraFrame:
    rgb: np.ndarray
    depth_mm: np.ndarray
    annotations: tuple[VirtualObjectAnnotation, ...]


def pinhole_intrinsics(width: int, height: int, fovy_degrees: float) -> PinholeIntrinsics:
    width = int(width)
    height = int(height)
    fovy = float(fovy_degrees)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if not math.isfinite(fovy) or not 0.0 < fovy < 180.0:
        raise ValueError("vertical field of view must be finite and in (0, 180)")
    focal = 0.5 * height / math.tan(math.radians(fovy) * 0.5)
    return PinholeIntrinsics(
        width=width,
        height=height,
        fx=focal,
        fy=focal,
        cx=(width - 1) * 0.5,
        cy=(height - 1) * 0.5,
    )


def rotation_matrix_to_quaternion_xyzw(matrix: Any) -> tuple[float, float, float, float]:
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("rotation matrix must be finite and have shape (3, 3)")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not math.isclose(
        float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6
    ):
        raise ValueError("rotation matrix must be orthonormal and right-handed")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return tuple(float(value) for value in quaternion)


def camera_optical_transform(
    *,
    camera_position_world: Any,
    camera_rotation_world_mujoco: Any,
    parent_position_world: Any,
    parent_rotation_world: Any,
    parent_frame_id: str,
    child_frame_id: str,
) -> VirtualCameraExtrinsics:
    camera_position = np.asarray(camera_position_world, dtype=np.float64)
    parent_position = np.asarray(parent_position_world, dtype=np.float64)
    camera_rotation = np.asarray(camera_rotation_world_mujoco, dtype=np.float64)
    parent_rotation = np.asarray(parent_rotation_world, dtype=np.float64)
    if camera_position.shape != (3,) or parent_position.shape != (3,):
        raise ValueError("camera and parent positions must have shape (3,)")
    if not np.isfinite(camera_position).all() or not np.isfinite(parent_position).all():
        raise ValueError("camera and parent positions must be finite")
    # MuJoCo cameras use +X right, +Y up and -Z forward. ROS optical frames use
    # +X right, +Y down and +Z forward.
    mujoco_to_optical = np.diag([1.0, -1.0, -1.0])
    rotation_parent_optical = parent_rotation.T @ camera_rotation @ mujoco_to_optical
    translation_parent_optical = parent_rotation.T @ (camera_position - parent_position)
    return VirtualCameraExtrinsics(
        parent_frame_id=str(parent_frame_id),
        child_frame_id=str(child_frame_id),
        translation_xyz=tuple(float(value) for value in translation_parent_optical),
        rotation_xyzw=rotation_matrix_to_quaternion_xyzw(rotation_parent_optical),
    )


def metric_depth_to_millimeters(
    depth_m: Any,
    *,
    max_depth_m: float,
    valid_mask: Any | None = None,
) -> np.ndarray:
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError("depth image must be two-dimensional")
    limit = float(max_depth_m)
    if not math.isfinite(limit) or not 0.0 < limit <= 65.535:
        raise ValueError("max_depth_m must be finite and in (0, 65.535]")
    valid = np.isfinite(depth) & (depth > 0.0) & (depth <= limit)
    if valid_mask is not None:
        mask = np.asarray(valid_mask, dtype=bool)
        if mask.shape != depth.shape:
            raise ValueError("valid_mask shape must match depth image")
        valid &= mask
    result = np.zeros(depth.shape, dtype=np.uint16)
    result[valid] = np.clip(np.rint(depth[valid] * 1000.0), 1, 65535).astype(np.uint16)
    return result


def segmentation_mask(
    segmentation: Any,
    geom_ids: Sequence[int],
    *,
    geom_object_type: int,
) -> np.ndarray:
    values = np.asarray(segmentation)
    if values.ndim != 3 or values.shape[2] != 2:
        raise ValueError("MuJoCo segmentation image must have shape (height, width, 2)")
    identifiers = tuple(int(value) for value in geom_ids)
    if not identifiers:
        return np.zeros(values.shape[:2], dtype=bool)
    # MuJoCo 3.x stores (object_id, object_type) in the two channels.
    return (values[:, :, 1] == int(geom_object_type)) & np.isin(
        values[:, :, 0], identifiers
    )


def bounding_box_from_mask(mask: Any) -> tuple[int, int, int, int] | None:
    values = np.asarray(mask, dtype=bool)
    if values.ndim != 2:
        raise ValueError("annotation mask must be two-dimensional")
    rows, columns = np.nonzero(values)
    if rows.size == 0:
        return None
    return int(columns.min()), int(rows.min()), int(columns.max()), int(rows.max())


class VirtualCameraRenderer:
    """Own one MuJoCo renderer tied to an existing simulation model/data pair."""

    def __init__(self, mujoco_module: Any, model: Any, data: Any, config: VirtualCameraConfig):
        self._mj = mujoco_module
        self._model = model
        self._data = data
        self.config = config
        self._closed = False
        camera_id = int(
            self._mj.mj_name2id(
                self._model, self._mj.mjtObj.mjOBJ_CAMERA, config.camera_name
            )
        )
        if camera_id < 0:
            raise ValueError(f"MuJoCo camera not found: {config.camera_name}")
        if hasattr(self._model, "cam_orthographic") and bool(
            self._model.cam_orthographic[camera_id]
        ):
            raise ValueError("orthographic MuJoCo cameras are not supported")
        self.intrinsics = pinhole_intrinsics(
            config.width, config.height, float(self._model.cam_fovy[camera_id])
        )
        parent_body_id = int(
            self._mj.mj_name2id(
                self._model, self._mj.mjtObj.mjOBJ_BODY, config.parent_body_name
            )
        )
        if parent_body_id < 0:
            raise ValueError(
                f"MuJoCo virtual camera parent body not found: {config.parent_body_name}"
            )
        self.extrinsics = camera_optical_transform(
            camera_position_world=np.asarray(self._data.cam_xpos[camera_id]),
            camera_rotation_world_mujoco=np.asarray(
                self._data.cam_xmat[camera_id]
            ).reshape(3, 3),
            parent_position_world=np.asarray(self._data.xpos[parent_body_id]),
            parent_rotation_world=np.asarray(self._data.xmat[parent_body_id]).reshape(3, 3),
            parent_frame_id=config.parent_frame_id,
            child_frame_id=config.frame_id,
        )
        self._geom_object_type = int(self._mj.mjtObj.mjOBJ_GEOM)
        self._body_geoms: dict[str, tuple[int, ...]] = {}
        for body_name in config.annotation_bodies:
            body_id = int(
                self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_BODY, body_name)
            )
            if body_id < 0:
                raise ValueError(f"MuJoCo annotation body not found: {body_name}")
            geom_ids = tuple(
                int(index)
                for index in np.flatnonzero(
                    np.asarray(self._model.geom_bodyid, dtype=np.int64) == body_id
                )
            )
            if not geom_ids:
                raise ValueError(f"MuJoCo annotation body has no geometry: {body_name}")
            self._body_geoms[body_name] = geom_ids
        self._renderer = self._mj.Renderer(
            self._model, height=config.height, width=config.width
        )

    @classmethod
    def from_simulation(cls, simulation: Any, config: VirtualCameraConfig):
        model, data = simulation._unsafe_viewer_handles()
        return cls(simulation._mj, model, data, config)

    def render(self) -> VirtualCameraFrame:
        if self._closed:
            raise RuntimeError("virtual camera renderer is closed")
        self._renderer.update_scene(self._data, camera=self.config.camera_name)
        rgb = np.ascontiguousarray(self._renderer.render().copy())

        self._renderer.enable_depth_rendering()
        try:
            self._renderer.update_scene(self._data, camera=self.config.camera_name)
            depth_m = self._renderer.render().copy()
        finally:
            self._renderer.disable_depth_rendering()

        self._renderer.enable_segmentation_rendering()
        try:
            self._renderer.update_scene(self._data, camera=self.config.camera_name)
            segmentation = self._renderer.render().copy()
        finally:
            self._renderer.disable_segmentation_rendering()

        geometry_mask = segmentation[:, :, 1] == self._geom_object_type
        depth_mm = metric_depth_to_millimeters(
            depth_m,
            max_depth_m=self.config.max_depth_m,
            valid_mask=geometry_mask,
        )
        annotations = []
        for body_name, geom_ids in self._body_geoms.items():
            mask = segmentation_mask(
                segmentation,
                geom_ids,
                geom_object_type=self._geom_object_type,
            )
            bbox = bounding_box_from_mask(mask)
            if bbox is not None:
                annotations.append(VirtualObjectAnnotation(body_name, *bbox))
        return VirtualCameraFrame(rgb, depth_mm, tuple(annotations))

    def close(self) -> None:
        if not self._closed:
            self._renderer.close()
            self._closed = True


class VirtualCameraWorker:
    """Keep an EGL renderer's complete lifecycle on one dedicated thread."""

    def __init__(
        self,
        simulation_access: Any,
        config: VirtualCameraConfig,
        *,
        on_frame: Callable[[VirtualCameraFrame, PinholeIntrinsics, tuple[int, int]], None],
        on_ready: Callable[[PinholeIntrinsics, VirtualCameraExtrinsics], None],
        on_error: Callable[[BaseException], None],
    ) -> None:
        self._simulation_access = simulation_access
        self._config = config
        self._on_frame = on_frame
        self._on_ready = on_ready
        self._on_error = on_error
        self._condition = threading.Condition()
        self._pending_stamp: tuple[int, int] | None = None
        self._stop_requested = False
        self._thread = threading.Thread(
            target=self._run,
            name="rebotarm-mujoco-virtual-camera",
            daemon=True,
        )
        self._thread.start()

    def submit(self, stamp: tuple[int, int]) -> bool:
        seconds, nanoseconds = int(stamp[0]), int(stamp[1])
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise ValueError("virtual camera stamp is invalid")
        with self._condition:
            if self._stop_requested or not self._thread.is_alive():
                return False
            # Keep only the newest request so rendering can never build backlog.
            self._pending_stamp = (seconds, nanoseconds)
            self._condition.notify()
            return True

    def close(self, timeout_sec: float = 5.0) -> bool:
        with self._condition:
            self._stop_requested = True
            self._pending_stamp = None
            self._condition.notify()
        self._thread.join(timeout=float(timeout_sec))
        return not self._thread.is_alive()

    def _run(self) -> None:
        renderer: VirtualCameraRenderer | None = None
        try:
            renderer = self._simulation_access.run(
                lambda simulation: VirtualCameraRenderer.from_simulation(
                    simulation, self._config
                )
            )
            self._on_ready(renderer.intrinsics, renderer.extrinsics)
            while True:
                with self._condition:
                    self._condition.wait_for(
                        lambda: self._stop_requested or self._pending_stamp is not None
                    )
                    if self._stop_requested:
                        return
                    stamp = self._pending_stamp
                    self._pending_stamp = None
                frame = self._simulation_access.run(
                    lambda _simulation: renderer.render()
                )
                self._on_frame(frame, renderer.intrinsics, stamp)
        except BaseException as exc:
            self._on_error(exc)
        finally:
            if renderer is not None:
                try:
                    self._simulation_access.run(
                        lambda _simulation: renderer.close()
                    )
                except BaseException as exc:
                    self._on_error(exc)
