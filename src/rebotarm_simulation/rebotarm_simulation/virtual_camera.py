"""MuJoCo 离屏 RGB-D 渲染与物体真值标注。

本模块为仿真后端提供“虚拟相机”：复用 MuJoCo 的离屏渲染器（无窗口图形上下文，本机默认
``MUJOCO_GL=egl``；无 EGL 设备时应保持该功能关闭）输出与真实相机同构的 RGB、深度和分割
图，再由分割图生成物体级真值包围盒、掩膜多边形，使感知与抓取链路可以在没有真实相机的
情况下做联调与回归。

职责范围：
- 校验并规范化虚拟相机配置（相机名、分辨率、帧率、深度量程、待标注刚体）；
- 由相机垂直视场角推导针孔内参，把相机的世界位姿换算为父坐标系下的光学系外参；
- 渲染 RGB / 深度 / 分割三张图，并把米制深度量化为 16 位毫米图；
- 由分割图给出每个待标注刚体的轴对齐包围盒与掩膜多边形。

坐标系与单位约定：
- 深度以毫米整数（``mono16`` 编码）输出，无效或超量程像素记为 0；
- 内参只由垂直视场角与图像高度决定，主点取像素中心 ``(width-1)/2``、``(height-1)/2``；
- 外参是父坐标系（默认机器人基座）到相机光学坐标系的变换，四元数顺序为 xyzw，平移单位 m；
  MuJoCo 相机轴向为 +X 右 / +Y 上 / -Z 前，光学坐标系为 +X 右 / +Y 下 / +Z 前，换算时
  右乘 diag(1, -1, -1) 完成轴向翻转。

线程与边界：
- MuJoCo 的渲染上下文绑定创建它的线程，因此渲染器必须在同一线程内构造、渲染与销毁，
  ``VirtualCameraWorker`` 用一条专用 daemon 线程承担这一职责；
- 所有对仿真 model/data 的访问都经上层传入的串行化访问器执行，避免与物理步进并发；
- 本模块只读取仿真状态用于成像，不推进物理步进、不下发控制指令、也不接触任何真实硬件
  通道，属纯仿真侧组件。
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any, Callable, Sequence

import numpy as np


@dataclass(frozen=True)
class VirtualCameraConfig:
    """虚拟相机配置（不可变，构造即校验）。

    只描述“相机在哪、以多大分辨率多快出图、深度取到多远、给哪些刚体出真值标注”，本身
    不持有任何渲染资源；渲染资源由 ``VirtualCameraRenderer`` 在渲染线程内按需创建。
    """

    # 相机名：必须在 MuJoCo 模型中存在（按名字查 mjOBJ_CAMERA），决定视角与安装位姿
    camera_name: str = "fixed_camera"
    # 图像/CameraInfo 的 header.frame_id，也是 TF 中的子坐标系（光学坐标系）
    frame_id: str = "mujoco_fixed_camera_optical_frame"
    # 相机挂载的 MuJoCo body 名：外参以该 body 的坐标系为参考
    parent_body_name: str = "base_link"
    # 父坐标系名，与 parent_body_name 对应，作为静态 TF 的 header.frame_id
    parent_frame_id: str = "base_link"
    # 渲染宽度（像素），取值范围 [16, 4096]
    width: int = 640
    # 渲染高度（像素），取值范围 [16, 4096]；与 width 共同决定内参、图像步长与数据量
    height: int = 480
    # 期望渲染频率（Hz），取值范围 (0, 120]；由上层按仿真时间节流，超出即丢帧而非排队
    rate_hz: float = 15.0
    # 深度有效上限（m），取值范围 (0, 65.535]：受 16 位毫米计数上限约束，超限像素记 0
    max_depth_m: float = 2.0
    # 需要输出真值包围盒/掩膜的 MuJoCo body 名元组，必须非空、无重复、无空白字符；
    # 默认只标注 "bottle"；新增目标时必须保证模型中存在同名 body 且其下挂有几何体
    annotation_bodies: tuple[str, ...] = ("bottle",)

    def __post_init__(self) -> None:
        """规范化字符串/数值并逐项校验，非法配置在构造期抛 ``ValueError``。

        校验规则（宁可启动即失败，也不要等渲染线程起来后才崩）：
        - 名字类字段必须非空且不含空白字符，frame 名额外去掉首尾 ``/``；
        - 父、子坐标系名不能相同，否则静态 TF 会自环；
        - 宽高取整后必须落在 [16, 4096]，并显式排除 bool（避免 True 被当成 1）；
        - 频率取浮点后必须是 (0, 120] 内的有限值；
        - 深度上限取浮点后必须是 (0, 65.535] 内的有限值（65.535 m 即 65535 mm，是 16 位
          毫米计数的最大可表示值）；
        - ``annotation_bodies`` 必须非空、无空白、无重复。

        由于 dataclass 被冻结，规范化结果通过 ``object.__setattr__`` 写回。
        """
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
    """针孔相机内参（像素单位，按无畸变理想模型假设）。"""

    # 图像宽、高（像素），与渲染尺寸一致，用于校验并填充 CameraInfo
    width: int
    height: int
    # fx / fy：x、y 方向焦距（像素）。虚拟相机像素为正方形，两者恒等
    fx: float
    fy: float
    # cx / cy：主点（光轴与像面交点）像素坐标，取图像几何中心 (w-1)/2、(h-1)/2
    cx: float
    cy: float


@dataclass(frozen=True)
class VirtualCameraExtrinsics:
    """相机外参：父坐标系 → 相机光学坐标系的刚体变换，可直接填入静态 TF。"""

    # 父坐标系名（静态 TF 的 header.frame_id）
    parent_frame_id: str
    # 子坐标系名（静态 TF 的 child_frame_id），即虚拟相机的光学坐标系
    child_frame_id: str
    # 平移 (x, y, z)，在父坐标系下表达，单位 m
    translation_xyz: tuple[float, float, float]
    # 姿态四元数 (x, y, z, w)：把父坐标系旋转到相机光学坐标系，已归一化且 w >= 0
    rotation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class VirtualObjectAnnotation:
    """单个目标物体的真值标注（轴对齐包围盒）。"""

    # 类别名：当前直接取 MuJoCo body 名（如 "bottle"），上层原样填入检测结果的 class_name
    class_name: str
    # 包围盒像素边界（图像坐标，原点在左上角、x 向右、y 向下），四值均为闭区间端点
    x_min: int
    y_min: int
    x_max: int
    y_max: int

    @property
    def center_u(self) -> int:
        # 包围盒中心列坐标（四舍五入到整数像素），对应检测接口的 center_u
        return int(round((self.x_min + self.x_max) / 2.0))

    @property
    def center_v(self) -> int:
        # 包围盒中心行坐标，对应检测接口的 center_v
        return int(round((self.y_min + self.y_max) / 2.0))

    @property
    def mask_polygon_xy(self) -> tuple[float, ...]:
        # 掩膜多边形按 x1, y1, x2, y2, ... 扁平存放：左上→右上→右下→左下（矩形四角）
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
    """一次渲染的输出：RGB、毫米深度与真值标注（三张图共用同一时间戳与光学坐标系）。"""

    # RGB 图像：形状 (height, width, 3)、dtype uint8、通道序 rgb8
    rgb: np.ndarray
    # 深度图像：形状 (height, width)、dtype uint16，单位毫米；0 表示无效/超量程
    depth_mm: np.ndarray
    # 本帧中可见的目标标注；被完全遮挡或不可见的目标不会出现在这里
    annotations: tuple[VirtualObjectAnnotation, ...]


def pinhole_intrinsics(width: int, height: int, fovy_degrees: float) -> PinholeIntrinsics:
    """由垂直视场角推导针孔内参（像素）。

    焦距按 ``f = 0.5 * height / tan(fovy / 2)`` 计算：MuJoCo 的 ``cam_fovy`` 是垂直视场
    角（度），只与图像高度有关；x/y 用同一焦距，等价于假设正方形像素。主点取图像几何中心
    ``(width - 1) / 2``、``(height - 1) / 2``，与上层填充 CameraInfo 的口径一致。

    宽、高必须为正；``fovy_degrees`` 必须是 (0, 180) 内的有限值，否则抛 ``ValueError``。
    """
    width = int(width)
    height = int(height)
    fovy = float(fovy_degrees)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if not math.isfinite(fovy) or not 0.0 < fovy < 180.0:
        raise ValueError("vertical field of view must be finite and in (0, 180)")
    # 半视场角的正切把"半高"映射到焦距；这里用弧度制，故先做 math.radians 换算
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
    """把 3x3 旋转矩阵转换为单位四元数，返回顺序 (x, y, z, w)。

    先校验矩阵有限、正交（``R^T R = I``）且右手（det = +1），容差 1e-6；不满足直接抛
    ``ValueError``，避免把带缩放或镜像的矩阵静默当成旋转。

    数值上采用“取最大主元”的分支写法：trace 明显为正时走 trace 分支，否则选对角元最大的
    分量作分母（Shepperd 思路），避免 w 接近 0 时除以小量而放大误差。结果归一化后统一取
    w >= 0 的那一半表示，保证同一旋转的输出唯一，便于比较与测试。
    """
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("rotation matrix must be finite and have shape (3, 3)")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not math.isclose(
        float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6
    ):
        raise ValueError("rotation matrix must be orthonormal and right-handed")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        # trace > 0：w 分量最大，此时用 trace 作主元数值最稳
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        # trace <= 0：改用对角元最大者所在分支，保证分母 scale 远离 0
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
    # q 与 -q 表示同一旋转，这里约定 w 非负以消除二义性
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
    """把 MuJoCo 相机的世界位姿换算为父坐标系下的光学系外参。

    参数（全部为关键字参数）：
    - ``camera_position_world`` / ``camera_rotation_world_mujoco``：相机在 MuJoCo 世界系中
      的位置 (3,) 与旋转 (3, 3)，直接取自 ``data.cam_xpos`` 与 ``data.cam_xmat``；
    - ``parent_position_world`` / ``parent_rotation_world``：父 body 的世界位姿，取自
      ``data.xpos`` 与 ``data.xmat``；
    - ``parent_frame_id`` / ``child_frame_id``：写入结果的坐标系名。

    平移与旋转都统一在父坐标系下表达：``R_parent^T · R_cam`` 与
    ``R_parent^T · (p_cam - p_parent)``（``R_parent`` 为正交矩阵，故转置即逆）；随后右乘
    ``diag(1, -1, -1)`` 把 MuJoCo 相机轴向换成光学坐标系轴向。位置必须为有限值，旋转的
    正交性由四元数转换函数负责校验。
    """
    camera_position = np.asarray(camera_position_world, dtype=np.float64)
    parent_position = np.asarray(parent_position_world, dtype=np.float64)
    camera_rotation = np.asarray(camera_rotation_world_mujoco, dtype=np.float64)
    parent_rotation = np.asarray(parent_rotation_world, dtype=np.float64)
    if camera_position.shape != (3,) or parent_position.shape != (3,):
        raise ValueError("camera and parent positions must have shape (3,)")
    if not np.isfinite(camera_position).all() or not np.isfinite(parent_position).all():
        raise ValueError("camera and parent positions must be finite")
    # MuJoCo 相机轴向为 +X 右、+Y 上、-Z 前；ROS 光学坐标系为 +X 右、+Y 下、+Z 前，
    # 因此用 diag(1, -1, -1) 翻转 Y、Z 两轴即可把相机系旋转映射到光学系旋转。
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
    """米制浮点深度图 → 16 位毫米深度图，无效像素一律置 0。

    有效条件：数值有限、大于 0 且不超过 ``max_depth_m``；若给出 ``valid_mask``，还必须落在
    掩膜内（上层通常传入分割图的几何体掩膜，因为 MuJoCo 深度图中天空/背景的数值不可用）。
    有效值乘 1000 后四舍五入并截断到 [1, 65535]：下界取 1 使“贴近相机”与“无深度(0)”可以
    区分，上界由 uint16 表示范围决定（因此 ``max_depth_m`` 最大只能到 65.535 m）。

    返回与输入同形状的 uint16 数组；``depth_m`` 不是二维或 ``valid_mask`` 形状不匹配时抛
    ``ValueError``。
    """
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError("depth image must be two-dimensional")
    limit = float(max_depth_m)
    if not math.isfinite(limit) or not 0.0 < limit <= 65.535:
        raise ValueError("max_depth_m must be finite and in (0, 65.535]")
    # NaN/Inf、非正深度与超量程都视为"无深度"，最终保持 0
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
    """从 MuJoCo 分割图中取出指定几何体的布尔掩膜。

    MuJoCo 3.x 的分割图为整型数组、形状 (height, width, 2)：通道 0 存对象 id，通道 1 存
    对象类型（此处由调用方传入 ``mjOBJ_GEOM``，即几何体类型）。因此“该像素属于这些几何体”
    等价于“类型是几何体且 geom id 在给定集合内”。``geom_ids`` 为空时直接返回全 False 的
    掩膜，不做空集合比较。分割图形状不符时抛 ``ValueError``。
    """
    values = np.asarray(segmentation)
    if values.ndim != 3 or values.shape[2] != 2:
        raise ValueError("MuJoCo segmentation image must have shape (height, width, 2)")
    identifiers = tuple(int(value) for value in geom_ids)
    if not identifiers:
        return np.zeros(values.shape[:2], dtype=bool)
    # 通道 1 为对象类型、通道 0 为对象 id，两个条件同时成立才算命中目标几何体
    return (values[:, :, 1] == int(geom_object_type)) & np.isin(
        values[:, :, 0], identifiers
    )


def bounding_box_from_mask(mask: Any) -> tuple[int, int, int, int] | None:
    """求布尔掩膜的轴对齐包围盒，返回 ``(x_min, y_min, x_max, y_max)``（像素，闭区间）。

    掩膜必须二维；掩膜全为 False（目标不可见或被完全遮挡）时返回 ``None``，调用方据此不为
    该目标生成标注。
    """
    values = np.asarray(mask, dtype=bool)
    if values.ndim != 2:
        raise ValueError("annotation mask must be two-dimensional")
    rows, columns = np.nonzero(values)
    if rows.size == 0:
        return None
    return int(columns.min()), int(rows.min()), int(columns.max()), int(rows.max())


class VirtualCameraRenderer:
    """持有一个绑定到既有 MuJoCo model/data 的离屏渲染器。

    构造阶段完成一次性解析：相机 id、父 body id、每个待标注 body 名下的 geom id 列表，并
    由 ``cam_fovy`` 算出内参、由当前仿真状态算出外参；每次 ``render`` 前都会 ``update_scene``
    重新同步仿真状态，因此能跟随机械臂与物体的运动。

    线程约束：MuJoCo 渲染上下文绑定创建它的线程，渲染器必须在同一线程内构造、渲染与销毁，
    否则可能出现上下文丢失或崩溃。本类不关心物理步进，调用方需自行保证访问 model/data 时
    与步进互斥（上层通过串行化访问器做到这一点）。
    """

    def __init__(self, mujoco_module: Any, model: Any, data: Any, config: VirtualCameraConfig):
        """解析相机与刚体句柄并创建离屏渲染器（会占用一个图形上下文）。

        ``mujoco_module`` 为已导入的 MuJoCo 模块（避免本模块在 import 期强依赖 MuJoCo）；
        ``model`` / ``data`` 来自上层仿真实例，调用期间不得有并发的物理步进。

        相机名不存在、相机为正交投影、父 body 不存在、待标注 body 不存在或其下没有任何
        几何体时，均抛 ``ValueError``。
        """
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
        # 正交相机没有 fovy 语义，针孔内参公式不成立，故直接拒绝而不是给出错误内参
        if hasattr(self._model, "cam_orthographic") and bool(
            self._model.cam_orthographic[camera_id]
        ):
            raise ValueError("orthographic MuJoCo cameras are not supported")
        # 内参只由垂直视场角与渲染尺寸决定，与相机安装位姿无关
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
        # 外参只在构造时计算一次并由 on_ready 上报一次；默认配置是固定在基座上的相机，
        # 若把相机挂到运动连杆上，上层发布的静态 TF 不会随运动更新，需自行处理
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
        # 分割图通道 1 的"几何体"类型常量，用于把标注限制在几何体像素上
        self._geom_object_type = int(self._mj.mjtObj.mjOBJ_GEOM)
        self._body_geoms: dict[str, tuple[int, ...]] = {}
        for body_name in config.annotation_bodies:
            body_id = int(
                self._mj.mj_name2id(self._model, self._mj.mjtObj.mjOBJ_BODY, body_name)
            )
            if body_id < 0:
                raise ValueError(f"MuJoCo annotation body not found: {body_name}")
            # 预先建好 body 名 → geom id 列表的映射，渲染时每帧只查表
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
        """从上层仿真对象构造渲染器（取其底层 model/data 句柄）。

        必须在持有该仿真串行访问权的线程内调用；``_unsafe_viewer_handles`` 是对底层句柄的
        受控越界访问，这样可以直接使用正在推进的 model/data，而不必复制整个模型。
        """
        model, data = simulation._unsafe_viewer_handles()
        return cls(simulation._mj, model, data, config)

    def render(self) -> VirtualCameraFrame:
        """渲染一帧：依次渲染 RGB、深度、分割，再合成毫米深度与真值标注。

        三遍渲染各自都要 ``update_scene``：该调用把 data 中的最新位姿写入渲染场景，而切换
        渲染模式不会自动刷新场景。深度与分割是渲染器的全局开关，用 ``try/finally`` 保证
        异常时也复位，否则会污染后续的 RGB 输出。

        渲染器已关闭时抛 ``RuntimeError``。返回的数组都是独立副本，可安全交给其它线程。
        """
        if self._closed:
            raise RuntimeError("virtual camera renderer is closed")
        self._renderer.update_scene(self._data, camera=self.config.camera_name)
        # copy() 让像素数据脱离渲染器内部缓冲，避免下一帧渲染覆盖本帧结果
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

        # 只有几何体像素的深度可信：背景/天空的深度值不可用，掩掉后统一记为 0
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
        """销毁离屏渲染上下文；幂等，重复调用不会重复释放。"""
        if not self._closed:
            self._renderer.close()
            self._closed = True


class VirtualCameraWorker:
    """把渲染器的完整生命周期固定在一条专用线程上，对外提供非阻塞的取帧请求。

    构造即启动一条 daemon 渲染线程：线程内先经串行化访问器创建渲染器并回调 ``on_ready``
    （内参 + 外参），随后循环等待 ``submit`` 提交的时间戳，渲染一帧后回调 ``on_frame``。
    渲染线程内抛出的任何异常都交给 ``on_error``，不会静默退出，也不会影响物理仿真。

    背压策略：只保留最新一次请求（丢旧帧而不是排队），因此渲染慢于发布时表现为掉帧，延迟
    不会持续累积。

    线程模型：三个回调都在渲染线程上执行，实现方需保证线程安全（上层通常只是构造消息并
    发布），且不要在回调里阻塞等待，否则会拖住渲染循环。
    """

    def __init__(
        self,
        simulation_access: Any,
        config: VirtualCameraConfig,
        *,
        on_frame: Callable[[VirtualCameraFrame, PinholeIntrinsics, tuple[int, int]], None],
        on_ready: Callable[[PinholeIntrinsics, VirtualCameraExtrinsics], None],
        on_error: Callable[[BaseException], None],
    ) -> None:
        """保存配置与回调后立即启动渲染线程（构造返回时渲染器尚不一定就绪）。

        - ``simulation_access``：提供 ``run(operation)`` 的串行化访问器，所有对仿真
          model/data 的操作都必须经它执行，避免与物理步进并发；
        - ``on_frame``：每成功渲染一帧回调一次，参数为帧数据、内参、提交时的时间戳
          (sec, nanosec)；
        - ``on_ready``：渲染器创建成功后回调一次，参数为内参与外参，供上层发布 CameraInfo
          与静态 TF；
        - ``on_error``：渲染线程内出现异常时回调，参数为异常对象，供上层记日志；异常之后渲染
          线程即结束（``submit`` 随之返回 False），不会再产出新帧。
        """
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
        """登记一次渲染请求（非阻塞），返回请求是否被接受。

        ``stamp`` 为 (sec, nanosec)，原样回传给 ``on_frame`` 作为图像时间戳：秒必须非负、
        纳秒必须落在 [0, 1e9) 内，否则抛 ``ValueError``。渲染线程已停止或已退出时返回
        ``False``，调用方应据此停止继续提交。
        """
        seconds, nanoseconds = int(stamp[0]), int(stamp[1])
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise ValueError("virtual camera stamp is invalid")
        with self._condition:
            if self._stop_requested or not self._thread.is_alive():
                return False
            # 只保留最新请求：渲染慢于提交时丢旧帧，避免请求积压导致画面越来越滞后
            self._pending_stamp = (seconds, nanoseconds)
            self._condition.notify()
            return True

    def close(self, timeout_sec: float = 5.0) -> bool:
        """请求停止渲染线程并等待其退出。

        先清空待渲染请求再 ``join``，返回线程是否已在 ``timeout_sec`` 内退出；返回 ``False``
        表示超时（例如渲染器卡在图形调用里），调用方应记错误日志，而不是继续无限等待。
        """
        with self._condition:
            self._stop_requested = True
            self._pending_stamp = None
            self._condition.notify()
        self._thread.join(timeout=float(timeout_sec))
        return not self._thread.is_alive()

    def _run(self) -> None:
        """渲染线程主体：建渲染器 → 上报就绪 → 循环渲染，退出时销毁渲染器。

        渲染器必须在本线程创建、也必须在本线程销毁（``finally`` 中），否则会跨线程操作图形
        上下文。渲染与销毁都包在 ``simulation_access.run`` 内，保证与物理步进互斥。时间戳在
        释放条件锁之前取出，确保回调拿到的是与本帧对应的仿真时间。
        """
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
                    # 无请求且未要求停止时挂起；被 submit/close 唤醒后重新判断条件
                    self._condition.wait_for(
                        lambda: self._stop_requested or self._pending_stamp is not None
                    )
                    # 停止优先：即使同时存在新请求也直接退出，不再渲染
                    if self._stop_requested:
                        return
                    stamp = self._pending_stamp
                    self._pending_stamp = None
                # 渲染器已绑定 model/data 句柄，这里忽略传入的仿真对象，只借访问器取得互斥
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
