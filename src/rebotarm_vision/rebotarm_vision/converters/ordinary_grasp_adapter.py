"""旧版 ordinary_grasp 抓取算法与 ROS 抓取消息之间的适配器。

职责与位置
    本模块属于视觉包的 converters 层（纯算法转换，不持有任何 ROS 节点、话题、参数或
    定时器）。它把外部旧工程（由 ``ordinary_grasp_root`` 指定的 rebot_grasp 目录）里
    的普通抓取算法包装成抓取候选与抓取计划消息，供普通抓取节点调用，再由下游候选
    过滤、运动规划与执行门控消费。真实抓取必须经过规划、碰撞检查与执行门控，本模块
    只做几何换算与候选生成，不发布命令、不接触硬件。

调用链
    Detection2DArray（一帧 2D 检测）+ 深度图 + 相机内参
    -> 逐检测的深度质量门（剔除深度不可信目标）
    -> 旧算法 estimate_grasps 求取抓取
    -> 抓取位姿规范化（统一 TCP 接近轴）
    -> 打分排序（无效/超宽候选被丢弃）
    -> 返回 (GraspPlan 最优计划, GraspCandidateArray 全部候选)

坐标系与单位
    输入检测框为彩色图像像素坐标；输入的深度图元素为 uint16、单位 mm，0 表示无回波。
    内参为像素单位。输出的所有位姿都位于 ``output_frame_id`` 指定的相机深度光学坐标系
    下：位置单位 m，姿态为四元数 xyzw。候选的 ``jaw_width`` 与 ``object_length`` 单位 m。

约定与边界
    深度图缺失或尺寸为空时直接返回无效计划，绝不抛异常穿透到节点回调。
    ``best_index`` 按消息契约：列表非空为 0（已按得分降序），为空为 -1，使用前必须判负。
    计划无效时 ``reason`` 为英文短句（对外诊断接口，属字符串禁区，不得翻译）。

字符串禁区
    本文件对外的类别名、来源标识、失败原因（例如 "ordinary_grasp_obb_depth"、
    "missing depth image"）全部保持英文原样；docstring 中不书写未在本文件导入的
    包名字面量，需要指代时用中文描述。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import numpy as np
import cv2
from geometry_msgs.msg import Pose

from rebotarm_msgs.msg import Detection2D, Detection2DArray, GraspCandidate, GraspCandidateArray, GraspPlan

from ..depth_quality import DepthQualityConfig, evaluate_detection_depth_quality
from ..grasp_candidate_policy import GraspCandidateScoringConfig, score_grasp_candidate


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔相机内参，用于把像素坐标反投影到相机坐标系。

    fx/fy 为焦距（像素，通常取自 CameraInfo 的 K[0]/K[4]），cx/cy 为主点（像素，
    K[2]/K[5]）。本站彩色/深度图分辨率 640x480 时 cx、cy 分别约为 320、240。
    旧算法依靠该矩阵与深度图配合完成反投影，取值必须与产出检测框和深度图的那台
    相机一致，否则抓取点会整体偏移。
    """

    fx: float
    fy: float
    cx: float
    cy: float

    def as_matrix(self) -> np.ndarray:
        """返回 3x3 内参矩阵（float32），末行为 [0, 0, 1] 的齐次形式。"""

        return np.asarray(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )


@dataclass(frozen=True)
class OrdinaryGraspConfig:
    """普通抓取适配器的静态配置（节点参数与环境变量最终都映射到这里）。

    ordinary_grasp_root：旧算法工程根目录（绝对路径，需含 utils 子包）。
      为空字符串时节点会在启动阶段直接报错，因为无法导入算法。
    depth_quantile：深度采样分位数，取值 (0, 1)，默认 0.75。旧算法在目标掩膜/框内
      按该分位数取深度代表值：调大更靠近相机侧表面（偏保守、抓取点略靠前），调小
      则更靠近背景，过小可能把桌面或背景深度算进来。需与深度质量门的范围联动。
    output_frame_id：所有输出位姿的参考坐标系，默认相机深度光学系。必须与实际发布
      深度图的坐标系一致，否则下游 TF 变换会整体错位。
    pregrasp_offset_m：接近点沿 TCP 接近轴反方向的退让距离，单位 m，默认 0.08。
      调大更安全但空行程更长（节拍变慢），调小可能让末端在进给前已贴住目标或桌面。
    """

    ordinary_grasp_root: Path
    depth_quantile: float = 0.75
    output_frame_id: str = "camera_depth_frame"
    pregrasp_offset_m: float = 0.08


# ---------------------------------------------------------------------------
# 下面这组 _Boxes / _Obb / _Masks / _MaskData / _TensorLike / _YoloLikeResult
# 只为一个目的存在：模拟旧算法所期望的"YOLO 结果对象"协议（result.names、
# result.boxes.xyxy/cls/conf、result.obb.xywhr/xyxyxyxy、result.masks.data、
# result.orig_shape，以及 torch 张量式的索引、float()、numpy() 调用）。
# 这样旧工程代码可以零修改复用，而视觉包不必引入模型推理依赖，也不必依赖
# 旧工程内部的张量类型。它们的字段顺序与旧算法的读取顺序一一对应，不可调整。
# ---------------------------------------------------------------------------


class _Boxes:
    """检测框视图：按索引返回一行 (xyxy, cls, conf)，对齐旧算法的逐框访问习惯。"""

    def __init__(self, detections: list[Detection2D]) -> None:
        self._detections = detections

    def __len__(self) -> int:
        return len(self._detections)

    def __getitem__(self, index: int) -> Any:
        # 每次索引都构造一个匿名行对象，字段名与旧算法一致：
        # xyxy 为前景框 (x_min, y_min, x_max, y_max)（像素）；cls 这里用检测在数组中的
        # 下标充当类别索引，配合 _YoloLikeResult.names 映射回真实类别名；conf 为置信度。
        det = self._detections[index]
        return type(
            "BoxRow",
            (),
            {
                "xyxy": _TensorLike(
                    [[det.x_min, det.y_min, det.x_max, det.y_max]],
                ),
                "cls": _TensorLike([index]),
                "conf": _TensorLike([det.confidence]),
            },
        )()


class _Obb:
    """旋转框（OBB）视图：整批一次性堆叠成数组，字段与模型 xywhr 输出顺序一致。"""

    def __init__(self, detections: list[Detection2D]) -> None:
        self._detections = detections
        # cls 同样用下标当类别索引；conf 为检测置信度，均转为 float32 对齐旧算法。
        self.cls = np.asarray([index for index, _ in enumerate(detections)], dtype=np.float32)
        self.conf = np.asarray([det.confidence for det in detections], dtype=np.float32)
        # 轴对齐前景框，每行 (x_min, y_min, x_max, y_max)，单位 pixel。
        self.xyxy = np.asarray(
            [[det.x_min, det.y_min, det.x_max, det.y_max] for det in detections],
            dtype=np.float32,
        )
        # 旋转框参数化形式，每行 (cx, cy, w, h, theta)，单位 pixel / rad。
        self.xywhr = np.asarray(
            [
                [
                    det.obb_cx,
                    det.obb_cy,
                    det.obb_w,
                    det.obb_h,
                    det.obb_theta,
                ]
                for det in detections
            ],
            dtype=np.float32,
        )
        # 旋转框四角点，形状 (N, 4, 2)，由扁平的 obb_points_xy 重塑而来，单位 pixel。
        self.xyxyxyxy = np.asarray(
            [
                np.asarray(det.obb_points_xy, dtype=np.float32).reshape(4, 2)
                for det in detections
            ],
            dtype=np.float32,
        )

    def __len__(self) -> int:
        return len(self._detections)


class _YoloLikeResult:
    """单帧检测结果，模仿旧算法接收的模型输出对象。

    与旧算法行为相关的两个细节：
    - ``obb`` 仅当**所有**检测都带有效旋转框时才挂载，否则为 None：旧算法会据此退化到
      轴对齐框分支，避免混用两种框型导致旋转框与前景框索引错位。
    - ``masks`` 只要有**任一**检测带有效掩膜就挂载，掩膜数量与检测数量保持一一对应
      （无掩膜的检测填全零掩膜，见 _Masks），旧算法按同一索引取框和掩膜。
    """

    def __init__(self, detections: list[Detection2D], image_shape: tuple[int, int]) -> None:
        # 类别索引 -> 类别名；索引来源见 _Boxes/_Obb 的 cls 字段。
        self.names = {index: det.class_name for index, det in enumerate(detections)}
        self.boxes = _Boxes(detections)
        self.obb = _Obb(detections) if all(_has_valid_obb(det) for det in detections) else None
        self.masks = _Masks(detections, image_shape) if any(_has_valid_mask(det) for det in detections) else None
        # 原始图像尺寸 (高, 宽)，单位 pixel，供旧算法做尺度还原。
        self.orig_shape = image_shape


class _Masks:
    """实例掩膜视图：把多边形顶点栅格化成与图像同尺寸的二值掩膜。"""

    def __init__(self, detections: list[Detection2D], image_shape: tuple[int, int]) -> None:
        height, width = image_shape
        masks = []
        for det in detections:
            # 掩膜用 float32 的 0/1 表示（而非 bool），以匹配旧算法对张量 dtype 的预期。
            mask = np.zeros((height, width), dtype=np.float32)
            if _has_valid_mask(det):
                polygon = np.asarray(det.mask_polygon_xy, dtype=np.float32).reshape(-1, 2)
                # 顶点先四舍五入到整数像素再填充，避免 fillPoly 的坐标取整歧义。
                cv2.fillPoly(mask, [np.round(polygon).astype(np.int32)], 1.0)
            masks.append(mask)
        self.data = _MaskData(masks)


class _MaskData:
    """掩膜集合视图：支持 len() 与按下标取单张掩膜（返回 _TensorLike 包装）。"""

    def __init__(self, masks: list[np.ndarray]) -> None:
        self._masks = masks

    def __len__(self) -> int:
        return len(self._masks)

    def __getitem__(self, index: int):
        return _TensorLike(self._masks[index])


class _TensorLike:
    """极简"仿张量"包装：只实现旧算法实际用到的那几个接口。

    旧算法的代码写成 ``boxes.xyxy[0].cpu().numpy()``、``int(box.cls)`` 这种深度学习
    框架的风格，这里用 numpy 提供同形接口，避免为了适配而引入重量级依赖。
    支持的调用：链式下标、np.asarray/float()/int()、detach()/cpu() 空操作、numpy()。
    """

    def __init__(self, values) -> None:
        self._values = np.asarray(values, dtype=np.float32)

    def __getitem__(self, index):
        return _TensorLike(self._values[index])

    def __array__(self, dtype=None):
        # 让 np.asarray()/float() 之外的隐式转换也能工作；dtype 为 None 时原样返回。
        if dtype is None:
            return self._values
        return self._values.astype(dtype)

    def __float__(self) -> float:
        # 标量语义：包装的可能是 [[v]] 或 [v] 这类带外层维度的数组，统一拍平取首元素。
        return float(self._values.reshape(-1)[0])

    def __int__(self) -> int:
        return int(float(self))

    def detach(self):
        # 无梯度概念，直接返回自身以保持链式调用可用。
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._values


def _has_valid_obb(det: Detection2D) -> bool:
    """检测是否带可用旋转框：标志位为真且角点恰好 4 个点（展开 8 个数）。"""
    return bool(det.has_obb and len(det.obb_points_xy) == 8)


def _has_valid_mask(det: Detection2D) -> bool:
    """检测是否带可用掩膜多边形：至少有 3 个顶点（6 个数）才能构成有效多边形。

    用 getattr 做兼容读取，是因为上游消息可能来自旧版本或测试替身，缺少这两个字段；
    缺字段时按"无掩膜"处理，不影响轴对齐框流程。
    """
    return bool(getattr(det, "has_mask", False) and len(getattr(det, "mask_polygon_xy", [])) >= 6)


def _import_ordinary_grasp(root: Path):
    """按需导入外部旧工程提供的普通抓取算法，返回三个可调用对象。

    参数 root：旧工程根目录（绝对路径），其下需有 utils.ordinary_grasp 与 utils.transforms。
    返回 (estimate_grasps, select_best_grasp, canonicalize_parallel_gripper_tcp_rotation)。

    副作用：会把 root 插入到 sys.path 首位（仅在尚未存在时），因此本模块的导入路径
    优先级高于同名第三方模块；这是复用旧工程的既定做法，重复调用不会重复插入。
    """
    root = Path(root).resolve()
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    # 必须在插入 sys.path 之后再导入，且放在函数内部：未启用普通抓取时不应因缺少
    # 旧工程而让本模块导入失败。
    from utils.ordinary_grasp import estimate_grasps, select_best_grasp
    from utils.transforms import canonicalize_parallel_gripper_tcp_rotation

    return estimate_grasps, select_best_grasp, canonicalize_parallel_gripper_tcp_rotation


def _quaternion_from_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """把 3x3 旋转矩阵转换为单位四元数，返回 (x, y, z, w)。

    采用标准的分支算法（Shepperd 方法）：先看迹 trace 是否为正，为正时用迹的公式；
    否则取对角线上最大的分量作为主分量再求解，避免接近 180° 旋转时分母趋零造成精度
    崩溃。各分支里的 1e-12 是防止 sqrt 参数为负或为零的兜底下限。
    最后按 L2 范数归一化，消除浮点舍入带来的模长偏差（下游按单位四元数解读姿态）。
    """
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (matrix[2, 1] - matrix[1, 2]) * s
        qy = (matrix[0, 2] - matrix[2, 0]) * s
        qz = (matrix[1, 0] - matrix[0, 1]) * s
    else:
        # 主分量分支：idx 为对角线上最大元素的行列号（0=x, 1=y, 2=z）。
        idx = int(np.argmax(np.diag(matrix)))
        if idx == 0:
            s = 2.0 * np.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 1e-12))
            qw = (matrix[2, 1] - matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (matrix[0, 1] + matrix[1, 0]) / s
            qz = (matrix[0, 2] + matrix[2, 0]) / s
        elif idx == 1:
            s = 2.0 * np.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 1e-12))
            qw = (matrix[0, 2] - matrix[2, 0]) / s
            qx = (matrix[0, 1] + matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 1e-12))
            qw = (matrix[1, 0] - matrix[0, 1]) / s
            qx = (matrix[0, 2] + matrix[2, 0]) / s
            qy = (matrix[1, 2] + matrix[2, 1]) / s
            qz = 0.25 * s
    # 与 ROS 消息一致的排列顺序是 (x, y, z, w)，此处按该顺序组装后再归一化。
    quat = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-12)
    return tuple(float(value) for value in quat)


def _pose_from_position_rotation(position: np.ndarray, rotation: np.ndarray) -> Pose:
    """位置(m) + 旋转矩阵 -> Pose 消息。

    位置保留 6 位小数：一是抑制反投影的浮点尾差让输出可复现，二是 1e-6 m = 1 微米，
    远小于机械臂重复定位精度，不会损失有效信息。
    """
    qx, qy, qz, qw = _quaternion_from_matrix(rotation)
    pose = Pose()
    pose.position.x = round(float(position[0]), 6)
    pose.position.y = round(float(position[1]), 6)
    pose.position.z = round(float(position[2]), 6)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _empty_plan(frame_id: str, reason: str) -> GraspPlan:
    """构造无效计划：valid=False、source 固定为 ordinary_grasp、reason 说明失败原因。

    这是一个统一出口——所有失败分支（缺深度、无检测、深度质量全被拒、无有效候选）都
    走这里，保证下游只需判 valid 与读 reason，不必区分失败来源。
    """
    plan = GraspPlan()
    plan.header.frame_id = frame_id
    plan.valid = False
    plan.source = "ordinary_grasp"
    plan.reason = reason
    return plan


def _source_name(*, has_obb_input: bool, has_mask_input: bool) -> str:
    """按输入信息的丰富程度给出候选来源标识，用于排障溯源（字符串对外，不可翻译）。

    优先级：旋转框 > 实例掩膜 > 仅轴对齐框。三者的抓取点估计精度依次下降，下游可据此
    判断本次结果的可信度。
    """
    if has_obb_input:
        return "ordinary_grasp_obb_depth"
    if has_mask_input:
        return "ordinary_grasp_mask_depth"
    return "ordinary_grasp_bbox_depth"


def _candidate_from_grasp(grasp, output_frame_id: str, source: str) -> GraspCandidate:
    """把旧算法的单个抓取对象转换为抓取候选消息。

    字段映射：class_name/conf 来自检测，position(m) + tcp_rotation 组成位姿，
    jaw_width_m 为夹爪开口宽度(m)，object_length_m 为目标长度(m)。
    valid 固定为 True——不可用的抓取已在打分阶段被丢弃，能到这里就是可用候选。
    """
    candidate = GraspCandidate()
    candidate.header.frame_id = output_frame_id
    candidate.class_name = str(grasp.class_name)
    candidate.confidence = float(grasp.conf)
    candidate.pose = _pose_from_position_rotation(
        np.asarray(grasp.position, dtype=np.float64),
        np.asarray(grasp.tcp_rotation, dtype=np.float64),
    )
    candidate.jaw_width = float(grasp.jaw_width_m)
    candidate.object_length = float(grasp.object_length_m)
    candidate.valid = True
    candidate.source = source
    return candidate


def build_candidate_array_from_grasps(
    grasps: list,
    *,
    output_frame_id: str,
    source: str,
    scoring_config: GraspCandidateScoringConfig | None = None,
) -> GraspCandidateArray:
    """把旧算法的抓取列表转成候选数组消息，按得分降序排序。

    grasps：旧算法返回的抓取对象序列，需带 conf、jaw_width_m、is_valid 属性；
      字段缺失按 0/False 处理（getattr 默认值），因此上游模型字段不全也不会抛异常。
    output_frame_id：所有候选位姿的参考坐标系（相机深度光学系）。
    source：来源标识，由 _source_name 生成，写入每个候选。
    scoring_config：打分配置，None 时使用打分模块的默认配置。

    打分规则见 score_grasp_candidate：返回负分表示该候选不可用（无效、置信度低于阈值、
    夹爪宽度为 0 或超过夹爪行程上限），这类候选被直接丢弃而不进入数组。
    排序使用 list.sort（稳定排序，同分时保留旧算法给出的原顺序），并据此把 best_index
    置为 0；没有候选时置 -1 以符合消息契约。
    """
    scored: list[tuple[float, GraspCandidate]] = []
    for grasp in grasps:
        score = score_grasp_candidate(
            confidence=float(getattr(grasp, "conf", 0.0) or 0.0),
            jaw_width_m=float(getattr(grasp, "jaw_width_m", 0.0) or 0.0),
            valid=bool(getattr(grasp, "is_valid", False)),
            config=scoring_config,
        )
        # 约定：负分即"不可用"，在此处丢弃，避免无效候选流向下游运动侧。
        if score < 0.0:
            continue
        candidate = _candidate_from_grasp(grasp, output_frame_id, source)
        scored.append((score, candidate))

    scored.sort(key=lambda item: item[0], reverse=True)
    array = GraspCandidateArray()
    array.header.frame_id = output_frame_id
    array.candidates = [candidate for _, candidate in scored]
    array.best_index = 0 if array.candidates else -1
    return array


def _plan_from_best_candidate(
    *,
    detections: Detection2DArray,
    candidates: GraspCandidateArray,
    pregrasp_offset_m: float,
    reason_if_empty: str,
) -> GraspPlan:
    """由候选数组中的最优候选补出接近点，组装抓取计划。

    detections：原始检测数组，仅用于沿用其 header.stamp（时间基准与检测帧一致）；
      frame_id 一律以 candidates.header.frame_id 为准，二者不一致时以候选为准。
    pregrasp_offset_m：接近点沿 TCP 接近轴反方向的退让距离，单位 m。
    reason_if_empty：无候选时写入计划 reason 的英文短句（对外诊断接口）。

    接近点构造：候选姿态的四元数 -> 旋转矩阵，取第 1 列即 TCP 坐标系的 x 轴作为接近
    轴（该轴由旧算法经规范化处理，指向"从夹爪指向目标"的前进方向），抓取点沿该轴反
    向平移 offset 得到接近点位置。因此从接近点沿 +x 前进 offset 即到达抓取点。
    接近点姿态直接复用候选姿态，避免额外一次矩阵->四元数换算引入姿态跳变。
    """
    if candidates.best_index < 0 or not candidates.candidates:
        return _empty_plan(candidates.header.frame_id, reason_if_empty)
    # best_index 已按消息契约校验过非负，此处索引安全。
    best_candidate = candidates.candidates[int(candidates.best_index)]
    tcp_rotation = np.eye(3, dtype=np.float64)
    # Recover pregrasp direction from the candidate quaternion through the same canonical TCP x-axis.
    q = (
        float(best_candidate.pose.orientation.x),
        float(best_candidate.pose.orientation.y),
        float(best_candidate.pose.orientation.z),
        float(best_candidate.pose.orientation.w),
    )
    # 局部导入：把点变换工具留在真正需要它的路径上，同时避免与工具模块形成导入环。
    from ..transform_points import quaternion_to_rotation_matrix

    tcp_rotation_list = quaternion_to_rotation_matrix(q)
    # 旋转矩阵按行存储，第 1 列 (R[0][0], R[1][0], R[2][0]) 才是 TCP 的 x 轴方向。
    tcp_x = np.asarray([tcp_rotation_list[0][0], tcp_rotation_list[1][0], tcp_rotation_list[2][0]], dtype=np.float64)
    grasp_position = np.asarray(
        [
            float(best_candidate.pose.position.x),
            float(best_candidate.pose.position.y),
            float(best_candidate.pose.position.z),
        ],
        dtype=np.float64,
    )
    pregrasp_position = grasp_position - tcp_x * float(pregrasp_offset_m)
    plan = GraspPlan()
    # 先继承检测的 header 以带上 stamp，再覆盖 frame_id 与候选保持一致。
    plan.header = detections.header
    plan.header.frame_id = candidates.header.frame_id
    plan.candidate = best_candidate
    plan.grasp_pose = best_candidate.pose
    plan.pregrasp_pose = _pose_from_position_rotation(pregrasp_position, np.asarray(tcp_rotation, dtype=np.float64))
    # 姿态以候选姿态为准（上面的旋转矩阵参数只用于算位置），保证与 grasp_pose 完全一致。
    plan.pregrasp_pose.orientation = best_candidate.pose.orientation
    plan.jaw_width = float(best_candidate.jaw_width)
    plan.valid = True
    plan.source = "ordinary_grasp"
    # 计划有效时 reason 按契约留空串。
    plan.reason = ""
    return plan


def plan_and_candidates_from_detections_and_depth(
    detections: Detection2DArray,
    depth_mm: np.ndarray,
    intrinsics: CameraIntrinsics,
    ordinary_grasp_root: str | Path,
    output_frame_id: str = "camera_depth_frame",
    depth_quantile: float = 0.75,
    pregrasp_offset_m: float = 0.08,
    scoring_config: GraspCandidateScoringConfig | None = None,
    depth_quality_config: DepthQualityConfig | None = None,
) -> tuple[GraspPlan, GraspCandidateArray]:
    """主入口：由一帧检测 + 深度图产出抓取计划与全部候选。

    参数
        detections：一帧 2D 检测（帧内所有目标）。
        depth_mm：与彩色图对齐的深度图，uint16，单位 mm；元素 0 表示该像素无有效回波。
        intrinsics：相机内参，须与产出该深度图/检测框的相机一致（见 CameraIntrinsics）。
        ordinary_grasp_root：旧算法工程根目录，用于定位外部算法实现。
        output_frame_id：输出位姿的参考坐标系。
        depth_quantile：深度采样分位数，取值 (0, 1)，默认 0.75（见 OrdinaryGraspConfig）。
        pregrasp_offset_m：接近点退让距离，单位 m，默认 0.08。
        scoring_config：候选打分配置，None 用默认值（夹爪行程上限等在此约束）。
        depth_quality_config：深度质量门配置，None 用默认值。

    返回
        (GraspPlan, GraspCandidateArray)。计划无效时仍会返回（可能为空的）候选数组，
        便于调用方一并发布或调试；候选数组的 best_index 在无候选时为 -1。

    安全语义
        本函数只做几何换算与候选生成，不下发运动指令。深度质量门在此处先于抓取求解
        执行：深度缺失、像素太少、偏远处（超出配置范围）、抖动过大或中心区域无深度的
        目标会被剔除，因为这类目标的抓取点可能落到背景或悬空位置。剔除结果全部记入
        计划 reason，便于现场判断是"没检到目标"还是"深度不可信"。
    """
    if depth_mm is None or depth_mm.size == 0:
        # 深度图缺失时不能反投影，直接给出无效计划与空候选（best_index = -1）。
        empty = GraspCandidateArray()
        empty.header.frame_id = output_frame_id
        empty.best_index = -1
        return _empty_plan(output_frame_id, "missing depth image"), empty
    depth_quality = depth_quality_config or DepthQualityConfig()
    filtered = []
    rejected_reasons: list[str] = []
    for det in detections.detections:
        # 置信度非正视为无效检测，连质量评估都不做（上游一般已过滤，这里做兜底）。
        if det.confidence <= 0.0:
            continue
        quality = evaluate_detection_depth_quality(det, depth_mm, depth_quality)
        if quality.accepted:
            filtered.append(det)
        else:
            # 记录类名 + 原因码 + 关键指标：现场可据此判断是阈值过严还是传感器确实没数据。
            class_name = str(getattr(det, "class_name", "") or "object")
            rejected_reasons.append(
                f"{class_name}:{quality.reason}"
                f"(valid={quality.valid_depth_pixels}, ratio={quality.valid_depth_ratio:.2f}, "
                f"z={quality.z_median_m:.3f}, mad={quality.z_mad_m:.3f})"
            )
    if not filtered:
        empty = GraspCandidateArray()
        empty.header.frame_id = output_frame_id
        empty.best_index = -1
        # 两种情况区分开：有被拒记录说明深度质量门起作用，否则是本帧根本没有可用检测。
        if rejected_reasons:
            return _empty_plan(
                output_frame_id,
                "depth quality rejected all detections: " + "; ".join(rejected_reasons),
            ), empty
        return _empty_plan(output_frame_id, "no detections"), empty

    # 只要有检测带旋转框/掩膜，就按更精细的输入路径处理；来源标识据此决定。
    has_obb_input = any(_has_valid_obb(det) for det in filtered)
    has_mask_input = any(_has_valid_mask(det) for det in filtered)
    # 三个返回值中第二个（择优函数）不使用：本适配器改用打分配置统一排序，避免两套
    # 择优标准不一致。
    estimate_grasps, _select_best_grasp, canonicalize_tcp_rotation = _import_ordinary_grasp(
        Path(ordinary_grasp_root)
    )
    result = _YoloLikeResult(filtered, depth_mm.shape[:2])
    raw_grasps = estimate_grasps([result], depth_mm, intrinsics.as_matrix(), depth_quantile=depth_quantile)
    grasps = []
    for grasp in raw_grasps:
        if getattr(grasp, "is_valid", False):
            # 规范化平行夹爪 TCP 旋转：把等价的多种姿态收敛到同一朝向约定，保证本模块
            # 按 TCP x 轴取接近方向时结果稳定（同一目标多次求解不会左右翻转）。
            grasp.tcp_rotation = canonicalize_tcp_rotation(grasp.tcp_rotation)
        grasps.append(grasp)

    candidates = build_candidate_array_from_grasps(
        grasps,
        output_frame_id=output_frame_id,
        source=_source_name(has_obb_input=has_obb_input, has_mask_input=has_mask_input),
        scoring_config=scoring_config,
    )
    plan = _plan_from_best_candidate(
        detections=detections,
        candidates=candidates,
        pregrasp_offset_m=pregrasp_offset_m,
        reason_if_empty="ordinary grasp found no valid candidate",
    )
    return plan, candidates


def plan_from_detections_and_depth(
    detections: Detection2DArray,
    depth_mm: np.ndarray,
    intrinsics: CameraIntrinsics,
    ordinary_grasp_root: str | Path,
    output_frame_id: str = "camera_depth_frame",
    depth_quantile: float = 0.75,
    pregrasp_offset_m: float = 0.08,
) -> GraspPlan:
    """兼容入口：只返回抓取计划，丢弃候选数组。

    内部委托给 plan_and_candidates_from_detections_and_depth，不额外传打分与深度质量
    配置（一律用默认值）。需要候选列表或需要自定义阈值时请直接调用上面那个函数。
    """
    plan, _candidates = plan_and_candidates_from_detections_and_depth(
        detections,
        depth_mm,
        intrinsics,
        ordinary_grasp_root=ordinary_grasp_root,
        output_frame_id=output_frame_id,
        depth_quantile=depth_quantile,
        pregrasp_offset_m=pregrasp_offset_m,
    )
    return plan
