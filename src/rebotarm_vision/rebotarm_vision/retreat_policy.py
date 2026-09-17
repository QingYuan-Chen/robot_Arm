"""抓取后的“抬升 / 安全后退”位姿策略（纯几何计算，无副作用）。

视觉抓取在夹紧物体之后必须先抬升脱离支撑面，再沿指定方向后退，才能进入
放置流程或回安全位。本模块只负责把这两个位姿算出来，不做运动规划、碰撞
检查或任何安全门判断——真正的执行仍由上层执行节点按阶段校验后下发。

坐标与单位约定：位置是三维平移，单位米；姿态为四元数 ``(x, y, z, w)``；
坐标系与传入的抓取位姿一致（通常是机械臂基座系，由上游统一决定）。

对外接口：``RetreatPolicyConfig``、``build_lift_pose``、``build_retreat_pose``。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .visual_grasp_sequence import PoseTarget


@dataclass(frozen=True)
class RetreatPolicyConfig:
    """安全抬升与后退的参数集合。

    这里的默认值是模块级兜底值；上层执行节点会用同名语义的运行期参数
    ``safe_retreat_enabled``、``safe_retreat_min_lift_z_m``、
    ``safe_retreat_distance_m``、``safe_retreat_axis_xyz`` 逐项覆盖它们。
    """

    # 总开关：为 False 时上层既不抬高抬升下限，也不追加 safe_retreat 阶段。
    enabled: bool = False
    # 抬升后的绝对高度下限，单位米（基座系 z 轴向上）。抬升目标取
    # max(抓取点 z + 实际抬升量, 本下限)，避免贴地抓取时抬升量偏小。
    min_lift_z_m: float = 0.22
    # 后退距离，单位米。越大离物体越远；过大可能超出工作空间或撞到后方障碍。
    retreat_distance_m: float = 0.06
    # 后退方向向量，基座系 x/y/z 分量（无量纲）。内部会归一化，因此只有方向
    # 有意义、模长被忽略；默认 (-1, 0, 0.5) 表示向 -x 方向并略向上后退。
    retreat_axis_xyz: tuple[float, float, float] = (-1.0, 0.0, 0.5)


def _normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """把三维向量归一化为单位向量；零向量属于配置错误，直接抛 ValueError。"""

    x, y, z = (float(vector[0]), float(vector[1]), float(vector[2]))
    norm = (x * x + y * y + z * z) ** 0.5
    # 1e-9 是浮点零判定阈值：模长低于它时无法可靠归一化，宁可报错也不返回 NaN。
    if norm <= 1e-9:
        raise ValueError("retreat_axis_xyz must be non-zero")
    return (x / norm, y / norm, z / norm)


def build_lift_pose(grasp: PoseTarget, *, lift_z_m: float, min_lift_z_m: float = 0.0) -> PoseTarget:
    """由抓取位姿构造“抬升”位姿：只改 z，x/y 与姿态保持不变。

    lift_z_m 是相对抓取点的抬升量（米，通常来自 ``lift_z_m`` 参数）；
    min_lift_z_m 是抬升后的绝对高度下限（米，未启用安全后退时传 0）。
    函数内延迟导入位姿类型，避免与序列模块形成模块级循环依赖。
    """

    from .visual_grasp_sequence import PoseTarget

    lift_z = max(float(grasp.position[2]) + float(lift_z_m), float(min_lift_z_m))
    return PoseTarget(
        position=(
            float(grasp.position[0]),
            float(grasp.position[1]),
            lift_z,
        ),
        orientation=grasp.orientation,
    )


def build_retreat_pose(lift: PoseTarget, config: RetreatPolicyConfig) -> PoseTarget:
    """由抬升位姿沿配置方向后退一段距离，姿态保持不变。

    距离取 max(retreat_distance_m, 0)：负值按 0 处理，即原地不动而不是反向
    扎回物体。返回位姿与输入同坐标系（米）。
    """

    from .visual_grasp_sequence import PoseTarget

    axis = _normalize_vector(config.retreat_axis_xyz)
    distance = max(float(config.retreat_distance_m), 0.0)
    return PoseTarget(
        position=(
            float(lift.position[0]) + axis[0] * distance,
            float(lift.position[1]) + axis[1] * distance,
            float(lift.position[2]) + axis[2] * distance,
        ),
        orientation=lift.orientation,
    )
