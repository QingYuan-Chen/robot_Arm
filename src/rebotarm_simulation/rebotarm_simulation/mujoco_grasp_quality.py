"""抓取质量判定：把"碰到了"与"真的抬起来了"分开评估。

职责与位置
    纯逻辑模块，属于仿真包的离线抓取评估能力：输入一次抓取试验的接触计数与箱体
    前后高度，输出结构化判定结果。不做物理计算、不读写文件、不依赖 ROS，便于离线
    基准脚本与单元测试直接调用。

判定规则
    ``contact_detected``  接触点计数 > 0，只说明夹爪碰到了物体；
    ``lift_detected``     末态高度 − 初态高度 ≥ ``min_lift_m``，说明物体确实被抬离台面；
    ``success``           两者同时成立才算抓取成功。
    之所以必须区分：真机上"碰到但没抬起"通常意味着夹持力不足或物体打滑，把它
    报成成功会掩盖真实的抓取缺陷。

状态取值
    ``grasp_lift_success``（接触且抬起）、``contact_without_lift``（仅接触）、
    ``no_contact``（未接触）；这些字符串是对外证据字段，不可翻译或改名。

单位约定
    高度与抬升量单位为米（m），接触计数为无量纲整数。
"""

from __future__ import annotations

from dataclasses import dataclass

@dataclass(frozen=True)
class GraspQuality:
    """一次抓取试验的质量判定结果。"""

    contact_detected: bool  # 是否检测到接触（接触点计数 > 0）
    lift_detected: bool  # 抬升量是否达到最小阈值
    lift_height_m: float  # 实测抬升量 = 末态高度 − 初态高度（m），可为负
    success: bool  # 最终判定：接触且抬起
    status: str  # 状态字符串，见模块 docstring 的三种取值


def evaluate_grasp_quality(
    *,
    contact_count: int,
    initial_box_height_m: float | None,
    final_box_height_m: float | None,
    min_lift_m: float = 0.03,
) -> GraspQuality:
    """根据接触计数与箱体高度变化判定抓取是否成功。

    参数：
        contact_count: 抓取过程中的接触点计数（无量纲）；> 0 视为有接触。
        initial_box_height_m: 试验前箱体高度（m）；缺失（None）时无法判定抬升。
        final_box_height_m: 试验后箱体高度（m）；缺失（None）时无法判定抬升。
        min_lift_m: 判定"抬起"所需的最小抬升量（m），默认 0.03 m（3 cm）——
            低于该值可能只是接触压紧或测量噪声，不足以证明物体离台。

    返回：
        ``GraspQuality``。高度缺失时 ``lift_height_m`` 保持 0.0、``lift_detected``
        为 False：缺少证据时按未抬起处理，不猜测成功。
    """
    contact_detected = int(contact_count) > 0
    lift_height = 0.0
    if initial_box_height_m is not None and final_box_height_m is not None:
        lift_height = float(final_box_height_m) - float(initial_box_height_m)
    lift_detected = lift_height >= float(min_lift_m)
    if contact_detected and lift_detected:
        status = "grasp_lift_success"
    elif contact_detected:
        status = "contact_without_lift"
    else:
        status = "no_contact"
    return GraspQuality(
        contact_detected=contact_detected,
        lift_detected=lift_detected,
        lift_height_m=lift_height,
        success=contact_detected and lift_detected,
        status=status,
    )
