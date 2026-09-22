"""夹爪闭合的"接触"判据（纯函数，不接触硬件）。

背景：本站夹爪没有接触/力传感器，闭合指令"未到位"既可能是夹到物体（正常），
也可能是被线缆或机构卡住（异常）。本模块用"停在哪里 + 相对张开位置移动了多少"
两个可观测量推断接触，供执行节点在位置指令回报失败时决定是否仍判本次夹持成功。
"""

from __future__ import annotations


def close_contact_success(
    *,
    command_success: bool,
    target_position_m: float,
    reached_position_m: float,
    previous_open_position_m: float | None,
    contact_margin_m: float,
    min_closure_delta_m: float,
) -> bool:
    """判断"闭合指令未成功"时能否按"已夹到物体"放行。

    参数（关键字传参；位置单位统一为 m，指两指间距）：
    - ``command_success``：夹爪服务回报的指令是否成功。为真直接返回 True
      （正常闭合到位，无需推断接触）。
    - ``target_position_m``：本次闭合的目标位置。
    - ``reached_position_m``：实际停在的位置。
    - ``previous_open_position_m``：上一次张开阶段的实际到位位置，作为闭合行程的基准；
      为 ``None``（从未成功张开过）时无法计算行程，保守返回 False。
    - ``contact_margin_m``：允许停在目标外侧的余量；实际到位 >= 目标 + 该余量即认为
      是被目标挡住而非机械卡滞。
    - ``min_closure_delta_m``：相对张开位置的最小闭合行程，用来排除"几乎没动"的空夹。

    返回：两个条件同时成立才为 True——(1) 停得足够靠外（疑似被物体挡住），
    (2) 相对张开位置确实闭合了足够行程（确实夹到了东西）。

    安全语义：本函数只放宽"位置未到位"这一条判据，不放宽夹持力与夹爪行程限制；
    调用方还应叠加自身的开关（如执行节点的接触判据开关）才真正生效。
    """
    if command_success:
        return True
    if previous_open_position_m is None:
        return False

    target = float(target_position_m)
    reached = float(reached_position_m)
    previous_open = float(previous_open_position_m)

    # 实际到位比目标更靠外（数值更大）且超出余量 → 疑似被目标挡住
    closed_enough_to_contact = reached >= target + float(contact_margin_m)
    # 相对张开位置至少闭合了 min_closure_delta_m → 不是"原地没动"的空夹
    moved_from_open = previous_open - reached >= float(min_closure_delta_m)
    return closed_enough_to_contact and moved_from_open
