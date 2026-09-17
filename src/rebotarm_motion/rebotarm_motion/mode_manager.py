"""控制模式字符串的唯一解析入口（配置/界面输入到枚举的适配层）。

外部输入（参数文件、界面请求、环境变量）只能给出字符串，本模块负责把"仿真/真实"这类写法
规范化后映射成 ``ControlMode`` 枚举。解析失败直接抛异常而不返回默认值：控制模式决定后续
命令落到仿真后端还是真实机械臂，任何拼写错误都必须显式暴露，绝不能被静默降级到任一分支。
"""

from __future__ import annotations

from .command_models import ControlMode


def parse_control_mode(value: str) -> ControlMode:
    """把字符串解析成 ``ControlMode``；无法识别时抛 ``ValueError``。

    大小写与首尾空白不敏感（``" Real "`` 同样合法），比较基准是枚举的 ``value``（"simulation"
    / "real"），因此这两个枚举值属于对外契约，不能改动。异常消息保留英文并原样带上入参，
    便于定位是哪个配置项写错了。
    """
    # 先规范化再比较，避免 "REAL"、空格、换行等写法被误判为不支持的模式。
    normalized = value.strip().lower()
    if normalized == ControlMode.SIMULATION.value:
        return ControlMode.SIMULATION
    if normalized == ControlMode.REAL.value:
        return ControlMode.REAL
    raise ValueError(f"unsupported control mode: {value}")
