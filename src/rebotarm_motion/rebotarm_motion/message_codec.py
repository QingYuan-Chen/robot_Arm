"""预览命令与运行状态的 JSON 编解码（运动包对外的文本协议适配层）。

用途
    预览/执行链路的请求方与服务方之间用一段 JSON 文本传递"关节或位姿预览命令"，以及"当前
    模式与状态"。本模块是纯函数式的编解码工具：不依赖任何中间件类型，不做校验与业务判断，
    因此节点与单元测试都可以直接调用。

报文格式（键名与取值是对外契约，改名即破坏兼容性）
    ``state``           随命令一起回传的状态字符串（取值见 command_models 里的执行状态枚举）；
    ``command_type``    命令类型字符串：``"joint"`` 关节空间 / ``"pose"`` 笛卡尔位姿空间；
    ``reachable``       预览结果是否可达（布尔）；
    ``message``         供人阅读的说明文本，为固定英文常量，不翻译；
    ``joint_names``     关节名列表，其顺序即 ``joint_positions`` 的顺序；
    ``joint_positions`` 关节角（rad），与 ``joint_names`` 一一对应；
    ``pose_target``     可选字段；存在时是含 ``x``/``y``/``z``（m）与 ``roll``/``pitch``/
                        ``yaw``（rad）的子字典。
    :func:`encode_status` 则使用 ``mode``/``state``/``message`` 三个键。

编解码约定
    编码统一使用 ``separators=(",", ":")`` 生成无多余空格的紧凑文本，减少话题带宽占用；
    解码遇到任何异常（非法 JSON、缺字段、数值无法转成 float 等）都吞掉并返回 ``(None, None)``，
    由调用方决定是丢弃该帧还是上报错误。
"""

from __future__ import annotations

import json

from .command_models import PoseTarget, PreviewCommand


def encode_preview_command(preview: PreviewCommand, *, state: str) -> str:
    """把预览命令编码成紧凑 JSON 文本。

    ``state`` 是随命令一起下发的状态字符串（关键字参数，必填）。关节名与关节角就地转成
    普通 list（原对象是 tuple，JSON 没有元组类型）。``pose_target`` 为 None 时整个键直接省略，
    而不是写成 ``null``——解码端据此区分"关节预览"与"位姿预览"两种命令。
    """
    # 固定字段顺序：state 在前便于排查，其余顺序与 ``PreviewCommand`` 的字段声明一致。
    payload = {
        "state": state,
        "command_type": preview.command_type,
        "reachable": bool(preview.reachable),
        "message": preview.message,
        "joint_names": list(preview.joint_names),
        "joint_positions": list(preview.joint_positions),
    }
    if preview.pose_target is not None:
        # 位姿子字典只在确实有位姿目标时出现，键名与解码端逐一对齐。
        payload["pose_target"] = {
            "x": preview.pose_target.x,
            "y": preview.pose_target.y,
            "z": preview.pose_target.z,
            "roll": preview.pose_target.roll,
            "pitch": preview.pose_target.pitch,
            "yaw": preview.pose_target.yaw,
        }
    return json.dumps(payload, separators=(",", ":"))


def decode_preview_command(payload: str) -> tuple[str | None, PreviewCommand | None]:
    """把 JSON 文本还原成 ``(state, PreviewCommand)``。

    解码失败（非法 JSON、缺字段、数值无法转成 float）一律返回 ``(None, None)`` 而不抛异常，
    调用方可把它当作"无效帧"直接丢弃；成功时 ``state`` 与命令一起返回，避免调用方二次解析。

    注意两处容易误读的边界：
    1. ``state`` 用 ``str(data.get("state"))`` 取值，键缺失时得到的不是 None 而是字符串
       ``"None"``（非字符串取值也会被字符串化），因此不能只用"是否为 None"判断该键是否存在；
    2. ``pose_target`` 只有在确实是 dict 时才构造，缺省或类型不符时保持 None。
    """
    try:
        data = json.loads(payload)
        pose_target = None
        raw_pose_target = data.get("pose_target")
        if isinstance(raw_pose_target, dict):
            # 六个分量全部取出并要求可转 float；任一缺失或非数值都会走进 except 分支。
            pose_target = PoseTarget(
                x=float(raw_pose_target["x"]),
                y=float(raw_pose_target["y"]),
                z=float(raw_pose_target["z"]),
                roll=float(raw_pose_target["roll"]),
                pitch=float(raw_pose_target["pitch"]),
                yaw=float(raw_pose_target["yaw"]),
            )
        preview = PreviewCommand(
            command_type=str(data["command_type"]),
            reachable=bool(data["reachable"]),
            message=str(data["message"]),
            joint_names=tuple(str(v) for v in data["joint_names"]),
            joint_positions=tuple(float(v) for v in data["joint_positions"]),
            pose_target=pose_target,
        )
        return str(data.get("state")), preview
    except Exception:
        # 捕获范围刻意放宽（含 KeyError/TypeError/ValueError/JSONDecodeError）：解码是边界入口，
        # 任何形态异常都应转成"无效帧"返回，不能让畸形报文打断调用方的回调线程。
        return None, None


def encode_status(*, mode: str, state: str, message: str) -> str:
    """把 ``mode``/``state``/``message`` 编码成紧凑 JSON 状态文本。

    三个字段都是关键字参数：``mode`` 是控制模式字符串（仿真/真实），``state`` 是当前执行状态，
    ``message`` 是固定英文说明文本（不翻译，直接对外展示）。
    """
    return json.dumps(
        {
            "mode": mode,
            "state": state,
            "message": message,
        },
        separators=(",", ":"),
    )
