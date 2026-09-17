"""纯演练（dry run）命令路由表。

本模块只做一件事：把归一化后的意图命令（``IntentCommand``）映射为"发往哪个
ROS 服务或动作、带哪些参数"的 ``RouteResult``，但**不真正下发**——产出的
``RouteResult`` 默认 ``dry_run=True``，仅用于让操作员/上层确认将要调用的目标
接口。真正的执行模式分支在 ``execution_modes`` 中完成。

被路由到的目标接口（均为项目约定的对外契约名，不可随意改动）：

    /rebotarm/enable、/rebotarm/disable、/rebotarm/safe_home、
    /rebotarm/gripper/set、/rebotarm/move_to_pose（动作）、
    /rebotarm/move_relative（动作）、/rebotarm/pick_object（动作）、
    /rebotarm/place_object（动作）、/rebotarm/inspect_workspace、
    /rebotarm/confirm_action、/rebotarm/cancel_task、/rebotarm/stop

安全语义：任何无法识别的命令名一律抛 ``SafetyViolationError`` 终止流水线，
而不是"尽力猜测"或退化为空路由；命名位姿必须已存在于配置中，否则同样拒绝。
"""

from __future__ import annotations

from .models import IntentCommand, RouteResult, SafetyViolationError, VoiceControlConfig


class DryRunCommandRouter:
    """命令名 → 目标接口的静态路由表（演练用）。

    无状态、无订阅，构造时仅持有配置引用；``route`` 是纯函数式映射，可以被
    安全校验、任务模板展开与执行模式路由反复调用而不会产生副作用。
    """

    def __init__(self, config: VoiceControlConfig):
        self._config = config

    def route(self, command: IntentCommand) -> RouteResult:
        """把一条意图命令映射为目标接口。

        参数 ``command``：已经过安全校验（或即将被校验）的意图命令。

        返回 ``RouteResult``，其 ``mode`` 取 "service" / "action" /
        "dry_run_stop" 之一，``params`` 为下发给目标的参数。

        异常 ``SafetyViolationError``：命令名不在本路由表内，或引用了配置中
        不存在的命名位姿。上层必须视为硬失败并放弃执行。
        """
        if command.command == "enable_arm":
            return RouteResult(command.intent, "/rebotarm/enable", "service", {})
        if command.command == "disable_arm":
            return RouteResult(command.intent, "/rebotarm/disable", "service", {})
        if command.command == "safe_home":
            return RouteResult(command.intent, "/rebotarm/safe_home", "service", {})
        if command.command == "set_gripper":
            return RouteResult(
                command.intent,
                "/rebotarm/gripper/set",
                "service",
                dict(command.params),
            )
        if command.command == "move_named_pose":
            # 命名位姿必须能在 named_poses.yaml 中解析出具体坐标，否则丢弃该命令：
            # 不允许把未定义的位姿名透传给运动层去"猜"。
            name = command.params.get("name")
            pose = self._config.named_poses.get(str(name))
            if pose is None:
                raise SafetyViolationError(f"cannot route unknown named pose: {name}")
            return RouteResult(
                command.intent,
                "/rebotarm/move_to_pose",
                "action",
                {"name": name, "pose": pose},
            )
        if command.command == "move_relative":
            return RouteResult(
                command.intent,
                "/rebotarm/move_relative",
                "action",
                dict(command.params),
            )
        if command.command == "pick_object":
            return RouteResult(command.intent, "/rebotarm/pick_object", "action", dict(command.params))
        if command.command == "place_object":
            return RouteResult(command.intent, "/rebotarm/place_object", "action", dict(command.params))
        if command.command == "inspect_workspace":
            return RouteResult(command.intent, "/rebotarm/inspect_workspace", "service", dict(command.params))
        if command.command == "confirm_action":
            return RouteResult(command.intent, "/rebotarm/confirm_action", "service", dict(command.params))
        if command.command == "cancel_task":
            return RouteResult(command.intent, "/rebotarm/cancel_task", "service", dict(command.params))
        if command.command == "stop_motion":
            # 停止类命令特意标记为 dry_run_stop：它不是普通的服务/动作调用，
            # 由上层按最高优先级立即处理，不参与常规执行模式重写。
            return RouteResult(command.intent, "/rebotarm/stop", "dry_run_stop", {})
        # 未列入白名单的命令一律硬失败，杜绝隐式兜底行为。
        raise SafetyViolationError(f"cannot route command: {command.command}")
