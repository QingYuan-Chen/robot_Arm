"""执行模式路由：dry_run / sim / real 三种后端的选择与目标改写。

``ExecutionModeRouter`` 在安全门之后运行，决定命令最终发往哪里：

- ``dry_run``（默认）：只做接口映射，不产生任何真实调用；
- ``sim``：动作类目标改写到 /rebotarm/sim 前缀的仿真动作，其中
  move_relative 使用专门的仿真动作，其余命令沿用演练映射再改写前缀；
- ``real``：必须由 safety_limits.yaml 显式设置 allow_real_ros_calls: true
  才放行，否则直接判为安全违规；放行时使用与演练相同的真实接口映射，但把
  dry_run 标志置为 False，表示该路由允许被真正执行。

模式来源优先级：构造参数 execution_mode > safety_limits.yaml 的
execution_mode 键 > "dry_run"。
"""

from __future__ import annotations

from .command_router import DryRunCommandRouter
from .models import ExecutionRouteResult, IntentCommand, RouteResult, SafetyViolationError, VoiceControlConfig


class ExecutionModeRouter:
    """按执行模式把意图命令路由到演练 / 仿真 / 真实接口。"""

    def __init__(self, config: VoiceControlConfig, execution_mode: str | None = None):
        self._config = config
        self.execution_mode = execution_mode or str(
            config.safety_limits.get("execution_mode", "dry_run")
        )
        self._dry_run_router = DryRunCommandRouter(config)

    def route(self, command: IntentCommand) -> ExecutionRouteResult:
        """计算该命令在当前执行模式下的路由结果。

        real 模式下若未显式打开 allow_real_ros_calls，抛
        SafetyViolationError 而不是静默回退到仿真；未知模式同样直接拒绝。
        """
        if self.execution_mode == "dry_run":
            # 演练模式：只做接口映射，dry_run 保持 True，执行器会拒绝执行。
            return ExecutionRouteResult(
                execution_mode="dry_run",
                route=self._dry_run_router.route(command),
                simulated=False,
            )
        if self.execution_mode == "sim":
            return ExecutionRouteResult(
                execution_mode="sim",
                route=self._route_sim(command),
                simulated=True,
            )
        if self.execution_mode == "real":
            # 真实调用默认关闭；只有配置显式允许时才放行。
            if not bool(self._config.safety_limits.get("allow_real_ros_calls", False)):
                raise SafetyViolationError("real ROS calls are disabled by safety_limits.yaml")
            dry_route = self._dry_run_router.route(command)
            return ExecutionRouteResult(
                execution_mode="real",
                route=RouteResult(
                    dry_route.intent,
                    dry_route.target,
                    dry_route.mode,
                    dry_route.params,
                    dry_run=False,
                ),
                simulated=False,
            )
        raise SafetyViolationError(f"unsupported execution mode: {self.execution_mode}")

    def _route_sim(self, command: IntentCommand) -> RouteResult:
        """把命令映射到仿真后端，返回 dry_run=False 的可执行路由。

        move_relative 在仿真侧有专用动作；其余命令沿用演练映射，再把路径
        前缀 /rebotarm 替换为 /rebotarm/sim。
        """
        if command.command == "move_relative":
            return RouteResult(
                command.intent,
                "/rebotarm/sim/move_relative",
                "action",
                dict(command.params),
                dry_run=False,
            )
        dry_route = self._dry_run_router.route(command)
        return RouteResult(
            dry_route.intent,
            f"/rebotarm/sim{dry_route.target.removeprefix('/rebotarm')}",
            dry_route.mode,
            dry_route.params,
            dry_run=False,
        )
