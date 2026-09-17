"""任务模板展开器：把一条高层指令炸开为有序的低层步骤。

职责：当意图命令的 ``command`` 为 "run_task_template" 时，从 task_templates.yaml
取出对应模板的 ``steps``，逐条转换为 ``IntentCommand``；非模板命令原样返回单步
列表，使调用方无需分支处理。

安全语义：展开前先校验顶层命令，展开后**逐步再次校验**——模板文件属于可编辑
配置，不能因为"来自模板"就绕过白名单、工作空间与位移上限检查。steps 中的
params 使用深拷贝，防止某个步骤后续被就地修改而污染共享的模板配置。
"""

from __future__ import annotations

from copy import deepcopy

from .models import IntentCommand, SafetyViolationError, VoiceControlConfig
from .safety_guard import SafetyGuard


class TaskPlanner:
    """无状态展开器；构造时注入配置与安全守卫。"""

    def __init__(self, config: VoiceControlConfig, safety_guard: SafetyGuard):
        self._config = config
        self._safety_guard = safety_guard

    def expand(self, command: IntentCommand) -> list[IntentCommand]:
        """把命令展开为按顺序执行的步骤列表。

        参数 ``command``：待展开的意图命令。

        返回：模板命令返回其 steps 对应的多条命令（顺序即执行顺序）；其他命令
        返回只含自身的单元素列表。

        异常：``SafetyViolationError``——顶层命令未通过校验，或模板名在
        task_templates.yaml 中不存在（不接受未定义模板）。
        """
        command = self._safety_guard.validate(command)
        if command.command != "run_task_template":
            return [command]

        template_name = command.params.get("name")
        template = self._config.task_templates.get(str(template_name))
        if template is None:
            raise SafetyViolationError(f"task template does not exist: {template_name}")

        steps = []
        for raw_step in template.get("steps", []):
            step = IntentCommand(
                intent=str(raw_step["intent"]),
                command=str(raw_step["command"]),
                params=deepcopy(raw_step.get("params", {})),
                need_confirm=bool(raw_step.get("need_confirm", False)),
                # 原始文本透传自顶层命令，保证审计日志能追溯到用户真实说法。
                source_text=command.source_text,
                priority=str(raw_step.get("priority", "normal")),
            )
            # 每个步骤都必须独立过安全校验，模板不是豁免通道。
            steps.append(self._safety_guard.validate(step))
        return steps
