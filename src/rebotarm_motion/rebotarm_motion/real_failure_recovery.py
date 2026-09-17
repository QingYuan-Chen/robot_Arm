"""真机任务失败后的受保护恢复策略。

职责与位置
----------
本模块是运动层的纯逻辑工具（不依赖运行时框架、不直接访问硬件），被真机验收
/夹取 runner 在 try/except 中调用：一次真机任务（例如往返轨迹、手眼标定、
单瓶夹取）失败后，决定"控制器接下来该处于什么状态"。

核心安全原则（对应仓库硬规则）
------------------------------
健康电机绝不能被自动失能。一次可恢复的任务失败后只有两条路径：

- 控制器仍处于"已使能 + 控制环运行 + 六轴状态码正常 + 无错误码"的健康保持
  状态时，**保持使能**并交给操作员处理，或（仅在允许受控回基线时）用五次
  多项式轨迹把机械臂送回基线，落位误差在容差内才失能；
- 只有在**关键状态**（状态未使能/控制环停止/单轴状态码异常/存在错误码）时，
  才允许"远离基线失能"这一例外，因为此时保持力矩本身已不可信。

恢复结论通过 `report["failure_recovery"]` 结构化回写，包含 `outcome` 字符串
（每个分支一个稳定取值，测试与报告工具依赖它）、失败原因、位置与误差快照。

调用约定
--------
`node` 是调用方持有的运行时对象，本模块通过鸭子类型使用它的接口：`call_trigger`、
`stop_client`、`disable_client`、`hold_and_collect`、`latest_status`、
`_status_payload`、`canonical_positions`、`execute_leg`。`execute_leg` 返回的
leg 字典需含 `success` 与 `result` 字段。
"""

from __future__ import annotations

from collections.abc import MutableMapping, Sequence
from typing import Any

from .paired_trajectory_protocol import build_quintic_command


def healthy_enabled_hold(status: object | None) -> bool:
    """判断状态快照是否属于"健康的使能保持"。

    四个条件必须同时成立，缺一即视为不可信保持力矩：
    - `enabled`：驱动器已使能；
    - `control_loop_active`：控制环在跑（停止运行说明反馈已过期）；
    - `per_joint_status_code` 恰为六个 `1`：单轴状态码 1 表示该轴正常使能
      （0 为未使能、其他值为异常），`[1] * 6` 也隐含了轴数必须为 6；
    - `error_codes` 为空：无任何故障码。
    """
    return bool(
        status is not None
        and bool(getattr(status, "enabled", False))
        and bool(getattr(status, "control_loop_active", False))
        and list(getattr(status, "per_joint_status_code", ())) == [1] * 6
        and not list(getattr(status, "error_codes", ()))
    )


def _attempt_protective_disable(
    *,
    node: Any,
    recovery: MutableMapping[str, Any],
    services: list[Any],
    outcome: str,
) -> bool:
    """Attempt a critical-condition disable and record an unverifiable failure.

    关键状态下的保护性失能。返回值是该函数的"控制器是否仍使能"结论：
    失能调用成功返回 False（已失能）；失能调用本身抛异常时，无法确认是否真的
    失能，因此保守地返回 True（按"可能仍使能"处理），并把 `outcome` 改成
    `<outcome>_failed`、记录 `disable_failure`，绝不谎报失能成功。
    """
    recovery["outcome"] = outcome
    try:
        services.append(node.call_trigger(node.disable_client, "disable"))
    except Exception as exc:
        recovery["disable_failure"] = f"{type(exc).__name__}: {exc}"
        recovery["outcome"] = f"{outcome}_failed"
        return True
    return False


def recover_real_failure(
    *,
    node: Any,
    report: MutableMapping[str, Any],
    baseline: Sequence[float],
    allow_controlled_return: bool,
    legs_key: str,
    command_label: str,
    duration_sec: float = 20.0,
) -> bool:
    """Recover a real-arm task failure without dropping healthy holding torque.

    Returns whether the controller remains enabled. Automatic disable away from
    baseline is reserved for critical status where holding torque is not
    trustworthy.

    参数（均为关键字参数）：
    - `node`：运行时对象，见模块说明中的鸭子类型约定；
    - `report`：结果字典，本函数写入 `failure_recovery`，并向 `services` 与
      `legs_key` 指定的列表追加停止/失能服务结果与恢复轨迹记录；
    - `baseline`：已校验的基线关节位置，六个值，单位 rad（回到它才允许失能）；
    - `allow_controlled_return`：是否允许主动回基线。只有当下发中的轨迹已停下、
      作业流程明确处于"可安全回撤"窗口时才应为 True；否则只保持使能并交人处理；
    - `legs_key`：`report` 中存放轨迹 leg 列表的键名（如 "legs"、"motion_legs"）；
    - `command_label`：回基线命令的标签，写入命令载荷用于审计；
    - `duration_sec`：回基线轨迹总时长，单位 s，默认 20（低速回撤）。

    返回 True 表示控制器应当仍处于使能状态（调用方需继续按使能处理），
    返回 False 表示已确认失能。

    分支顺序（每一步都用 `outcome` 记录）：先请求停止 -> 保持并回读状态 ->
    状态缺失则不改状态直接返回 -> 非使能保持则按"已失能"返回 -> 状态不健康
    则保护性失能 -> 停止失败或不允许回撤则保持使能交人 -> 否则执行回基线腿并
    按停止后的状态重新判定。
    """
    recovery: dict[str, Any] = {
        "allow_controlled_return": bool(allow_controlled_return),
        "outcome": "started",
    }
    report["failure_recovery"] = recovery
    services = report.setdefault("services", [])
    legs = report.setdefault(legs_key, [])
    stop_failed = False
    try:
        # 任何恢复动作之前先请求停止：让正在跑的那条轨迹停下来，避免与回基线
        # 轨迹互相叠加。停止失败会被记录，并改变后续可选项。
        services.append(node.call_trigger(node.stop_client, "trajectory_stop"))
    except Exception as exc:
        stop_failed = True
        recovery["stop_failure"] = f"{type(exc).__name__}: {exc}"
    # 0.3 s 保持采样：给控制环和状态发布留出刷新时间，再读取判决依据。
    node.hold_and_collect(0.3)
    status = node.latest_status
    recovery["hold_status"] = node._status_payload()
    if status is None:
        # 没有状态就无从判断：不改动控制器状态，最保守地按"仍使能"上报。
        recovery["outcome"] = "status_unavailable_leave_state_unchanged"
        return True
    if not bool(status.enabled) or not bool(status.control_loop_active):
        # 控制器已不在使能保持态（可能已自行断开或已失能），无需也不应再下发命令。
        recovery["outcome"] = "controller_not_in_enabled_hold"
        return False
    if not healthy_enabled_hold(status):
        # 关键状态：保持力矩不可信，这是唯一允许远离基线自动失能的例外。
        outcome = (
            "stop_failure_critical_status_protective_disable"
            if stop_failed
            else "critical_status_protective_disable"
        )
        return _attempt_protective_disable(
            node=node,
            recovery=recovery,
            services=services,
            outcome=outcome,
        )
    if stop_failed:
        # 状态健康但停止请求失败：轨迹可能仍在跑，此时回基线会与它冲突，
        # 因此保持使能并标记为需要人工恢复。
        recovery["outcome"] = (
            "stop_failed_healthy_enabled_hold_requires_operator_recovery"
        )
        return True
    if not allow_controlled_return:
        # 上层明确表示当前窗口不允许回撤（例如轨迹仍在受控执行中）。
        recovery["outcome"] = "healthy_enabled_hold_requires_operator_recovery"
        return True

    baseline_values = tuple(float(value) for value in baseline)
    current = tuple(float(value) for value in node.canonical_positions())
    recovery["return_start_positions"] = list(current)
    # 五次多项式混合：起止速度与加速度均为 0，避免回撤启停时的冲击。
    command = build_quintic_command(
        current,
        baseline_values,
        duration_sec=float(duration_sec),
        cadence_sec=0.05,
        label=command_label,
    )
    try:
        return_leg = node.execute_leg(command)
    except Exception as exc:
        # 回撤指令本身抛异常（如反馈过期）：再判一次状态，健康则保持使能、
        # 不健康则保护性失能。
        recovery["return_failure"] = f"{type(exc).__name__}: {exc}"
        try:
            node.hold_and_collect(0.3)
        except Exception as status_exc:
            recovery["return_status_collection_failure"] = (
                f"{type(status_exc).__name__}: {status_exc}"
            )
        status = node.latest_status
        recovery["return_status"] = node._status_payload()
        if status is None:
            recovery["outcome"] = "return_exception_status_unavailable_leave_state_unchanged"
            return True
        if not bool(status.enabled) or not bool(status.control_loop_active):
            recovery["outcome"] = "return_exception_controller_not_in_enabled_hold"
            return False
        if not healthy_enabled_hold(status):
            return _attempt_protective_disable(
                node=node,
                recovery=recovery,
                services=services,
                outcome="return_exception_critical_status_protective_disable",
            )
        recovery["outcome"] = "return_exception_healthy_enabled_hold"
        return True
    # 这条腿是失败恢复产生的，打标记便于报告工具与正常作业腿区分。
    return_leg["recovery_leg"] = True
    legs.append(return_leg)
    if not bool(return_leg["success"]):
        # 回基线执行失败但未抛异常：同样按状态决定"保持使能"还是"保护性失能"。
        recovery["return_result"] = return_leg["result"]
        try:
            node.hold_and_collect(0.3)
        except Exception as status_exc:
            recovery["return_status_collection_failure"] = (
                f"{type(status_exc).__name__}: {status_exc}"
            )
        status = node.latest_status
        recovery["return_status"] = node._status_payload()
        if status is None:
            recovery["outcome"] = "return_failed_status_unavailable_leave_state_unchanged"
            return True
        if not bool(status.enabled) or not bool(status.control_loop_active):
            recovery["outcome"] = "return_failed_controller_not_in_enabled_hold"
            return False
        if not healthy_enabled_hold(status):
            return _attempt_protective_disable(
                node=node,
                recovery=recovery,
                services=services,
                outcome="return_failed_critical_status_protective_disable",
            )
        recovery["outcome"] = "return_failed_healthy_enabled_hold"
        return True
    # 已回到基线附近：再做一次 0.5 s 稳定采样后核对残差。
    node.hold_and_collect(0.5)
    final = tuple(float(value) for value in node.canonical_positions())
    errors = tuple(target - actual for target, actual in zip(baseline_values, final))
    recovery["enabled_final_positions"] = list(final)
    recovery["enabled_final_errors"] = list(errors)
    # 0.02 rad 残差门限：与工具侧"回到基线"判定保持一致；超差说明没真正回位，
    # 此时保持使能（保持力矩仍在）交操作员处理，而不是贸然失能。
    if max(abs(value) for value in errors) > 0.02:
        recovery["outcome"] = "return_error_healthy_enabled_hold"
        return True
    # 只有确认落在基线容差内，才允许主动失能。
    services.append(node.call_trigger(node.disable_client, "disable"))
    node.hold_and_collect(0.5)
    recovery["final_status"] = node._status_payload()
    recovery["outcome"] = "returned_to_baseline_then_disabled"
    return False
