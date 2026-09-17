#!/usr/bin/env python3
"""P0 验收工具：显式 enable、原位 hold 与 disable 的实机安全门（Gate B/C）。

本脚本是现场验收工具而非运行时组件，只读地消费硬件包发布的状态与反馈：

- 订阅 ``/<namespace>/arm_status``（锁存状态，见 ``_status_cb``）与
  ``/<namespace>/joint_states``（传感器数据 QoS，见 ``_joint_state_cb``）；
- 调用 ``/<namespace>/enable``、``/<namespace>/disable``、
  ``/<namespace>/trajectory_stop`` 三个 Trigger 服务；服务名由 ``--namespace``
  拼出，默认命名空间为 ``rebotarm``。

验收流程（对应 Gate B/C）：

1. 预检：必须同时拿到最新的 arm_status 与至少 3 帧 joint_states，六轴名称/位置/
   速度有效，电机处于失能、控制循环未激活、状态机为 IDLE、六个电机状态码全为
   0、无 error code，且当前绝对速度本身不超阈值，否则立即失败不做任何动作；
2. 人工确认：打印基线与提示，只有操作员逐字输入 ``ENABLE_HOLD_TEST`` 才继续，
   否则记为 ABORTED 并退出；
3. 显式 enable：调用 enable 服务，并逐项复核 arm_status 已进入使能且关节
   状态码全为 1；
4. hold 监控：在 ``--hold-seconds`` 内不发送任何目标点，只以 enable 瞬间的位置
   为基线，监视每帧的最大位置跳变与最大绝对速度，超阈值立即失败；
5. disable：停止运动监控后调用 disable，复核状态回到失能、状态码全为 0，
   并确认 disable 之后 joint_states 仍在继续刷新。

安全约束：真机默认失能；本工具绝不发送 trajectory、回位或夹爪命令；任何异常、
超时或操作员中断（SIGINT/SIGTERM）都会走 finally 里的补偿清理——只要本次使能过
或发现硬件意外处于使能态，就先 ``trajectory_stop`` 再 ``disable``。

无论通过、失败还是中止，都会在 ``--report-dir`` 下写出 JSON 证据
（``gate-bc-<时间戳>.json``），退出码为 0 仅代表 outcome == "PASSED"。
"""

from __future__ import annotations

import argparse
import json
import signal
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from rebotarm_msgs.msg import ArmStatus
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from ..p0_acceptance_core import (
    EXPECTED_JOINTS,
    arm_status_snapshot,
    joint_state_snapshot,
    motion_sample_details,
    validate_preflight,
)

# 操作员必须在安全确认处逐字输入的确认词；字符串内容属于验收流程约定，
# 现场文档、测试与历史证据都按该字面量核对，不可改动。
CONFIRMATION_TOKEN = "ENABLE_HOLD_TEST"


class GateBCAcceptance(Node):
    """Gate B/C 验收节点：持有订阅/客户端，并在回调中累计运动监控指标。

    回调模型：本类不使用 executor，全部由 ``wait_until`` / ``call_trigger`` 里的
    ``rclpy.spin_once`` 在调用线程上驱动，因此所有成员只在单线程中被读写。
    该节点不发布任何话题、也不发送运动命令，只调用 enable/disable/stop 三个服务。
    """

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("p0_gate_bc_acceptance")
        self._args = args
        # 去掉首尾斜杠，便于按 "/<namespace>/<资源名>" 统一拼接话题与服务名。
        namespace = args.namespace.strip("/")
        self._namespace = namespace
        # 最近一次收到的状态/反馈，以及最后一帧关节反馈的到达时刻（单调时钟）。
        self._latest_status: ArmStatus | None = None
        self._latest_joint_state: JointState | None = None
        self._last_joint_state_monotonic: float | None = None
        # 累计收到的关节反馈帧数，用于判断反馈是否仍在刷新。
        self._joint_sample_count = 0
        # enable 瞬间的关节位置基线；监控未开始时为 None。
        self._baseline_by_name: dict[str, float] | None = None
        self._monitor_motion = False
        # 峰值统计：监视期间见过的最大位置跳变与最大绝对速度及对应帧详情。
        self._max_position_jump_rad = 0.0
        self._max_abs_velocity_rad_s = 0.0
        self._peak_position_jump: dict[str, Any] | None = None
        self._peak_velocity: dict[str, Any] | None = None
        # 首个越限原因，记录后保持不再覆盖；非 None 即代表验收应失败。
        self._monitor_error: str | None = None
        # 由信号处理器置位：请求停止时应尽快进入补偿清理，而不是继续 hold。
        self.stop_requested = False

        # arm_status 是锁存状态（depth=1 + 瞬时本地），晚加入的订阅者也能立刻拿到
        # 最后一条，避免因等待周期发布而误判硬件不可达。
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            ArmStatus,
            f"/{namespace}/arm_status",
            self._status_cb,
            status_qos,
        )
        # 关节反馈是高频流数据，用传感器数据 QoS（尽力而为、小队列），
        # 丢帧可接受，但不能因积压而引入陈旧数据。
        self.create_subscription(
            JointState,
            f"/{namespace}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self._enable_client = self.create_client(Trigger, f"/{namespace}/enable")
        self._disable_client = self.create_client(Trigger, f"/{namespace}/disable")
        # 轨迹急停：hold 越限或异常时的第一道软件止动手段。
        self._stop_client = self.create_client(
            Trigger,
            f"/{namespace}/trajectory_stop",
        )

    def _status_cb(self, msg: ArmStatus) -> None:
        self._latest_status = msg

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        self._last_joint_state_monotonic = time.monotonic()
        self._joint_sample_count += 1
        # 监控未启动（enable 之前）时不做越限判定，避免把预检阶段的静止噪声计入指标。
        if not self._monitor_motion or self._baseline_by_name is None:
            return
        try:
            details = motion_sample_details(
                self._baseline_by_name,
                msg,
            )
        except ValueError as exc:
            self._monitor_error = str(exc)
            return
        max_jump = float(details["max_position_jump_rad"])
        max_velocity = float(details["max_abs_velocity_rad_s"])
        # 指标只增不减地保留峰值，便于失败后写进报告复盘。
        if max_jump > self._max_position_jump_rad:
            self._max_position_jump_rad = max_jump
            self._peak_position_jump = dict(details)
        if max_velocity > self._max_abs_velocity_rad_s:
            self._max_abs_velocity_rad_s = max_velocity
            self._peak_velocity = dict(details)
        # 越限即记录原因（位置优先于速度）。这里不直接调用服务：本回调在
        # spin_once 内执行，任何阻塞式服务调用都会卡住反馈采集本身。
        if max_jump > self._args.max_position_jump_rad:
            self._monitor_error = (
                f"{details['position_jump_joint']} position jump "
                f"{max_jump:.6f} rad exceeds "
                f"{self._args.max_position_jump_rad:.6f} rad"
            )
        elif max_velocity > self._args.max_abs_velocity_rad_s:
            self._monitor_error = (
                f"{details['velocity_joint']} velocity "
                f"{max_velocity:.6f} rad/s exceeds "
                f"{self._args.max_abs_velocity_rad_s:.6f} rad/s"
            )

    def wait_until(
        self,
        predicate: Callable[[], bool],
        timeout_sec: float,
    ) -> bool:
        """在超时内一边自旋处理回调一边轮询条件。

        参数 ``timeout_sec`` 小于等于 0 时只做一次判定，不做等待；轮询以 0.05 s
        为步长，保证 SIGINT/SIGTERM 触发的 ``stop_requested`` 能被及时看到。
        返回值：条件已满足为 True；超时或收到停止请求后在退出前再做一次判定并
        以该结果为准（避免边界上刚好满足却被判失败）。
        """
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while rclpy.ok() and not self.stop_requested and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def call_trigger(
        self,
        client,
        label: str,
        *,
        timeout_sec: float | None = None,
    ) -> tuple[bool, str]:
        """同步调用一个 Trigger 服务，返回 ``(success, message)``。

        ``label`` 仅用于拼装错误文本，调用方在报告里也用它标识是哪个服务。
        ``timeout_sec`` 为 None 时取 ``--service-timeout-sec``；等待服务可用最多花
        2 s（取超时与 2 s 的较小值），剩余的整段超时留给服务本身执行。
        任何失败（服务不可用/超时/无响应）都返回 False 而不抛异常，由调用方决定
        是中止验收还是走补偿清理。
        """
        timeout = self._args.service_timeout_sec if timeout_sec is None else timeout_sec
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return False, f"{label} service unavailable"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return False, f"{label} service timed out"
        response = future.result()
        if response is None:
            return False, f"{label} returned no response"
        return bool(response.success), str(response.message)

    def wait_for_initial_state(self) -> bool:
        """等待硬件状态与至少 3 帧关节反馈，三者齐备才算预检数据可用。"""
        return self.wait_until(
            lambda: self._latest_status is not None
            and self._latest_joint_state is not None
            and self._joint_sample_count >= 3,
            self._args.state_timeout_sec,
        )

    def start_motion_monitor(self) -> None:
        """以当前帧为基线开始运动监控，并清零历史峰值与越限原因。"""
        assert self._latest_joint_state is not None
        self._baseline_by_name = {
            name: float(position)
            for name, position in zip(
                self._latest_joint_state.name,
                self._latest_joint_state.position,
            )
        }
        self._max_position_jump_rad = 0.0
        self._max_abs_velocity_rad_s = 0.0
        self._peak_position_jump = None
        self._peak_velocity = None
        self._monitor_error = None
        self._monitor_motion = True

    def enabled_status_ready(self) -> bool:
        """判定 enable 是否真正生效并处于可保持状态。

        要求同时满足：使能标志为真、控制循环已激活、状态机回到 IDLE（无轨迹在
        跑）、六个电机状态码全为 1（对应达妙电机“已使能”的反馈）、无 error code。
        """
        status = self._latest_status
        return bool(
            status is not None
            and status.enabled
            and status.control_loop_active
            and status.state_machine == "IDLE"
            and list(status.per_joint_status_code) == [1] * len(EXPECTED_JOINTS)
            and not status.error_codes
        )

    def disabled_status_ready(self) -> bool:
        """判定 disable 是否彻底完成：失能、控制回路停转、状态机 IDLE、状态码全 0。"""
        status = self._latest_status
        return bool(
            status is not None
            and not status.enabled
            and not status.control_loop_active
            and status.state_machine == "IDLE"
            and list(status.per_joint_status_code) == [0] * len(EXPECTED_JOINTS)
        )

    def joint_state_is_fresh(self) -> bool:
        """反馈新鲜度自检：超过 ``--joint-state-stale-sec`` 未收到关节反馈即视为过期。

        陈旧反馈不能作为“机械臂保持不动”的证据——通信中断、控制循环卡死都会
        表现为反馈停更而不是报错，因此每次判定前都必须先过这一关。
        """
        return bool(
            self._last_joint_state_monotonic is not None
            and time.monotonic() - self._last_joint_state_monotonic
            <= self._args.joint_state_stale_sec
        )

    def emergency_cleanup(self) -> list[dict[str, Any]]:
        """补偿清理：先止动再失能，返回供报告记录的逐条服务结果。

        无论各步成功与否都不抛异常——清理是异常路径上的最后手段，不能因为某一步
        失败而中断后续（最关键的 disable）调用。止动超时压缩到 3 s，避免异常收尾
        被长时间阻塞。
        """
        events: list[dict[str, Any]] = []
        stop_ok, stop_message = self.call_trigger(
            self._stop_client,
            "trajectory_stop",
            timeout_sec=min(self._args.service_timeout_sec, 3.0),
        )
        events.append(
            {"service": "trajectory_stop", "success": stop_ok, "message": stop_message}
        )
        disable_ok, disable_message = self.call_trigger(
            self._disable_client,
            "disable",
        )
        events.append(
            {"service": "disable", "success": disable_ok, "message": disable_message}
        )
        self.wait_until(self.disabled_status_ready, self._args.state_timeout_sec)
        return events


def _parse_args() -> argparse.Namespace:
    """解析验收参数；--hold-seconds 等阈值由 main 再做正值校验。"""
    parser = argparse.ArgumentParser(description=__doc__)
    # 话题与服务的前缀命名空间，默认与硬件驱动出厂配置一致。
    parser.add_argument("--namespace", default="rebotarm")
    # 显式 enable 之后原地保持的时长，s；期间只监控、不发送任何目标点。
    parser.add_argument("--hold-seconds", type=float, default=10.0)
    # hold 期间允许的最大单帧位置跳变，rad（默认 0.03 rad ≈ 1.72°）。
    parser.add_argument("--max-position-jump-rad", type=float, default=0.03)
    # hold 期间允许的最大关节绝对速度，rad/s（默认 0.05）。
    parser.add_argument("--max-abs-velocity-rad-s", type=float, default=0.05)
    # 等待状态/反馈就绪以及等待状态位翻转的超时，s。
    parser.add_argument("--state-timeout-sec", type=float, default=5.0)
    # 单次 Trigger 服务调用的总超时，s；enable/disable 需要等待控制回路切换。
    parser.add_argument("--service-timeout-sec", type=float, default=12.0)
    # 关节反馈新鲜度上限，s；超过该时长没有新帧即判定反馈过期。
    parser.add_argument("--joint-state-stale-sec", type=float, default=0.5)
    # 验收证据 JSON 的输出目录（相对仓库根目录或绝对路径均可）。
    parser.add_argument(
        "--report-dir",
        default="Agent/evidence/P0",
        help="directory for the JSON acceptance record",
    )
    return parser.parse_args()


def _write_report(report: dict[str, Any], report_dir: str) -> Path:
    """把验收结果写成带本地时间戳的 JSON 证据文件，返回实际写入路径。

    文件名为 ``gate-bc-YYYYmmdd-HHMMSS.json``，时间取自本地时区；内容用
    ``ensure_ascii=False`` 保留中文、缩进 2 空格并以换行结尾，便于人工 diff。
    """
    output_dir = Path(report_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    path = output_dir / f"gate-bc-{stamp}.json"
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def main() -> None:
    """验收工具入口：预检 -> 人工确认 -> enable -> hold 监控 -> disable -> 写报告。

    退出码：outcome 为 "PASSED" 时 0，其余（FAILED/ABORTED）为 1，便于验收脚本判读。
    """
    args = _parse_args()
    # 阈值必须为正，否则越限判定失去意义；这些参数在后续被直接当作门限使用。
    if args.hold_seconds <= 0.0:
        raise SystemExit("--hold-seconds must be positive")
    if args.max_position_jump_rad <= 0.0:
        raise SystemExit("--max-position-jump-rad must be positive")
    if args.max_abs_velocity_rad_s <= 0.0:
        raise SystemExit("--max-abs-velocity-rad-s must be positive")
    if args.joint_state_stale_sec <= 0.0:
        raise SystemExit("--joint-state-stale-sec must be positive")

    # 报告先按最坏情况初始化（outcome=FAILED），任何一条异常路径都能直接落盘；
    # schema_version 供下游证据解析脚本判别字段结构。
    report: dict[str, Any] = {
        "schema_version": 1,
        "gate": "P0-Gate-B-C",
        "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "namespace": args.namespace.strip("/"),
        "thresholds": {
            "hold_seconds": float(args.hold_seconds),
            "max_position_jump_rad": float(args.max_position_jump_rad),
            "max_abs_velocity_rad_s": float(args.max_abs_velocity_rad_s),
            "joint_state_stale_sec": float(args.joint_state_stale_sec),
        },
        "services": [],
        "outcome": "FAILED",
        "failure_reason": None,
    }

    # 关掉 rclpy 自带的信号处理：本工具需要自己接管 SIGINT/SIGTERM，
    # 以便把中断转成“先补偿清理、再写报告”，而不是让进程直接被终止。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GateBCAcceptance(args)
    # 标记“本次已请求 enable 且尚未确认 disable 成功”，用于 finally 里决定是否需要清理。
    enable_requested = False

    def request_stop(_signum, _frame) -> None:
        # 信号处理器只置位并不做 I/O：真正的止动/失能放在 finally 中统一执行。
        node.stop_requested = True
        node.get_logger().warning("stop requested; disabling hardware")

    old_sigint = signal.getsignal(signal.SIGINT)
    old_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        # 预检一：状态与反馈可用性。
        if not node.wait_for_initial_state():
            raise RuntimeError("arm_status/joint_states preflight data unavailable")
        # 下面的快照/校验都需要非 None；断言同时起到类型收窄作用。
        assert node._latest_status is not None
        assert node._latest_joint_state is not None
        report["preflight_status"] = arm_status_snapshot(node._latest_status)
        report["preflight_joint_state"] = joint_state_snapshot(
            node._latest_joint_state
        )
        # 预检二：六轴名称/数值有效、失能、控制循环停转、状态码全 0、无 error code。
        errors = validate_preflight(node._latest_status, node._latest_joint_state)
        if errors:
            raise RuntimeError("preflight failed: " + "; ".join(errors))
        # 预检三：当前残余速度必须已经低于后续 hold 门限，否则静止判定无从谈起。
        baseline_speed = max(
            abs(float(value)) for value in node._latest_joint_state.velocity
        )
        if baseline_speed > args.max_abs_velocity_rad_s:
            raise RuntimeError(
                f"preflight velocity {baseline_speed:.6f} rad/s exceeds "
                f"{args.max_abs_velocity_rad_s:.6f} rad/s"
            )

        # 打印基线与现场要求，供操作员判断能否安全使能（此阶段机械臂仍失能）。
        print("\nP0 Gate B/C preflight passed.")
        print("No trajectory, safe-home, or gripper command will be sent.")
        print("Baseline joint positions / rad:")
        for name, position in zip(
            node._latest_joint_state.name,
            node._latest_joint_state.position,
        ):
            print(f"  {name}: {float(position):+.6f}")
        print(
            f"Type {CONFIRMATION_TOKEN!r} only when the workcell is clear "
            "and the physical emergency stop is ready."
        )
        # 人工安全确认是硬门：确认词不匹配即中止，全程不会调用 enable。
        confirmation = input("> ").strip()
        if confirmation != CONFIRMATION_TOKEN:
            report["outcome"] = "ABORTED"
            raise RuntimeError("operator confirmation was not provided")
        # 确认期间收到 Ctrl-C/SIGTERM 同样中止。
        if node.stop_requested:
            report["outcome"] = "ABORTED"
            raise RuntimeError("stop requested before enable")

        # 必须在 enable 之前开始监控：基线取“上电瞬间”的最后一帧位置，
        # 这样才能观察到位保持期间真正的跳变，而不是上电后的新稳态。
        node.start_motion_monitor()
        enable_requested = True
        enable_ok, enable_message = node.call_trigger(
            node._enable_client,
            "enable",
        )
        report["services"].append(
            {"service": "enable", "success": enable_ok, "message": enable_message}
        )
        if not enable_ok:
            raise RuntimeError(f"enable failed: {enable_message}")
        # 服务返回成功只是“请求被受理”，必须再等状态位与六个状态码同时到位。
        if not node.wait_until(node.enabled_status_ready, args.state_timeout_sec):
            raise RuntimeError("enabled status verification failed")
        if not node.joint_state_is_fresh():
            raise RuntimeError("joint states became stale after enable")
        if node._monitor_error is not None:
            raise RuntimeError(node._monitor_error)

        # hold 阶段：不发送任何目标点，只让电机保持当前位置并监视反馈；
        # 位置跳变、速度超限、反馈过期或操作员中断都会立刻跳出并在 finally 中清理。
        hold_deadline = time.monotonic() + args.hold_seconds
        while time.monotonic() < hold_deadline:
            if node.stop_requested:
                report["outcome"] = "ABORTED"
                raise RuntimeError("operator requested stop during hold")
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.joint_state_is_fresh():
                raise RuntimeError("joint states became stale during hold")
            if node._monitor_error is not None:
                raise RuntimeError(node._monitor_error)

        # 先冻结监控，避免把 disable 造成的姿态变化计入 hold 指标。
        node._monitor_motion = False
        samples_before_disable = node._joint_sample_count
        disable_ok, disable_message = node.call_trigger(
            node._disable_client,
            "disable",
        )
        report["services"].append(
            {"service": "disable", "success": disable_ok, "message": disable_message}
        )
        if not disable_ok:
            raise RuntimeError(f"disable failed: {disable_message}")
        if not node.wait_until(node.disabled_status_ready, args.state_timeout_sec):
            raise RuntimeError("disabled status verification failed")
        # 失能后反馈必须继续刷新：证明 disable 只停电机不停反馈通道，
        # 否则后续任何依赖关节反馈的安全判定都无法成立。
        if not node.wait_until(
            lambda: node._joint_sample_count >= samples_before_disable + 2,
            args.state_timeout_sec,
        ):
            raise RuntimeError("joint states did not continue after disable")

        # 走到这里说明失能已被确认，finally 不再需要补偿清理。
        enable_requested = False
        report["outcome"] = "PASSED"
    except Exception as exc:
        report["failure_reason"] = str(exc)
        node.get_logger().error(str(exc))
    finally:
        # 兜底安全门：只要本次使能过，或硬件此刻意外处于使能态，就必须止动 + 失能。
        # 这是“无论验收结果如何都回到失能”的最后一道软件保证。
        unexpectedly_enabled = bool(
            node._latest_status is not None and node._latest_status.enabled
        )
        if enable_requested or unexpectedly_enabled:
            report["cleanup_services"] = node.emergency_cleanup()
        node._monitor_motion = False
        # 指标与末态快照无论成败都要落盘，作为验收证据。
        report["metrics"] = {
            "max_position_jump_rad": node._max_position_jump_rad,
            "max_abs_velocity_rad_s": node._max_abs_velocity_rad_s,
            "peak_position_jump": node._peak_position_jump,
            "peak_velocity": node._peak_velocity,
            "joint_samples": node._joint_sample_count,
        }
        if node._latest_status is not None:
            report["final_status"] = arm_status_snapshot(node._latest_status)
        if node._latest_joint_state is not None:
            report["final_joint_state"] = joint_state_snapshot(
                node._latest_joint_state
            )
        report["finished_at"] = datetime.now().astimezone().isoformat(
            timespec="seconds"
        )
        # 落盘证据、恢复原有信号处理器并释放节点；报告路径与结论同时打印到终端。
        report_path = _write_report(report, args.report_dir)
        print(f"Acceptance report: {report_path}")
        print(f"Outcome: {report['outcome']}")
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
        node.destroy_node()
        rclpy.shutdown()

    # 退出码约定：只有 PASSED 返回 0，FAILED/ABORTED 均返回 1。
    raise SystemExit(0 if report["outcome"] == "PASSED" else 1)


if __name__ == "__main__":
    main()
