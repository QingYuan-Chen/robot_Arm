"""示教回放的运行期跟踪守卫。

回放期间定期把**期望轨迹**与**实际关节反馈**做对比，一旦超差持续超过宽限期就请求
停止。这是最后一道运动守卫：它不参与规划，只回答「现在该不该停」。

判定顺序（自上而下短路）：

1. 未启用、无活动轨迹、未记录起始时刻，或此前已请求停止 → 不重复判停；
2. 处于启动宽限期（``start_grace_sec``，秒）→ 保持放行，避免控制器起停瞬间的反馈
   滞后被误判；
3. 调用轨迹安全评估模块的 ``evaluate_replay_tracking``，按期望轨迹插值出当前应有的
   关节角，检查跟踪误差（rad）与实时速度（rad/s）；
4. 一旦报错，先记录首次违规时刻，只有违规**连续**持续到超过 ``violation_grace_sec``
   （秒）才真正请求停止；期间若恢复达标则清零重新计时（避免单帧抖动停机）。

时间基准：``started_at`` 与 ``now`` 由调用方给出，必须来自同一时钟（上层用
``time.monotonic()``），否则宽限期会失效。本类是有状态对象，``check`` 不是纯函数；
多线程调用需调用方自行加锁。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .trajectory_safety_monitor import evaluate_replay_tracking


@dataclass(frozen=True)
class ReplayRuntimeMonitorConfig:
    """运行期守卫的判定阈值。

    - ``enabled``：总开关，关闭后 ``check`` 恒返回「不停」；
    - ``start_grace_sec``：轨迹开始后的启动宽限期（秒），此窗口内完全不做判定；
    - ``violation_grace_sec``：违规容忍时长（秒），超差需连续持续这么久才停机；
    - ``max_tracking_error_rad``：允许的最大跟踪误差（rad），建议远小于相邻关节的
      机械间隙与碰撞距离余量；
    - ``max_live_velocity_rad_s``：允许的最大实时关节速度（rad/s），用于识别「反馈
      速度异常」这类误差尚未体现但已经失控的情形。
    """

    enabled: bool
    start_grace_sec: float
    violation_grace_sec: float
    max_tracking_error_rad: float
    max_live_velocity_rad_s: float


@dataclass(frozen=True)
class ReplayRuntimeMonitorDecision:
    """一次运行期检查的结论。

    ``should_stop`` 为 ``True`` 时 ``status`` 必带 ``state="safety_stop"`` 的状态字典，
    调用方应据此请求控制器停止并取消回放目标；为 ``False`` 时 ``status`` 为空字典。
    """

    should_stop: bool
    status: dict = field(default_factory=dict)


class ReplayRuntimeMonitor:
    """面向活动示教回放的有状态运行期守卫。

    生命周期：一次回放开始前调用 :meth:`reset` 清空历史，回放期间周期性调用
    :meth:`check`；一旦 :attr:`stop_requested` 变为 ``True``，后续 ``check`` 直接短路，
    必须先 ``reset`` 才能用于下一次回放。守卫只提出停止请求，实际停止动作与目标取消
    由上层完成。
    """

    def __init__(self) -> None:
        # 首次发现违规的时刻（调用方时钟，秒）；达标后清零，用于实现连续违规计时。
        self.violation_since: float | None = None
        # 已请求过停止的锁存标志：置位后不再重复判定，避免重复下发停止请求。
        self.stop_requested = False

    def reset(self) -> None:
        """清空违规计时与停止锁存，供下一次回放复用同一实例。"""
        self.violation_since = None
        self.stop_requested = False

    def check(
        self,
        *,
        trajectory,
        started_at: float | None,
        joints: dict,
        now: float,
        config: ReplayRuntimeMonitorConfig,
    ) -> ReplayRuntimeMonitorDecision:
        """执行一次运行期跟踪检查。

    参数：
    - ``trajectory``：活动回放轨迹，需带 ``joint_names`` 与 ``points``（每点含
      ``positions`` 与 ``time_from_start``）；``None`` 表示当前没有回放；
    - ``started_at``：轨迹下发的起始时刻（秒，与 ``now`` 同一时钟）；``None`` 表示
      尚未开始计时；
    - ``joints``：最新关节状态，形如 ``{关节名: {"position": rad, "velocity": rad/s}}``，
      缺字段时按 0 处理；
    - ``now``：本次检查时刻（秒，需单调时钟）；
    - ``config``：阈值配置，见 :class:`ReplayRuntimeMonitorConfig`。

    返回「是否停止」及可发布的状态负载。超差时先记录首次违规时刻并放行，只有违规持
    续超过 ``violation_grace_sec`` 才置位 :attr:`stop_requested` 并返回停止请求；一旦
    恢复达标立即清除计时，因此偶发抖动不会导致误停。
    """
        if not bool(config.enabled) or trajectory is None or started_at is None or self.stop_requested:
            return ReplayRuntimeMonitorDecision(False)
        elapsed = float(now) - float(started_at)
        if elapsed < float(config.start_grace_sec):
            return ReplayRuntimeMonitorDecision(False)
        result = evaluate_replay_tracking(
            trajectory,
            joint_names=tuple(joints.keys()),
            positions=tuple(float(item.get("position", 0.0)) for item in joints.values()),
            velocities=tuple(float(item.get("velocity", 0.0)) for item in joints.values()),
            elapsed_sec=elapsed,
            max_tracking_error_rad=float(config.max_tracking_error_rad),
            max_live_velocity_rad_s=float(config.max_live_velocity_rad_s),
        )
        if result.ok:
            self.violation_since = None
            return ReplayRuntimeMonitorDecision(False)
        # 首次违规只记录起始时刻并放行，等待违规是否连续，避免单帧抖动触发停机。
        if self.violation_since is None:
            self.violation_since = float(now)
            return ReplayRuntimeMonitorDecision(False)
        if float(now) - float(self.violation_since) < float(config.violation_grace_sec):
            return ReplayRuntimeMonitorDecision(False)
        self.stop_requested = True
        return ReplayRuntimeMonitorDecision(
            True,
            {
                "state": "safety_stop",
                "message": f"runtime replay monitor stopped trajectory: {result.message}",
                "runtime_monitor": {
                    # 停机原因（tracking_error / live_velocity），供面板与日志区分。
                    "reason": result.reason,
                    "worst_joint": result.worst_joint,
                    "max_tracking_error_rad": result.max_tracking_error_rad,
                    "max_live_velocity_rad_s": result.max_live_velocity_rad_s,
                    "tracking_error": result.reason == "tracking_error",
                    "live_velocity": result.reason == "live_velocity",
                },
                # 真实回放的运行期停机，不是干跑演练，上层据此保留安全停机状态。
                "dry_run": False,
            },
        )
