"""预览与执行之间的协调器（模式切换、预览缓存、执行门控）。

本模块是「预览 → 执行」的单一准入点。它持有三类状态：

- 当前控制模式（仿真/真实），决定执行请求是只做预览还是允许下发真实运动；
- 上一次预览命令，作为执行请求的唯一来源，杜绝「未预览直接执行」；
- 执行状态机的当前状态，保证同一时刻最多只有一次执行在途。

职责边界：本类不做运动学计算，也不发送任何控制指令，只负责判定与打包；关节限位
校验由预览管理器完成（不可达的预览不会被缓存为可执行目标）。上层拿到
:class:`~rebotarm_motion.command_models.ExecutionDecision` 后，若 ``accepted`` 为
``True`` 才继续调用控制器/执行节点，并在执行结束或用户停止时回调
:meth:`InteractiveCoordinator.execution_finished` 或
:meth:`InteractiveCoordinator.stop_execution`，否则状态机会一直卡在 ``EXECUTING``
而拒绝后续请求。
"""

from __future__ import annotations

from .command_models import (
    ControlMode,
    ExecutionDecision,
    ExecutionRequest,
    ExecutionState,
    PoseTarget,
    PreviewCommand,
)
from .preview_manager import PreviewManager


class InteractiveCoordinator:
    """负责第一阶段的模式切换、预览缓存与执行门控。

    生命周期：构造后先 :meth:`set_mode` 选定模式（默认仿真），再反复「预览 → 执行」。
    预览成功会把状态推进到 ``PREVIEW_READY``，预览不可达则退回 ``IDLE``；
    :meth:`execute_preview` 受理后进入 ``EXECUTING``，必须由结束/停止回调释放。
    本类不加锁，假设由单线程（例如定时器或状态机回调）串行驱动。
    """

    def __init__(
        self,
        *,
        preview_manager: PreviewManager,
        default_mode: ControlMode = ControlMode.SIMULATION,
    ) -> None:
        # 默认为仿真模式：即使调用方未显式选择，也不会把运动意外下发到真实机械臂。
        self._preview_manager = preview_manager
        self._mode = default_mode
        self._execution_state = ExecutionState.IDLE
        self._last_preview: PreviewCommand | None = None

    @property
    def mode(self) -> ControlMode:
        return self._mode

    @property
    def execution_state(self) -> ExecutionState:
        return self._execution_state

    @property
    def last_preview(self) -> PreviewCommand | None:
        return self._last_preview

    def set_mode(self, mode: ControlMode) -> None:
        """切换控制模式（仿真/真实）。

        只改模式本身，不清空已有预览，也不影响进行中的执行；切换模式后如需按新模式
        重新决策，应由上层重新发起预览。
        """
        self._mode = mode

    def preview_joint_target(self, joint_targets: dict[str, float]) -> PreviewCommand:
        """关节空间预览并缓存结果。

        ``joint_targets`` 为 ``{关节名: 目标角(rad)}``，可只给部分关节（未给出的沿用
        当前角，由预览管理器负责合并与限位校验）。预览成功后状态置为
        ``PREVIEW_READY``，失败（未知关节、超限）退回 ``IDLE``，两种情况下缓存都会被
        更新，因此后续执行请求一定对应最近一次预览。
        """
        preview = self._preview_manager.preview_joint_target(joint_targets)
        self._last_preview = preview
        self._execution_state = (
            ExecutionState.PREVIEW_READY if preview.reachable else ExecutionState.IDLE
        )
        return preview

    def preview_pose_target(self, pose_target: PoseTarget) -> PreviewCommand:
        """位姿预览（逆解）并缓存结果。

        与关节预览的差别在于可达性由逆解给出；位姿预览会携带 ``pose_target``，供后续
        规划与碰撞预检复用。状态迁移规则与 :meth:`preview_joint_target` 一致。
        """
        preview = self._preview_manager.preview_pose_target(pose_target)
        self._last_preview = preview
        self._execution_state = (
            ExecutionState.PREVIEW_READY if preview.reachable else ExecutionState.IDLE
        )
        return preview

    def stop_execution(self) -> None:
        """外部停止回调：把状态机复位为 ``IDLE``，使后续预览/执行可以继续。

        本方法只改状态，不发送停止指令——真正的停机由调用方所在的执行链路负责。
        """
        self._execution_state = ExecutionState.IDLE

    def execution_finished(self) -> None:
        """执行正常结束回调：把状态机复位为 ``IDLE``，与停止路径做同样的释放。"""
        self._execution_state = ExecutionState.IDLE

    def execute_preview(self, *, duration: float) -> ExecutionDecision:
        """请求执行最近一次预览目标。

        ``duration`` 为目标轨迹总时长（秒），需为正数，由执行侧换算成轨迹时间戳。

        三道门按顺序拒绝：已有执行在途（防重入）、没有预览（防跳过预览）、预览不可达
        （防执行无效目标）；全部通过才构造 :class:`ExecutionRequest` 并把状态推进到
        ``EXECUTING``。``preview_only`` 在仿真模式下恒为 ``True``，即仿真模式不会产生
        真实硬件运动；被拒绝时 ``request`` 必为 ``None``。
        """
        if self._execution_state == ExecutionState.EXECUTING:
            return ExecutionDecision(
                accepted=False,
                message="execution already in progress",
                request=None,
            )
        if self._last_preview is None:
            return ExecutionDecision(
                accepted=False,
                message="no preview available",
                request=None,
            )
        if not self._last_preview.reachable:
            return ExecutionDecision(
                accepted=False,
                message="target pose unreachable",
                request=None,
            )

        request = ExecutionRequest(
            mode=self._mode,
            joint_names=self._last_preview.joint_names,
            joint_positions=self._last_preview.joint_positions,
            duration=float(duration),
            # 仿真模式一律标记为仅预览，真实运动必须由真实模式显式发起。
            preview_only=self._mode == ControlMode.SIMULATION,
            # 带上原始预览命令，供执行侧复核可达性与关节顺序。
            preview_command=self._last_preview,
        )
        self._execution_state = ExecutionState.EXECUTING
        return ExecutionDecision(
            accepted=True,
            message="execution accepted",
            request=request,
        )
