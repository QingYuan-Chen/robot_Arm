"""预览状态管理：在不触碰硬件的前提下维护"当前关节状态 + 最后一次预览"。

职责与位置
----------
本模块属于运动层的纯状态组件（不依赖运行时框架、不下发任何命令、不访问硬件）。
上层协调器 `InteractiveCoordinator` 持有一个实例，把操作员的关节/位姿目标先交给
它做可达性与限位判定，只有 `reachable=True` 的预览才允许进入执行门。

设计要点
--------
- **部分更新语义**：目标字典只给一部分关节时，其余关节沿用当前值（增量式操作）；
- **失败不改状态**：未知关节名、超限或逆解失败都返回 `reachable=False`，并且
  **保持 `current_positions` 不变**，避免"半应用的"预览污染后续目标；
- **可达性不等于安全**：这里的判定只覆盖关节名合法性与关节限位；碰撞、速度、
  真机状态由执行链路上的其他组件负责。
"""

from __future__ import annotations

from .command_models import PoseSolverProtocol, PoseTarget, PreviewCommand


class PreviewManager:
    """维护一级预览状态，全程不触碰硬件执行。

    （原英文说明：Maintains phase-1 preview state without touching hardware execution.）

    维护两级预览状态：当前关节位置 `_current_positions`（六个关节，rad，按
    `joint_names` 的顺序排列）与最后一次预览结果 `_last_preview`。

    生命周期：构造时给定关节名、限位与初始位置；运行期通过
    `sync_current_positions` 与真实状态对齐，通过 `preview_joint_target` /
    `preview_pose_target` 产生预览。本身无线程，调用方需保证同一实例不被并发修改。
    """

    def __init__(
        self,
        *,
        joint_names: tuple[str, ...],
        joint_limits: dict[str, tuple[float, float]],
        initial_positions: tuple[float, ...],
        pose_solver: PoseSolverProtocol | None = None,
    ) -> None:
        """参数：`joint_names` 关节名顺序；`joint_limits` 每关节 (下限, 上限)，单位 rad；
        `initial_positions` 与关节名等长的初始角度（rad）；`pose_solver` 可选的逆解器，
        为 None 时位姿预览恒返回不可达（关节预览不受影响）。

        关节名与初始位置长度不一致直接抛 `ValueError`：长度错配会让整条预览链
        的索引全部错位，属于不可接受的编程错误，故不允许静默截断。
        """
        if len(joint_names) != len(initial_positions):
            raise ValueError("joint_names and initial_positions must have same length")
        self._joint_names = joint_names
        self._joint_limits = joint_limits
        self._current_positions = tuple(float(v) for v in initial_positions)
        self._pose_solver = pose_solver
        self._last_preview: PreviewCommand | None = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self._joint_names

    @property
    def current_positions(self) -> tuple[float, ...]:
        return self._current_positions

    @property
    def last_preview(self) -> PreviewCommand | None:
        return self._last_preview

    def sync_current_positions(self, current_positions: dict[str, float]) -> None:
        """用实测关节状态刷新内部当前值。

        只覆盖关节名在 `joint_names` 中出现的项，其余关节保持原值（部分话题
        可能只发布本组关节，或首次同步时缺少个别轴）。传入值单位 rad，
        不做限位裁剪——越界属于硬件/上游问题，不应在这里被悄悄修正。
        """
        merged = list(self._current_positions)
        index_by_name = {name: idx for idx, name in enumerate(self._joint_names)}
        for name, value in current_positions.items():
            if name not in index_by_name:
                continue
            merged[index_by_name[name]] = float(value)
        self._current_positions = tuple(merged)

    def preview_joint_target(self, joint_targets: dict[str, float]) -> PreviewCommand:
        """预览一组关节目标（增量语义），返回 `command_type="joint"` 的结果。

        参数 `joint_targets`：关节名 -> 目标角（rad）。只给出的关节被替换，其余
        沿用当前值。判定顺序为"先校验全部目标、再一次性提交"：

        - 任一关节名不在 `joint_names` 中 -> `reachable=False`，消息为未知关节；
        - 任一关节超出 `joint_limits` -> `reachable=False`，消息为限位越界；
        - 全部通过 -> 提交为新当前值并返回 `reachable=True`。

        失败时当前状态保持原样，返回的 `joint_positions` 也是未修改的当前值。
        """
        updated = list(self._current_positions)
        index_by_name = {name: idx for idx, name in enumerate(self._joint_names)}
        for name, value in joint_targets.items():
            if name not in index_by_name:
                preview = PreviewCommand(
                    command_type="joint",
                    reachable=False,
                    message=f"unknown joint: {name}",
                    joint_names=self._joint_names,
                    joint_positions=self._current_positions,
                )
                self._last_preview = preview
                return preview
            lower, upper = self._joint_limits[name]
            if value < lower or value > upper:
                preview = PreviewCommand(
                    command_type="joint",
                    reachable=False,
                    message=f"joint limit exceeded: {name}",
                    joint_names=self._joint_names,
                    joint_positions=self._current_positions,
                )
                self._last_preview = preview
                return preview
            updated[index_by_name[name]] = float(value)

        result_positions = tuple(updated)
        preview = PreviewCommand(
            command_type="joint",
            reachable=True,
            message="joint preview ready",
            joint_names=self._joint_names,
            joint_positions=result_positions,
        )
        self._current_positions = result_positions
        self._last_preview = preview
        return preview

    def preview_pose_target(self, pose_target: PoseTarget) -> PreviewCommand:
        """预览一个末端位姿目标，返回 `command_type="pose"` 的结果。

        `pose_target` 为末端在 `frame_id` 下的位置（m）与姿态（roll/pitch/yaw，rad）。
        逆解由注入的 `pose_solver` 完成，并以**当前关节值作为种子**（保证解的
        分支与当前姿态连续，避免无谓的大幅关节翻转）。

        - 未注入逆解器 -> `reachable=False`，"pose preview solver unavailable"；
        - 逆解失败 -> `reachable=False`，沿用逆解器给出的 message，关节位置回填当前值；
        - 逆解成功 -> `reachable=True`，`joint_positions` 为解。

        与关节预览不同，本方法**不修改** `_current_positions`：位姿预览只是候选，
        真正提交发生在后续的执行受理环节（由协调器负责）。
        """
        if self._pose_solver is None:
            preview = PreviewCommand(
                command_type="pose",
                reachable=False,
                message="pose preview solver unavailable",
                joint_names=self._joint_names,
                joint_positions=self._current_positions,
                pose_target=pose_target,
            )
            self._last_preview = preview
            return preview

        result = self._pose_solver.solve_pose(
            pose_target,
            self._current_positions,
            self._joint_names,
        )
        preview = PreviewCommand(
            command_type="pose",
            reachable=bool(result.success),
            message=result.message,
            joint_names=self._joint_names,
            joint_positions=(
                result.joint_positions if result.success else self._current_positions
            ),
            pose_target=pose_target,
        )
        self._last_preview = preview
        return preview
