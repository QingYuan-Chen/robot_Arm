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
    """Owns phase-1 mode switching, preview caching, and execution gating."""

    def __init__(
        self,
        *,
        preview_manager: PreviewManager,
        default_mode: ControlMode = ControlMode.SIMULATION,
    ) -> None:
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
        self._mode = mode

    def preview_joint_target(self, joint_targets: dict[str, float]) -> PreviewCommand:
        preview = self._preview_manager.preview_joint_target(joint_targets)
        self._last_preview = preview
        self._execution_state = (
            ExecutionState.PREVIEW_READY if preview.reachable else ExecutionState.IDLE
        )
        return preview

    def preview_pose_target(self, pose_target: PoseTarget) -> PreviewCommand:
        preview = self._preview_manager.preview_pose_target(pose_target)
        self._last_preview = preview
        self._execution_state = (
            ExecutionState.PREVIEW_READY if preview.reachable else ExecutionState.IDLE
        )
        return preview

    def stop_execution(self) -> None:
        self._execution_state = ExecutionState.IDLE

    def execution_finished(self) -> None:
        self._execution_state = ExecutionState.IDLE

    def execute_preview(self, *, duration: float) -> ExecutionDecision:
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
            preview_only=self._mode == ControlMode.SIMULATION,
            preview_command=self._last_preview,
        )
        self._execution_state = ExecutionState.EXECUTING
        return ExecutionDecision(
            accepted=True,
            message="execution accepted",
            request=request,
        )
