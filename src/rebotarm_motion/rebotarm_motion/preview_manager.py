from __future__ import annotations

from .command_models import PoseSolverProtocol, PoseTarget, PreviewCommand


class PreviewManager:
    """Maintains phase-1 preview state without touching hardware execution."""

    def __init__(
        self,
        *,
        joint_names: tuple[str, ...],
        joint_limits: dict[str, tuple[float, float]],
        initial_positions: tuple[float, ...],
        pose_solver: PoseSolverProtocol | None = None,
    ) -> None:
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
        merged = list(self._current_positions)
        index_by_name = {name: idx for idx, name in enumerate(self._joint_names)}
        for name, value in current_positions.items():
            if name not in index_by_name:
                continue
            merged[index_by_name[name]] = float(value)
        self._current_positions = tuple(merged)

    def preview_joint_target(self, joint_targets: dict[str, float]) -> PreviewCommand:
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
