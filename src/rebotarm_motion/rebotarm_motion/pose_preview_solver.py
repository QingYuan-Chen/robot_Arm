from __future__ import annotations

import sys
from pathlib import Path

from .command_models import PoseTarget, PreviewSolveResult


def _sdk_candidates(workspace_root: Path) -> list[Path]:
    return [
        workspace_root / "third_party" / "reBotArm_control_py",
        workspace_root / "sdk" / "reBotArm_control_py",
        workspace_root.parent / "reBotArm_control_py-main",
    ]


def ensure_sdk_on_syspath(workspace_root: Path) -> Path:
    for root in _sdk_candidates(workspace_root):
        if (root / "reBotArm_control_py").is_dir():
            root_str = str(root)
            if root_str not in sys.path:
                sys.path.insert(0, root_str)
            return root
    candidates = "\n".join(f"  - {path}" for path in _sdk_candidates(workspace_root))
    raise FileNotFoundError(
        "Cannot find reBotArm_control_py for preview solving. Expected one of:\n"
        f"{candidates}"
    )


class PosePreviewSolver:
    """Pure preview solver that reuses the Python SDK kinematics without moving hardware."""

    def __init__(self, workspace_root: Path, end_frame_name: str = "end_link") -> None:
        ensure_sdk_on_syspath(workspace_root)

        from reBotArm_control_py.kinematics import (  # pylint: disable=import-outside-toplevel
            get_end_effector_frame_id,
            load_robot_model,
            pos_rot_to_se3,
            solve_ik_with_retry,
        )

        self._load_robot_model = load_robot_model
        self._get_end_effector_frame_id = get_end_effector_frame_id
        self._pos_rot_to_se3 = pos_rot_to_se3
        self._solve_ik_with_retry = solve_ik_with_retry

        self._model = self._load_robot_model()
        self._data = self._model.createData()
        self._end_frame_id = self._get_end_effector_frame_id(self._model)
        self._end_frame_name = end_frame_name

    def solve_pose(
        self,
        pose_target: PoseTarget,
        seed_positions: tuple[float, ...],
        joint_names: tuple[str, ...],
    ) -> PreviewSolveResult:
        del joint_names
        import numpy as np  # pylint: disable=import-outside-toplevel

        target = self._pos_rot_to_se3(
            np.array([pose_target.x, pose_target.y, pose_target.z], dtype=float),
            roll=pose_target.roll,
            pitch=pose_target.pitch,
            yaw=pose_target.yaw,
        )
        q_seed = np.array(seed_positions, dtype=float)
        result = self._solve_ik_with_retry(
            self._model,
            self._data,
            self._end_frame_id,
            target,
            q_seed,
        )
        if not result.success:
            return PreviewSolveResult(
                success=False,
                joint_positions=seed_positions,
                message=(
                    f"target pose unreachable for {self._end_frame_name}: "
                    f"error={result.error:.4e}"
                ),
            )
        return PreviewSolveResult(
            success=True,
            joint_positions=tuple(float(v) for v in result.q.tolist()),
            message="ik preview ok",
        )

    def compute_pose_target(self, joint_positions: tuple[float, ...]) -> PoseTarget:
        import numpy as np  # pylint: disable=import-outside-toplevel
        import pinocchio as pin  # pylint: disable=import-outside-toplevel
        from reBotArm_control_py.kinematics import compute_fk  # pylint: disable=import-outside-toplevel

        q = np.array(joint_positions, dtype=float)
        position, rotation, _ = compute_fk(self._model, q, frame_name=self._end_frame_name)
        roll, pitch, yaw = pin.rpy.matrixToRpy(rotation)
        return PoseTarget(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
        )
