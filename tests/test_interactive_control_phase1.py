from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "src" / "rebotarm_interactive_control"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rebotarm_interactive_control.command_models import (  # type: ignore[import-not-found]
    ControlMode,
    ExecutionState,
    PoseTarget,
    PreviewSolveResult,
)
from rebotarm_interactive_control.execution_coordinator import (  # type: ignore[import-not-found]
    InteractiveCoordinator,
)
from rebotarm_interactive_control.preview_manager import (  # type: ignore[import-not-found]
    PreviewManager,
)
from rebotarm_interactive_control.pose_math import (  # type: ignore[import-not-found]
    quaternion_to_rpy,
    rpy_to_quaternion,
)
from rebotarm_interactive_control.parameter_helpers import (  # type: ignore[import-not-found]
    build_joint_limits,
)
from rebotarm_interactive_control.parameter_helpers import (  # type: ignore[import-not-found]
    sensor_qos_kwargs,
)


class FakePoseSolver:
    def __init__(self, result: PreviewSolveResult) -> None:
        self.result = result
        self.calls: list[tuple[PoseTarget, tuple[float, ...], tuple[str, ...]]] = []

    def solve_pose(
        self,
        pose_target: PoseTarget,
        seed_positions: tuple[float, ...],
        joint_names: tuple[str, ...],
    ) -> PreviewSolveResult:
        self.calls.append((pose_target, seed_positions, joint_names))
        return self.result


class PreviewManagerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.joint_names = ("joint1", "joint2")
        self.joint_limits = {
            "joint1": (-1.0, 1.0),
            "joint2": (-2.0, 2.0),
        }

    def test_joint_preview_merges_partial_target_over_current_state(self) -> None:
        manager = PreviewManager(
            joint_names=self.joint_names,
            joint_limits=self.joint_limits,
            initial_positions=(0.2, -0.1),
        )

        preview = manager.preview_joint_target({"joint2": 1.5})

        self.assertTrue(preview.reachable)
        self.assertEqual(preview.command_type, "joint")
        self.assertEqual(preview.joint_positions, (0.2, 1.5))
        self.assertEqual(manager.current_positions, (0.2, 1.5))

    def test_joint_preview_rejects_limit_violation(self) -> None:
        manager = PreviewManager(
            joint_names=self.joint_names,
            joint_limits=self.joint_limits,
            initial_positions=(0.0, 0.0),
        )

        preview = manager.preview_joint_target({"joint1": 1.5})

        self.assertFalse(preview.reachable)
        self.assertIn("joint1", preview.message)
        self.assertEqual(manager.current_positions, (0.0, 0.0))

    def test_pose_preview_uses_solver_result_without_moving_current_state(self) -> None:
        solver = FakePoseSolver(
            PreviewSolveResult(
                success=True,
                joint_positions=(0.4, -0.6),
                message="ik ok",
            )
        )
        manager = PreviewManager(
            joint_names=self.joint_names,
            joint_limits=self.joint_limits,
            initial_positions=(0.0, 0.0),
            pose_solver=solver,
        )

        preview = manager.preview_pose_target(PoseTarget(0.1, 0.2, 0.3, 0.0, 0.0, 0.0))

        self.assertTrue(preview.reachable)
        self.assertEqual(preview.command_type, "pose")
        self.assertEqual(preview.joint_positions, (0.4, -0.6))
        self.assertEqual(manager.current_positions, (0.0, 0.0))
        self.assertEqual(len(solver.calls), 1)
        self.assertEqual(solver.calls[0][1], (0.0, 0.0))

    def test_sync_current_positions_updates_ordered_joint_state(self) -> None:
        manager = PreviewManager(
            joint_names=self.joint_names,
            joint_limits=self.joint_limits,
            initial_positions=(0.0, 0.0),
        )

        manager.sync_current_positions(
            {"joint2": -0.3, "joint1": 0.7, "unused_joint": 9.9}
        )

        self.assertEqual(manager.current_positions, (0.7, -0.3))


class InteractiveCoordinatorTests(unittest.TestCase):
    def setUp(self) -> None:
        joint_names = ("joint1", "joint2")
        joint_limits = {
            "joint1": (-1.0, 1.0),
            "joint2": (-1.5, 1.5),
        }
        self.preview_manager = PreviewManager(
            joint_names=joint_names,
            joint_limits=joint_limits,
            initial_positions=(0.0, 0.0),
        )

    def test_simulation_execution_returns_preview_only_request(self) -> None:
        coordinator = InteractiveCoordinator(
            preview_manager=self.preview_manager,
            default_mode=ControlMode.SIMULATION,
        )
        coordinator.preview_joint_target({"joint1": 0.5})

        decision = coordinator.execute_preview(duration=1.25)

        self.assertTrue(decision.accepted)
        self.assertIsNotNone(decision.request)
        assert decision.request is not None
        self.assertTrue(decision.request.preview_only)
        self.assertEqual(decision.request.mode, ControlMode.SIMULATION)
        self.assertEqual(decision.request.joint_positions, (0.5, 0.0))
        self.assertEqual(coordinator.execution_state, ExecutionState.EXECUTING)

    def test_stop_execution_returns_to_idle_without_latching(self) -> None:
        coordinator = InteractiveCoordinator(
            preview_manager=self.preview_manager,
            default_mode=ControlMode.REAL,
        )
        coordinator.preview_joint_target({"joint2": -0.5})
        first_decision = coordinator.execute_preview(duration=2.0)
        self.assertTrue(first_decision.accepted)

        coordinator.stop_execution()

        self.assertEqual(coordinator.execution_state, ExecutionState.IDLE)

    def test_stop_does_not_require_reset_before_next_execution(self) -> None:
        coordinator = InteractiveCoordinator(
            preview_manager=self.preview_manager,
            default_mode=ControlMode.REAL,
        )
        coordinator.preview_joint_target({"joint1": -0.25})
        first_decision = coordinator.execute_preview(duration=1.0)
        self.assertTrue(first_decision.accepted)
        coordinator.stop_execution()

        decision = coordinator.execute_preview(duration=2.5)

        self.assertTrue(decision.accepted)
        self.assertIsNotNone(decision.request)
        assert decision.request is not None
        self.assertFalse(decision.request.preview_only)
        self.assertEqual(decision.request.mode, ControlMode.REAL)
        self.assertEqual(decision.request.joint_positions, (-0.25, 0.0))
        self.assertEqual(coordinator.execution_state, ExecutionState.EXECUTING)

    def test_execute_preview_rejects_when_execution_is_already_running(self) -> None:
        coordinator = InteractiveCoordinator(
            preview_manager=self.preview_manager,
            default_mode=ControlMode.REAL,
        )
        coordinator.preview_joint_target({"joint1": 0.25})
        first_decision = coordinator.execute_preview(duration=1.0)
        self.assertTrue(first_decision.accepted)

        second_decision = coordinator.execute_preview(duration=1.0)

        self.assertFalse(second_decision.accepted)
        self.assertIn("already in progress", second_decision.message)
        self.assertIsNone(second_decision.request)


class PoseMathTests(unittest.TestCase):
    def test_quaternion_to_rpy_identity(self) -> None:
        roll, pitch, yaw = quaternion_to_rpy(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_rpy_quaternion_roundtrip_for_yaw(self) -> None:
        quat = rpy_to_quaternion(0.0, 0.0, 1.57079632679)
        roll, pitch, yaw = quaternion_to_rpy(*quat)
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(yaw, 1.57079632679, places=6)


class ParameterHelperTests(unittest.TestCase):
    def test_build_joint_limits_from_parallel_arrays(self) -> None:
        limits = build_joint_limits(
            joint_names=("joint1", "joint2"),
            lower_limits=(-1.0, -2.0),
            upper_limits=(1.0, 2.0),
        )

        self.assertEqual(
            limits,
            {
                "joint1": (-1.0, 1.0),
                "joint2": (-2.0, 2.0),
            },
        )

    def test_build_joint_limits_rejects_length_mismatch(self) -> None:
        with self.assertRaises(ValueError):
            build_joint_limits(
                joint_names=("joint1", "joint2"),
                lower_limits=(-1.0,),
                upper_limits=(1.0, 2.0),
            )

    def test_sensor_qos_kwargs_uses_best_effort_sensor_profile(self) -> None:
        qos = sensor_qos_kwargs()
        self.assertEqual(qos["depth"], 10)
        self.assertEqual(qos["reliability"], "best_effort")


if __name__ == "__main__":
    unittest.main()
