from __future__ import annotations

from dataclasses import dataclass

from .gripper_policy import GripperCommand
from .retreat_policy import RetreatPolicyConfig, build_lift_pose, build_retreat_pose


@dataclass(frozen=True)
class PoseTarget:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class VisualGraspSequenceConfig:
    open_before_approach: bool = False
    open_position_m: float = 0.09
    close_position_m: float = 0.025
    close_max_effort: float = 0.4
    lift_z_m: float = 0.08
    min_grasp_z_m: float = 0.0
    auto_gripper_width: bool = False
    detected_jaw_width_m: float = 0.0
    open_clearance_m: float = 0.0
    close_margin_m: float = 0.012
    min_open_position_m: float = 0.035
    max_open_position_m: float = 0.09
    min_close_position_m: float = 0.006
    max_close_position_m: float = 0.08
    gripper_command: GripperCommand | None = None
    retreat_policy: RetreatPolicyConfig = RetreatPolicyConfig()
    include_safe_home: bool = False


@dataclass(frozen=True)
class VisualGraspStage:
    name: str
    kind: str
    pose: PoseTarget | None = None
    gripper_position_m: float | None = None
    gripper_max_effort: float | None = None
    detected_jaw_width_m: float = 0.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def resolve_gripper_widths(config: VisualGraspSequenceConfig) -> tuple[float, float]:
    if not config.auto_gripper_width or float(config.detected_jaw_width_m) <= 0.0:
        return float(config.open_position_m), float(config.close_position_m)
    detected_width = max(float(config.detected_jaw_width_m), 0.0)
    open_width = _clamp(
        detected_width + float(config.open_clearance_m),
        float(config.min_open_position_m),
        float(config.max_open_position_m),
    )
    close_width = _clamp(
        detected_width - float(config.close_margin_m),
        float(config.min_close_position_m),
        float(config.max_close_position_m),
    )
    close_width = min(close_width, open_width)
    return open_width, close_width


def build_visual_grasp_sequence(
    pregrasp: PoseTarget,
    grasp: PoseTarget,
    config: VisualGraspSequenceConfig,
) -> list[VisualGraspStage]:
    if grasp.position[2] < config.min_grasp_z_m:
        raise ValueError(
            f"grasp z={grasp.position[2]:.3f} is below minimum "
            f"{config.min_grasp_z_m:.3f}"
        )

    min_lift_z = config.retreat_policy.min_lift_z_m if config.retreat_policy.enabled else 0.0
    lift_pose = build_lift_pose(grasp, lift_z_m=config.lift_z_m, min_lift_z_m=min_lift_z)
    if config.gripper_command is not None:
        if not config.gripper_command.allowed:
            raise ValueError(f"gripper policy rejected grasp: {config.gripper_command.reason}")
        open_width = float(config.gripper_command.open_width_m)
        close_width = float(config.gripper_command.close_width_m)
        max_effort = float(config.gripper_command.max_effort)
    else:
        open_width, close_width = resolve_gripper_widths(config)
        max_effort = float(config.close_max_effort)
    stages: list[VisualGraspStage] = []
    if config.open_before_approach:
        stages.append(
            VisualGraspStage(
                name="open_gripper",
                kind="gripper",
                gripper_position_m=open_width,
                gripper_max_effort=max_effort,
                detected_jaw_width_m=float(config.detected_jaw_width_m),
            )
        )
    stages.extend(
        [
            VisualGraspStage(
                name="move_to_pregrasp",
                kind="move",
                pose=pregrasp,
            ),
            VisualGraspStage(
                name="approach_grasp",
                kind="move",
                pose=grasp,
            ),
            VisualGraspStage(
                name="close_gripper",
                kind="gripper",
                gripper_position_m=close_width,
                gripper_max_effort=max_effort,
                detected_jaw_width_m=float(config.detected_jaw_width_m),
            ),
            VisualGraspStage(
                name="lift",
                kind="move",
                pose=lift_pose,
            ),
        ]
    )
    if config.retreat_policy.enabled:
        stages.append(
            VisualGraspStage(
                name="safe_retreat",
                kind="move",
                pose=build_retreat_pose(lift_pose, config.retreat_policy),
            )
        )
    if config.include_safe_home:
        stages.append(VisualGraspStage(name="safe_home", kind="safe_home"))
    return stages
