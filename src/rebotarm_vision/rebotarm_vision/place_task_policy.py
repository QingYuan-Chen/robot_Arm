from __future__ import annotations

from dataclasses import dataclass

from .visual_grasp_sequence import PoseTarget, VisualGraspStage


@dataclass(frozen=True)
class PlaceTaskConfig:
    enabled: bool = False
    place_position_xyz: tuple[float, float, float] = (0.20, -0.20, 0.25)
    place_orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    open_position_m: float = 0.08
    open_max_effort: float = 0.25
    retreat_z_m: float = 0.06


def build_place_stages(config: PlaceTaskConfig) -> list[VisualGraspStage]:
    if not config.enabled:
        return []
    place = PoseTarget(
        position=tuple(float(value) for value in config.place_position_xyz),
        orientation=tuple(float(value) for value in config.place_orientation_xyzw),
    )
    retreat = PoseTarget(
        position=(
            place.position[0],
            place.position[1],
            place.position[2] + max(0.0, float(config.retreat_z_m)),
        ),
        orientation=place.orientation,
    )
    return [
        VisualGraspStage(name="move_to_place", kind="move", pose=place),
        VisualGraspStage(
            name="open_gripper_at_place",
            kind="gripper",
            gripper_position_m=float(config.open_position_m),
            gripper_max_effort=float(config.open_max_effort),
        ),
        VisualGraspStage(name="place_retreat", kind="move", pose=retreat),
    ]
