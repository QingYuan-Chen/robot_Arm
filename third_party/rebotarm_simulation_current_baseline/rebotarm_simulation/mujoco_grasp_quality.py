from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspQuality:
    contact_detected: bool
    lift_detected: bool
    lift_height_m: float
    success: bool
    status: str


def evaluate_grasp_quality(
    *,
    contact_count: int,
    initial_box_height_m: float | None,
    final_box_height_m: float | None,
    min_lift_m: float = 0.03,
) -> GraspQuality:
    contact_detected = int(contact_count) > 0
    lift_height = 0.0
    if initial_box_height_m is not None and final_box_height_m is not None:
        lift_height = float(final_box_height_m) - float(initial_box_height_m)
    lift_detected = lift_height >= float(min_lift_m)
    if contact_detected and lift_detected:
        status = "grasp_lift_success"
    elif contact_detected:
        status = "contact_without_lift"
    else:
        status = "no_contact"
    return GraspQuality(
        contact_detected=contact_detected,
        lift_detected=lift_detected,
        lift_height_m=lift_height,
        success=contact_detected and lift_detected,
        status=status,
    )
