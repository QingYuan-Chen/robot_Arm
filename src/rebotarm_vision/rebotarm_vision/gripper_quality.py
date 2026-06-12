from __future__ import annotations


def close_contact_success(
    *,
    command_success: bool,
    target_position_m: float,
    reached_position_m: float,
    previous_open_position_m: float | None,
    contact_margin_m: float,
    min_closure_delta_m: float,
) -> bool:
    if command_success:
        return True
    if previous_open_position_m is None:
        return False

    target = float(target_position_m)
    reached = float(reached_position_m)
    previous_open = float(previous_open_position_m)

    closed_enough_to_contact = reached >= target + float(contact_margin_m)
    moved_from_open = previous_open - reached >= float(min_closure_delta_m)
    return closed_enough_to_contact and moved_from_open
