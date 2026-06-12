from __future__ import annotations


VALID_ARM_COMMANDS = frozenset(("safe_home", "enable", "disable"))
REPLAY_LOCK_STATES = frozenset(("replaying", "cancel_requested", "stop_requested"))
TRAJECTORY_STOP_COMMANDS = frozenset(("safe_home", "disable"))


def normalize_arm_command(command: object) -> str | None:
    value = str(command or "").strip()
    return value if value in VALID_ARM_COMMANDS else None


def arm_command_is_replay_locked(replay_state: object) -> bool:
    return str(replay_state or "").strip().lower() in REPLAY_LOCK_STATES


def status_state(value: object) -> str:
    if isinstance(value, dict):
        return str(value.get("state", "") or "").strip().lower()
    return str(value or "").strip().lower()


def should_stop_trajectory_before_arm_command(command: object) -> bool:
    normalized = normalize_arm_command(command)
    return normalized in TRAJECTORY_STOP_COMMANDS


def arm_command_timeout_sec(command: object) -> float:
    normalized = normalize_arm_command(command)
    return {"safe_home": 30.0, "enable": 8.0, "disable": 10.0}.get(str(normalized), 2.0)
