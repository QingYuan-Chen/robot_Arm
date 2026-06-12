from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable


def select_collision_samples(samples, *, max_samples: int) -> list[tuple[int, object]]:
    if not samples:
        return []
    limit = max(int(max_samples), 1)
    if len(samples) <= limit:
        return list(enumerate(samples))
    if limit == 1:
        return [(0, samples[0])]
    indices = sorted(
        {
            round(index * (len(samples) - 1) / (limit - 1))
            for index in range(limit)
        }
    )
    return [(index, samples[index]) for index in indices]


@dataclass(frozen=True)
class CollisionPrecheckConfig:
    enabled: bool
    service: str
    group_name: str
    max_samples: int
    timeout_sec: float
    default_joint_positions: tuple[tuple[str, float], ...] = field(default_factory=tuple)


class CollisionPrechecker:
    """Run MoveIt state-validity collision checks for sampled joint positions."""

    def __init__(
        self,
        *,
        client: Any,
        request_factory: Callable[[], Any],
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._client = client
        self._request_factory = request_factory
        self._monotonic = monotonic
        self._sleep = sleep

    def check_positions(
        self,
        *,
        joint_names: tuple[str, ...],
        positions_list: list[tuple[float, ...]],
        config: CollisionPrecheckConfig,
    ) -> dict:
        if not bool(config.enabled):
            return {"state": "disabled", "message": "collision precheck disabled"}
        if not joint_names or not positions_list:
            return {"state": "unknown", "message": "no trajectory samples to check"}
        if not self._service_available():
            return {
                "state": "unknown",
                "message": "MoveIt state validity service unavailable",
                "service": str(config.service),
                "checked_samples": 0,
            }
        joint_names, positions_list, added_defaults = self._with_default_joint_positions(
            joint_names,
            positions_list,
            config.default_joint_positions,
        )
        selected = select_collision_samples(positions_list, max_samples=max(int(config.max_samples), 1))
        collisions = []
        checked = 0
        deadline = self._monotonic() + max(float(config.timeout_sec), 0.1)
        for sample_index, positions in selected:
            if self._monotonic() >= deadline:
                return self._unknown_timeout(checked, selected, collisions)
            request = self._request_factory()
            request.group_name = str(config.group_name)
            request.robot_state.joint_state.name = list(joint_names)
            request.robot_state.joint_state.position = [float(v) for v in positions]
            future = self._client.call_async(request)
            while not future.done() and self._monotonic() < deadline:
                self._sleep(0.01)
            if not future.done():
                break
            try:
                response = future.result()
            except Exception as exc:
                return {
                    "state": "unknown",
                    "message": f"collision precheck failed: {exc}",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            checked += 1
            if not bool(getattr(response, "valid", False)):
                collisions.append(
                    {
                        "sample": sample_index,
                        "contacts": self._contacts_to_dicts(getattr(response, "contacts", [])),
                    }
                )
                break
        if collisions:
            return {
                "state": "collision",
                "message": "collision detected in teach trajectory",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": collisions,
            }
        if checked < len(selected):
            return {
                "state": "unknown",
                "message": "collision precheck incomplete",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": [],
            }
        return {
            "state": "pass",
            "message": "no collision detected in sampled teach trajectory",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": [],
            "added_default_joints": added_defaults,
        }

    def _service_available(self) -> bool:
        try:
            available = bool(self._client.service_is_ready())
            if not available:
                available = bool(self._client.wait_for_service(timeout_sec=0.0))
            return available
        except Exception:
            return False

    @staticmethod
    def _contacts_to_dicts(contacts) -> list[dict[str, str]]:
        result = []
        for contact in list(contacts)[:5]:
            result.append(
                {
                    "body_1": str(getattr(contact, "contact_body_1", "")),
                    "body_2": str(getattr(contact, "contact_body_2", "")),
                }
            )
        return result

    @staticmethod
    def _with_default_joint_positions(
        joint_names: tuple[str, ...],
        positions_list: list[tuple[float, ...]],
        default_joint_positions: tuple[tuple[str, float], ...],
    ) -> tuple[tuple[str, ...], list[tuple[float, ...]], list[str]]:
        names = tuple(str(name) for name in joint_names)
        existing = set(names)
        defaults = [
            (str(name), float(position))
            for name, position in default_joint_positions
            if str(name) not in existing
        ]
        if not defaults:
            return names, positions_list, []
        added_names = tuple(name for name, _ in defaults)
        added_values = tuple(position for _, position in defaults)
        return (
            names + added_names,
            [tuple(float(v) for v in positions) + added_values for positions in positions_list],
            list(added_names),
        )

    @staticmethod
    def _unknown_timeout(checked: int, selected, collisions: list) -> dict:
        return {
            "state": "unknown",
            "message": "collision precheck timed out",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": collisions,
        }
