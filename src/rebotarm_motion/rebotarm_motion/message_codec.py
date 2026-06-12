from __future__ import annotations

import json

from .command_models import PoseTarget, PreviewCommand


def encode_preview_command(preview: PreviewCommand, *, state: str) -> str:
    payload = {
        "state": state,
        "command_type": preview.command_type,
        "reachable": bool(preview.reachable),
        "message": preview.message,
        "joint_names": list(preview.joint_names),
        "joint_positions": list(preview.joint_positions),
    }
    if preview.pose_target is not None:
        payload["pose_target"] = {
            "x": preview.pose_target.x,
            "y": preview.pose_target.y,
            "z": preview.pose_target.z,
            "roll": preview.pose_target.roll,
            "pitch": preview.pose_target.pitch,
            "yaw": preview.pose_target.yaw,
        }
    return json.dumps(payload, separators=(",", ":"))


def decode_preview_command(payload: str) -> tuple[str | None, PreviewCommand | None]:
    try:
        data = json.loads(payload)
        pose_target = None
        raw_pose_target = data.get("pose_target")
        if isinstance(raw_pose_target, dict):
            pose_target = PoseTarget(
                x=float(raw_pose_target["x"]),
                y=float(raw_pose_target["y"]),
                z=float(raw_pose_target["z"]),
                roll=float(raw_pose_target["roll"]),
                pitch=float(raw_pose_target["pitch"]),
                yaw=float(raw_pose_target["yaw"]),
            )
        preview = PreviewCommand(
            command_type=str(data["command_type"]),
            reachable=bool(data["reachable"]),
            message=str(data["message"]),
            joint_names=tuple(str(v) for v in data["joint_names"]),
            joint_positions=tuple(float(v) for v in data["joint_positions"]),
            pose_target=pose_target,
        )
        return str(data.get("state")), preview
    except Exception:
        return None, None


def encode_status(*, mode: str, state: str, message: str) -> str:
    return json.dumps(
        {
            "mode": mode,
            "state": state,
            "message": message,
        },
        separators=(",", ":"),
    )
