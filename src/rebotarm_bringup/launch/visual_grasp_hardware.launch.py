"""Launch the verified real-hardware visual grasp profile."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


_ASSET_FIELDS = {
    "vision_python_executable",
    "graspnet_python_executable",
    "graspnet_model_root",
    "graspnet_checkpoint_path",
}

_ARGUMENT_TYPES = {
    "use_hardware": bool,
    "shutdown_safe_home": bool,
    "execution_mode": str,
    "vision_profile": str,
    "graspnet_source_mode": str,
    "start_visual_ready": bool,
    "move_to_visual_ready_on_start": bool,
    "start_motion_execution": bool,
    "start_visual_grasp_executor": bool,
    "start_sim_trajectory_controller": bool,
    "use_local_rviz": bool,
    "candidate_pose_policy": str,
    "pose_policy": str,
    "fixed_grasp_orientation_xyzw": list,
    "tcp_offset_xyz": list,
    "candidate_min_confidence": float,
    "candidate_max_jaw_width_m": float,
    "max_allowed_grasp_width_m": float,
    "max_plan_age_sec": float,
    "hardware_feedback_rate_hz": float,
    "gripper_feedback_stale_timeout_sec": float,
    "gripper_position_max_speed_rad_s": float,
    "gripper_position_torque_cap_nm": float,
    "grasp_hold_timeout_sec": float,
    "execute_gripper": bool,
    "gripper_grasp_enabled": bool,
    "gripper_grasp_close_force": float,
    "gripper_grasp_timeout_sec": float,
    "auto_gripper_effort": bool,
    "close_max_effort": float,
    "lift_z_m": float,
    "safe_retreat_enabled": bool,
    "safe_home_after_grasp": bool,
    "trajectory_precheck_enabled": bool,
    "auto_retry_enabled": bool,
}


def _resolve_workspace_asset(relative_path: str, *, anchor: Path) -> str:
    path = Path(relative_path)
    if path.is_absolute():
        candidate = path
    else:
        candidate = next(
            (parent / path for parent in anchor.resolve().parents if (parent / path).exists()),
            None,
        )
        if candidate is None:
            raise RuntimeError(f"找不到视觉抓取资产: {relative_path}")
    if not candidate.exists():
        raise RuntimeError(f"找不到视觉抓取资产: {candidate}")
    # Keep venv interpreter symlinks intact so Python detects the venv prefix.
    return str(candidate.absolute())


def _launch_text(name: str, value: Any) -> str:
    expected_type = _ARGUMENT_TYPES[name]
    if expected_type is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f"视觉抓取配置字段 {name} 必须是数值")
        return str(value)
    if not isinstance(value, expected_type):
        raise RuntimeError(
            f"视觉抓取配置字段 {name} 必须是 {expected_type.__name__}"
        )
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, list):
        return json.dumps(value, separators=(",", ":"))
    return value


def _load_hardware_profile(path: Path, *, anchor: Path) -> dict[str, str]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise RuntimeError(f"视觉抓取配置 schema_version 无效: {path}")

    arguments = payload.get("launch_arguments")
    assets = payload.get("workspace_assets")
    if not isinstance(arguments, dict) or not isinstance(assets, dict):
        raise RuntimeError(f"视觉抓取配置必须包含 launch_arguments 和 workspace_assets: {path}")
    if "channel" in arguments or "channel" in assets:
        raise RuntimeError("channel 必须在启动时显式指定，禁止写入视觉抓取配置")

    missing_arguments = set(_ARGUMENT_TYPES) - set(arguments)
    extra_arguments = set(arguments) - set(_ARGUMENT_TYPES)
    missing_assets = _ASSET_FIELDS - set(assets)
    extra_assets = set(assets) - _ASSET_FIELDS
    if missing_arguments or missing_assets:
        missing = sorted(missing_arguments | missing_assets)
        raise RuntimeError(f"视觉抓取配置缺少字段: {', '.join(missing)}")
    if extra_arguments or extra_assets:
        extra = sorted(extra_arguments | extra_assets)
        raise RuntimeError(f"视觉抓取配置包含未知字段: {', '.join(extra)}")

    if arguments["use_hardware"] is not True:
        raise RuntimeError("use_hardware 必须为 true")
    if arguments["execution_mode"] != "plan_only":
        raise RuntimeError(
            "execution_mode 必须为 plan_only；实机动作只允许由 P6 runner 发起"
        )
    if arguments["start_visual_grasp_executor"] is not False:
        raise RuntimeError(
            "start_visual_grasp_executor 必须为 false；禁止与 P6 runner 同时发起动作"
        )
    if arguments["start_motion_execution"] is not True:
        raise RuntimeError(
            "start_motion_execution 必须为 true；P6 runner 需要规划和轨迹执行服务"
        )

    resolved = {}
    for name, value in assets.items():
        if not isinstance(value, str):
            raise RuntimeError(f"视觉抓取资产字段 {name} 必须是路径字符串")
        resolved[name] = _resolve_workspace_asset(value, anchor=anchor)
    resolved.update(
        {name: _launch_text(name, value) for name, value in arguments.items()}
    )
    return resolved


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("rebotarm_bringup"))
    profile_path = bringup_share / "config" / "visual_grasp_hardware.yaml"
    launch_arguments = _load_hardware_profile(profile_path, anchor=Path(__file__))

    return LaunchDescription(
        [
            DeclareLaunchArgument("channel"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup_share / "launch" / "visual_grasp_system.launch.py")
                ),
                launch_arguments={
                    **launch_arguments,
                    "channel": LaunchConfiguration("channel"),
                }.items(),
            ),
        ]
    )
