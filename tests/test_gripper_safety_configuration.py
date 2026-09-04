from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SAFETY_DEFAULTS = {
    "hardware_feedback_rate_hz": 50.0,
    "gripper_position_torque_cap_nm": 1.0,
    "gripper_position_max_speed_rad_s": 0.5,
    "gripper_position_timeout_margin_sec": 1.5,
    "gripper_feedback_stale_timeout_sec": 0.15,
}


def _tree(relative_path: str) -> ast.AST:
    return ast.parse((ROOT / relative_path).read_text(encoding="utf-8-sig"))


def _declared_launch_defaults(tree: ast.AST) -> dict[str, object]:
    defaults: dict[str, object] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "DeclareLaunchArgument" or not node.args:
            continue
        name = ast.literal_eval(node.args[0])
        for keyword in node.keywords:
            if keyword.arg == "default_value":
                try:
                    defaults[name] = ast.literal_eval(keyword.value)
                except (ValueError, TypeError):
                    pass
    return defaults


def _dict_string_keys(tree: ast.AST) -> set[str]:
    keys: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key in node.keys:
            if isinstance(key, ast.Constant) and isinstance(key.value, str):
                keys.add(key.value)
    return keys


def test_real_hardware_launches_expose_and_forward_gripper_safety_defaults() -> None:
    for relative_path in (
        "src/rebotarm_bringup/launch/driver_only.launch.py",
        "src/rebotarm_bringup/launch/interactive_system.launch.py",
        "src/rebotarm_bringup/launch/moveit_hardware.launch.py",
        "src/rebotarm_bringup/launch/rebotarm_app.launch.py",
        "src/rebotarm_bringup/launch/visual_grasp_system.launch.py",
    ):
        tree = _tree(relative_path)
        defaults = _declared_launch_defaults(tree)
        parameter_keys = _dict_string_keys(tree)

        for name, expected in SAFETY_DEFAULTS.items():
            assert float(defaults[name]) == expected, relative_path
            assert name in parameter_keys, relative_path


def test_controller_declares_gripper_safety_defaults() -> None:
    tree = _tree("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")
    declared: dict[str, object] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != "declare_parameter" or len(node.args) < 2:
            continue
        if isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
            try:
                declared[node.args[0].value] = ast.literal_eval(node.args[1])
            except (ValueError, TypeError):
                pass

    assert {name: declared[name] for name in SAFETY_DEFAULTS} == SAFETY_DEFAULTS
