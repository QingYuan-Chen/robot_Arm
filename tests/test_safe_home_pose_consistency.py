"""Guards on the resting posture shared by the driver, MoveIt and MuJoCo.

The arm folds at its zero pose: joint3's origin is -0.264 m in x while joint4's is
+0.2426 m, so with joints 2-5 near zero the forearm doubles back onto the upper arm
and link2/link5 close to ~1.6 mm.  MoveIt then aborts every planning request in
CheckStartStateCollision with "1 contact(s) detected : link2 - link5", which makes
the zero pose a dead end -- commandable, but never plannable out of.  These tests
pin the collision-free replacement and keep the three layers from drifting apart.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SRDF_DIR = ROOT / "src" / "rebotarm_moveit_config" / "config"
SRDF_NAMES = ("rebotarm.srdf", "reBot-DevArm_fixend.srdf")
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

SAFE_HOME = (-1.5707963267948966, -0.1, -0.2, 0.2, 0.0, 0.0)
HOME = (0.0, -0.8, -1.2, 0.6, 0.0, 0.0)
WEB_SAFE_HOME = (
    -1.549363136291504,
    0.01659393310546875,
    -0.02002716064453125,
    -0.00858306884765625,
    0.10395240783691406,
    0.00133514404296875,
)


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def _group_states(path: Path) -> dict[str, tuple[float, ...]]:
    root = ET.parse(path).getroot()
    states: dict[str, tuple[float, ...]] = {}
    for state in root.findall("group_state"):
        if state.attrib.get("group") != "arm":
            continue
        values = {
            joint.attrib["name"]: float(joint.attrib["value"])
            for joint in state.findall("joint")
        }
        states[state.attrib["name"]] = tuple(values[name] for name in ARM_JOINTS)
    return states


def test_no_srdf_names_the_self_colliding_all_zero_pose():
    for srdf_name in SRDF_NAMES:
        states = _group_states(SRDF_DIR / srdf_name)
        assert states, f"{srdf_name} declares no arm group_state"
        for name, values in states.items():
            assert any(
                abs(value) > 1e-9 for value in values
            ), f"{srdf_name} names the folded all-zero pose as {name!r}"


def test_srdfs_agree_on_safe_home_and_home():
    for srdf_name in SRDF_NAMES:
        states = _group_states(SRDF_DIR / srdf_name)
        assert states["safe_home"] == SAFE_HOME
        assert states["home"] == HOME


def test_driver_safe_home_default_matches_the_srdf_named_state():
    source = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")
    rendered = ", ".join(repr(value) for value in SAFE_HOME)
    assert f"_SAFE_HOME_JOINT_POSITIONS = ({rendered})" in source


def test_mujoco_keyframe_matches_the_srdf_safe_home_state():
    source = _read("src/rebotarm_simulation/rebotarm_simulation/mujoco_model_profile.py")
    arm = " ".join(
        ("0" if value == 0.0 else repr(value)) for value in SAFE_HOME
    )
    assert f'key.set("name", "safe_home")' in source
    assert f'key.set("qpos", "{arm} 0.04 -0.04")' in source
    assert f'key.set("ctrl", "{arm} 0.04")' in source


def test_driver_safe_home_does_not_delegate_to_the_vendor_zero_homing():
    hardware = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")
    services = _read("src/rebotarmcontroller/rebotarmcontroller/ros_services.py")
    controller = _read("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")

    # ArmEndPos.safe_home() and ArmEndPos.end() both drive to all zeros.
    assert "endpos_ctrl.safe_home()" not in services
    assert "endpos_ctrl.safe_home()" not in controller
    assert "self._endpos_ctrl.end()" not in hardware

    assert "def safe_home(" in hardware
    assert "self._hardware.safe_home(" in services
    assert "self.hardware.safe_home(" in controller


def test_configured_safe_home_target_is_validated_against_joint_limits():
    hardware = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")
    assert "def _validated_safe_home_target(" in hardware
    assert "_JOINT_POSITION_LIMITS_RAD.get(name)" in hardware
    assert "safe_home target must be finite" in hardware


def test_controller_defaults_web_safe_home_to_the_recorded_operator_pose():
    controller = _read("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")
    rendered = ",\n    ".join(repr(value) for value in WEB_SAFE_HOME)

    assert f"_WEB_SAFE_HOME_JOINT_POSITIONS = (\n    {rendered},\n)" in controller
    assert '"safe_home_joint_positions", list(_WEB_SAFE_HOME_JOINT_POSITIONS)' in controller


def test_safe_home_never_exceeds_the_vendor_homing_speed():
    hardware = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")
    assert "if not 0.0 < speed <= ctrl._home_vel:" in hardware
