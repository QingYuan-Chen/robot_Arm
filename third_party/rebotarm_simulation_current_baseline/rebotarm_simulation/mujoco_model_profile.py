from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
ASSET_DIR = Path(__file__).resolve().parent / "assets"
DEFAULT_GRIPPER_XML = ASSET_DIR / "rebotarm_base.xml"
DEFAULT_GRASP_SCENE_XML = ASSET_DIR / "rebotarm_grasp_scene.xml"


def _default_mesh_dir() -> Path:
    source_meshes = (
        ROOT / "src" / "rebotarm_bringup" / "description" / "meshes"
    )
    if source_meshes.is_dir():
        return source_meshes
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("rebotarm_bringup"))
            / "description"
            / "meshes"
        )
    except Exception:
        return source_meshes


@dataclass(frozen=True)
class CollisionGeom:
    body: str
    name: str
    geom_type: str
    size: str
    pos: str = "0 0 0"
    rgba: str = "0 0.7 0.1 0.24"
    friction: str | None = None
    condim: str | None = None
    solref: str | None = None
    solimp: str | None = None


@dataclass(frozen=True)
class MotorProfile:
    joint: str
    ctrlrange: str
    forcerange: str
    kp: str
    kv: str | None = None


COLLISION_GEOMS = [
    CollisionGeom("base_link", "base_link_collision", "cylinder", "0.085 0.045", "0 0 0.045"),
    CollisionGeom("link1", "link1_collision", "cylinder", "0.05 0.07", "0 0 0.025"),
    CollisionGeom("link2", "link2_collision", "box", "0.145 0.05 0.045", "-0.13 0 -0.025"),
    CollisionGeom("link3", "link3_collision", "box", "0.135 0.045 0.04", "0.12 -0.025 -0.025"),
    CollisionGeom("link4", "link4_collision", "box", "0.08 0.04 0.04", "0.055 -0.04 -0.02"),
    CollisionGeom("link5", "link5_collision", "cylinder", "0.045 0.055", "0 0 0.035"),
    CollisionGeom("link6", "link6_collision", "cylinder", "0.045 0.065", "0 0 0.06"),
    CollisionGeom("end_link", "gripper_base_collision", "box", "0.06 0.045 0.035"),
    CollisionGeom(
        "left_finger_link",
        "left_finger_pad_collision",
        "box",
        "0.014 0.006 0.035",
        "0.005 -0.052 0",
        friction="1.2 0.02 0.001",
        condim="4",
        solref="0.01 1",
        solimp="0.9 0.95 0.001",
    ),
    CollisionGeom(
        "right_finger_link",
        "right_finger_pad_collision",
        "box",
        "0.014 0.006 0.035",
        "0.005 0.052 0",
        friction="1.2 0.02 0.001",
        condim="4",
        solref="0.01 1",
        solimp="0.9 0.95 0.001",
    ),
]


MOTOR_PROFILES = [
    MotorProfile("joint1", "-2.8 2.8", "-27 27", "270", "24"),
    MotorProfile("joint2", "-3.14 0", "-27 27", "270", "24"),
    MotorProfile("joint3", "-3.14 0", "-27 27", "270", "24"),
    MotorProfile("joint4", "-1.87 1.57", "-7 7", "70", "10"),
    MotorProfile("joint5", "-1.57 1.57", "-7 7", "70", "10"),
    MotorProfile("joint6", "-3.14 3.14", "-7 7", "70", "10"),
    MotorProfile("left_finger", "0 0.045", "-20 20", "600", "60"),
]

# Isolated simulation-only calibration candidate.  This follows the upstream
# arm torque limits for joint4-6 while keeping the current position-actuator
# controller and gripper contract.  It must never be used for hardware limits.
UPSTREAM_ARM_MOTOR_PROFILES = [
    MotorProfile("joint1", "-2.8 2.8", "-27 27", "270", "24"),
    MotorProfile("joint2", "-3.14 0", "-27 27", "270", "24"),
    MotorProfile("joint3", "-3.14 0", "-27 27", "270", "24"),
    MotorProfile("joint4", "-1.87 1.57", "-12.5 12.5", "70", "10"),
    MotorProfile("joint5", "-1.57 1.57", "-12.5 12.5", "70", "10"),
    MotorProfile("joint6", "-3.14 3.14", "-12.5 12.5", "70", "10"),
    MotorProfile("left_finger", "0 0.045", "-20 20", "600", "60"),
]


ADJACENT_BODY_EXCLUDES = [
    ("base_link", "link1"),
    ("link1", "link2"),
    ("link2", "link3"),
    ("link3", "link4"),
    ("link4", "link5"),
    ("link5", "link6"),
    ("link6", "end_link"),
    ("end_link", "left_finger_link"),
    ("end_link", "right_finger_link"),
    ("left_finger_link", "right_finger_link"),
]


def build_physics_profile_tree(
    source_xml: Path = DEFAULT_GRIPPER_XML,
    *,
    include_keyframes: bool = True,
    motor_profiles: list[MotorProfile] = MOTOR_PROFILES,
) -> ET.ElementTree:
    source_xml = source_xml.resolve()
    tree = ET.parse(source_xml)
    root = tree.getroot()

    _make_asset_files_absolute(root, source_xml.parent)
    _mark_mesh_geoms_as_visual(root)
    _add_collision_geoms(root)
    _tune_actuators(root, motor_profiles)
    _add_contact_excludes(root)
    if include_keyframes:
        _add_keyframes(root)
    else:
        _remove_keyframes(root)
    return tree


def write_physics_profile(
    output_xml: Path,
    source_xml: Path = DEFAULT_GRIPPER_XML,
    *,
    include_keyframes: bool = True,
    motor_profiles: list[MotorProfile] = MOTOR_PROFILES,
) -> Path:
    output_xml = output_xml.resolve()
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = build_physics_profile_tree(
        source_xml,
        include_keyframes=include_keyframes,
        motor_profiles=motor_profiles,
    )
    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def write_grasp_scene_profile(
    output_xml: Path,
    robot_xml: Path,
    source_scene_xml: Path = DEFAULT_GRASP_SCENE_XML,
) -> Path:
    output_xml = output_xml.resolve()
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source_scene_xml.resolve())
    root = tree.getroot()
    include = root.find("include")
    if include is None:
        include = ET.SubElement(root, "include")
    include.set("file", str(robot_xml.resolve()))
    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def xml_asset_references_are_readable(xml_path: Path) -> bool:
    xml_path = Path(xml_path)
    if not xml_path.exists():
        return False
    try:
        root = ET.parse(xml_path).getroot()
    except ET.ParseError:
        return False
    for element in list(root.findall(".//mesh")) + list(root.findall(".//texture")):
        file_name = element.get("file")
        if not file_name:
            continue
        asset_path = Path(file_name)
        if not asset_path.is_absolute():
            asset_path = xml_path.parent / asset_path
        if not asset_path.exists():
            return False
    return True


def _make_asset_files_absolute(root: ET.Element, asset_dir: Path) -> None:
    mesh_dir = _default_mesh_dir()
    for mesh in root.findall(".//mesh"):
        file_name = mesh.get("file")
        if not file_name:
            continue
        asset_path = Path(file_name)
        if not asset_path.is_absolute():
            local_asset = asset_dir / asset_path
            if not local_asset.exists():
                local_asset = mesh_dir / asset_path.name
            mesh.set("file", str(local_asset.resolve()))
    for texture in root.findall(".//texture"):
        file_name = texture.get("file")
        if not file_name:
            continue
        asset_path = Path(file_name)
        if not asset_path.is_absolute():
            texture.set("file", str((asset_dir / asset_path).resolve()))


def _mark_mesh_geoms_as_visual(root: ET.Element) -> None:
    visual_index = 0
    for geom in root.findall(".//geom"):
        if geom.get("type") != "mesh":
            continue
        visual_index += 1
        if "name" not in geom.attrib:
            mesh_name = geom.get("mesh", f"mesh_{visual_index}")
            geom.set("name", f"{mesh_name}_visual")
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        geom.set("group", "1")
        geom.set("density", "0")


def _add_collision_geoms(root: ET.Element) -> None:
    bodies = {body.get("name"): body for body in root.findall(".//body") if body.get("name")}
    for spec in COLLISION_GEOMS:
        body = bodies.get(spec.body)
        if body is None or body.find(f"./geom[@name='{spec.name}']") is not None:
            continue
        geom = ET.SubElement(body, "geom")
        geom.set("name", spec.name)
        geom.set("type", spec.geom_type)
        geom.set("pos", spec.pos)
        geom.set("size", spec.size)
        geom.set("rgba", spec.rgba)
        geom.set("group", "4")
        if spec.friction is not None:
            geom.set("friction", spec.friction)
        if spec.condim is not None:
            geom.set("condim", spec.condim)
        if spec.solref is not None:
            geom.set("solref", spec.solref)
        if spec.solimp is not None:
            geom.set("solimp", spec.solimp)


def _tune_actuators(root: ET.Element, motor_profiles: list[MotorProfile]) -> None:
    profiles = {profile.joint: profile for profile in motor_profiles}
    for actuator in root.findall(".//actuator/position"):
        joint = actuator.get("joint")
        if joint not in profiles:
            continue
        profile = profiles[joint]
        actuator.set("name", "gripper" if joint == "left_finger" else joint)
        actuator.set("ctrllimited", "true")
        actuator.set("ctrlrange", profile.ctrlrange)
        actuator.set("forcelimited", "true")
        actuator.set("forcerange", profile.forcerange)
        actuator.set("kp", profile.kp)
        if profile.kv is not None:
            actuator.set("kv", profile.kv)


def _add_contact_excludes(root: ET.Element) -> None:
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    existing = {
        (exclude.get("body1"), exclude.get("body2"))
        for exclude in contact.findall("exclude")
    }
    for body1, body2 in ADJACENT_BODY_EXCLUDES:
        if (body1, body2) in existing or (body2, body1) in existing:
            continue
        exclude = ET.SubElement(contact, "exclude")
        exclude.set("body1", body1)
        exclude.set("body2", body2)


def _add_keyframes(root: ET.Element) -> None:
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")
    if keyframe.find("./key[@name='zero']") is None:
        key = ET.SubElement(keyframe, "key")
        key.set("name", "zero")
        key.set("qpos", "0 -0.4 -1.0 0.4 0 0 0.035 -0.035")
        key.set("ctrl", "0 -0.4 -1.0 0.4 0 0 0.035")
    if keyframe.find("./key[@name='home']") is None:
        key = ET.SubElement(keyframe, "key")
        key.set("name", "home")
        key.set("qpos", "0 -0.8 -1.2 0.6 0 0 0.04 -0.04")
        key.set("ctrl", "0 -0.8 -1.2 0.6 0 0 0.04")


def _remove_keyframes(root: ET.Element) -> None:
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)
