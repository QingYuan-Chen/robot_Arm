from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tempfile
from typing import Sequence
import xml.etree.ElementTree as ET

import mujoco
import yaml


PACKAGE_MESH_PREFIX = "package://rebotarm_bringup/description/meshes/"
JOINTS = [f"joint{index}" for index in range(1, 7)] + [
    "left_finger_joint",
    "right_finger_joint",
]


def authoritative_urdf_path(repo_root: Path) -> Path:
    return repo_root / "src/rebotarm_moveit_config/config/rebotarm.urdf"


def _model_directory(repo_root: Path) -> Path:
    return repo_root / "src/rebotarm_simulation/models/rebotarm"


def actuator_name_for_joint(joint_name: str) -> str:
    if joint_name in {f"joint{index}" for index in range(1, 7)}:
        return f"{joint_name}_torque"
    return f"{joint_name.removesuffix('_joint')}_force"


def stage_urdf(source: Path, repo_root: Path, temporary_dir: Path) -> Path:
    root = ET.parse(source).getroot()
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        {"discardvisual": "false", "fusestatic": "false", "strippath": "false"},
    )
    source_assets = _model_directory(repo_root) / "assets"
    staged_assets = temporary_dir / "assets"
    staged_assets.mkdir(parents=True, exist_ok=True)

    for link in root.findall("link"):
        for role in ("visual", "collision"):
            for index, element in enumerate(link.findall(role)):
                element.set("name", f'{link.attrib["name"]}_{role}_{index}')

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename.startswith(PACKAGE_MESH_PREFIX):
            raise ValueError(f"unsupported URDF mesh URI: {filename}")
        basename = filename.removeprefix(PACKAGE_MESH_PREFIX)
        source_mesh = source_assets / basename
        if not source_mesh.is_file():
            raise FileNotFoundError(source_mesh)
        shutil.copy2(source_mesh, staged_assets / basename)
        mesh.set("filename", f"assets/{basename}")

    staged = temporary_dir / source.name
    ET.ElementTree(root).write(staged, encoding="utf-8", xml_declaration=True)
    return staged


def generate_mjcf_bytes(repo_root: Path) -> bytes:
    source = authoritative_urdf_path(repo_root)
    with tempfile.TemporaryDirectory(prefix="rebotarm-urdf-") as directory:
        staged = stage_urdf(source, repo_root, Path(directory))
        spec = mujoco.MjSpec.from_file(str(staged))
        spec.compile()
        root = ET.fromstring(spec.to_xml())
        urdf_root = ET.parse(source).getroot()
        _configure_compiler(root)
        _classify_geoms(root)
        _add_contact_exclusions(root)
        _add_finger_coupling(root)
        _add_sites(root)
        _add_joint_dynamics(root, _load_joint_dynamics(repo_root))
        _add_actuators(root, urdf_root, repo_root)
        _add_sensors(root)
    return _canonicalize(root)


def _configure_compiler(root: ET.Element) -> None:
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    default = ET.Element("default")
    ET.SubElement(
        default,
        "geom",
        {"solref": "0.01 1", "solimp": "0.9 0.95 0.001", "friction": "0.8 0.02 0.001"},
    )
    visual = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(visual, "geom", {"contype": "0", "conaffinity": "0", "group": "2"})
    collision = ET.SubElement(default, "default", {"class": "collision"})
    ET.SubElement(
        collision,
        "geom",
        {"contype": "1", "conaffinity": "1", "group": "3", "rgba": "0.2 0.5 0.8 0.12"},
    )
    compiler_index = list(root).index(compiler)
    root.insert(compiler_index + 1, default)


def _classify_geoms(root: ET.Element) -> None:
    for body in root.findall("worldbody//body"):
        body_name = body.attrib["name"]
        mesh_counts: dict[str, int] = {}
        for geom in body.findall("geom"):
            mesh = geom.attrib["mesh"]
            index = mesh_counts.get(mesh, 0)
            mesh_counts[mesh] = index + 1
            is_visual = geom.attrib.get("contype") == "0"
            role = "visual" if is_visual else "collision"
            geom.set("name", f"{body_name}_{mesh}_{role}")
            geom.set("class", role)
            geom.set("contype", "0" if is_visual else "1")
            geom.set("conaffinity", "0" if is_visual else "1")
            geom.set("group", "2" if is_visual else "3")
            geom.attrib.pop("density", None)
            if not is_visual:
                geom.attrib.pop("rgba", None)
            if not is_visual and mesh in {"left_finger", "right_finger"}:
                geom.set("friction", "1.2 0.02 0.001")


def _add_contact_exclusions(root: ET.Element) -> None:
    contact = ET.SubElement(root, "contact")
    for body1, body2 in (
        ("base_link", "link1"),
        ("link1", "link2"),
        ("link2", "link3"),
        ("link3", "link4"),
        ("link4", "link5"),
        ("link5", "link6"),
        ("link6", "end_link"),
        ("end_link", "left_finger_link"),
        ("end_link", "right_finger_link"),
        ("link2", "link4"),
        ("link4", "link6"),
        ("left_finger_link", "right_finger_link"),
    ):
        ET.SubElement(contact, "exclude", {"body1": body1, "body2": body2})


def _add_finger_coupling(root: ET.Element) -> None:
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "finger_coupling",
            "joint1": "right_finger_joint",
            "joint2": "left_finger_joint",
            "polycoef": "0 -1 0 0 0",
        },
    )


def _add_sites(root: ET.Element) -> None:
    end_link = root.find('.//body[@name="end_link"]')
    if end_link is None:
        raise ValueError("converted MJCF is missing end_link")
    ET.SubElement(end_link, "site", {"name": "ee_site", "pos": "-0.105 0 0", "size": "0.008"})
    ET.SubElement(
        end_link,
        "site",
        {"name": "wrist_camera_mount", "pos": "-0.04 0 0.04", "quat": "1 0 0 0", "size": "0.005"},
    )


def _load_joint_dynamics(repo_root: Path) -> dict[str, dict[str, float]]:
    path = repo_root / "src/rebotarm_simulation/config/motor_control_calibration.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    dynamics = payload.get("model_dynamics")
    if not isinstance(dynamics, dict):
        raise ValueError(f"expected model_dynamics mapping in {path}")
    result: dict[str, dict[str, float]] = {}
    for joint_name in JOINTS:
        values = dynamics.get(joint_name)
        if not isinstance(values, dict):
            raise ValueError(f"missing model_dynamics for {joint_name}")
        result[joint_name] = {
            "damping": float(values["damping"]),
            "armature": float(values["armature"]),
            "frictionloss": float(values["frictionloss"]),
        }
    return result


def _add_joint_dynamics(root: ET.Element, dynamics: dict[str, dict[str, float]]) -> None:
    for joint in root.findall("worldbody//joint"):
        name = joint.attrib["name"]
        if name not in dynamics:
            continue
        for key, value in dynamics[name].items():
            joint.set(key, f"{value:g}")


def _add_actuators(root: ET.Element, urdf_root: ET.Element, repo_root: Path) -> None:
    limits = {
        joint.attrib["name"]: joint.find("limit")
        for joint in urdf_root.findall("joint")
        if joint.attrib.get("name") in JOINTS
    }
    actuator = ET.SubElement(root, "actuator")
    finger_force_limit = _load_gripper_force_limit(repo_root)
    for joint_name in JOINTS:
        limit = limits[joint_name]
        if limit is None:
            raise ValueError(f"URDF joint has no limit: {joint_name}")
        effort = (
            finger_force_limit
            if joint_name in {"left_finger_joint", "right_finger_joint"}
            else float(limit.attrib["effort"])
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": actuator_name_for_joint(joint_name),
                "joint": joint_name,
                "gear": "1",
                "ctrlrange": f"-{effort} {effort}",
                "ctrllimited": "true",
                "forcelimited": "true",
                "forcerange": f"-{effort} {effort}",
            },
        )


def _load_gripper_force_limit(repo_root: Path) -> float:
    path = repo_root / "src/rebotarm_simulation/config/motor_control_calibration.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    try:
        value = float(payload["gripper"]["finger_force_limit_n"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"expected gripper.finger_force_limit_n in {path}") from exc
    if value <= 0.0:
        raise ValueError(f"gripper.finger_force_limit_n must be positive in {path}")
    return value


def _add_sensors(root: ET.Element) -> None:
    sensor = ET.SubElement(root, "sensor")
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(sensor, "jointpos", {"name": f"{prefix}_pos", "joint": joint_name})
        ET.SubElement(sensor, "jointvel", {"name": f"{prefix}_vel", "joint": joint_name})
    for joint_name in JOINTS:
        prefix = joint_name.removesuffix("_joint")
        ET.SubElement(
            sensor,
            "actuatorfrc",
            {"name": f"{prefix}_force", "actuator": actuator_name_for_joint(joint_name)},
        )
    ET.SubElement(sensor, "framepos", {"name": "ee_position", "objtype": "site", "objname": "ee_site"})
    ET.SubElement(sensor, "framequat", {"name": "ee_orientation", "objtype": "site", "objname": "ee_site"})


def _canonicalize(root: ET.Element) -> bytes:
    for mesh in root.findall("asset/mesh"):
        mesh.attrib.pop("content_type", None)
    ET.indent(root, space="  ")
    xml = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    warning = "<!-- AUTO-GENERATED from rebotarm.urdf; do not edit robot.xml manually. -->\n"
    return (warning + xml.replace("\r\n", "\n").rstrip() + "\n").encode("utf-8")


def check_generated_model(repo_root: Path, output: Path) -> bool:
    return output.is_file() and output.read_bytes() == generate_mjcf_bytes(repo_root)


def write_generated_model(repo_root: Path, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    try:
        temporary.write_bytes(generate_mjcf_bytes(repo_root))
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)


def _default_repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_output(repo_root: Path) -> Path:
    return _model_directory(repo_root) / "robot.xml"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate reBotArm MJCF from the authoritative URDF")
    parser.add_argument("--repo-root", type=Path, default=_default_repo_root())
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    output = (args.output or _default_output(repo_root)).resolve()

    if args.check:
        if check_generated_model(repo_root, output):
            print(f"MJCF is up to date: {output}")
            return 0
        print(f"MJCF is stale; regenerate with rebotarm_urdf_to_mjcf --repo-root \"{repo_root}\"")
        return 1

    write_generated_model(repo_root, output)
    print(f"Generated MJCF: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
