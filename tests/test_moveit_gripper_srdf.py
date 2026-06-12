from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


def _disabled_pairs(path: Path) -> set[tuple[str, str]]:
    root = ET.parse(path).getroot()
    pairs = set()
    for item in root.findall("disable_collisions"):
        link1 = item.attrib["link1"]
        link2 = item.attrib["link2"]
        pairs.add(tuple(sorted((link1, link2))))
    return pairs


def test_gripper_internal_collisions_are_disabled_in_moveit_srdf():
    config_dir = Path(__file__).resolve().parents[1] / "src" / "rebotarm_moveit_config" / "config"
    expected = {
        tuple(sorted(("end_link", "left_finger_link"))),
        tuple(sorted(("end_link", "right_finger_link"))),
        tuple(sorted(("left_finger_link", "right_finger_link"))),
    }

    for srdf_name in ("rebotarm.srdf", "reBot-DevArm_fixend.srdf"):
        assert expected.issubset(_disabled_pairs(config_dir / srdf_name))
