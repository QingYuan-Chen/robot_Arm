from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"


def test_canonical_mujoco_scene_uses_bottle_target_proxy():
    root = ET.parse(SCENE).getroot()
    bottle = root.find(".//body[@name='bottle']")

    assert bottle is not None
    assert bottle.find("freejoint") is not None
    assert bottle.find("geom[@name='bottle_body']") is not None
    assert bottle.find("geom[@name='bottle_neck']") is not None
    assert bottle.find("geom[@name='bottle_cap']") is not None
    assert root.find(".//body[@name='test_cube']") is None


def test_canonical_bottle_proxy_sits_on_table_and_has_stable_home_pose():
    root = ET.parse(SCENE).getroot()
    bottle = root.find(".//body[@name='bottle']")
    assert bottle is not None
    assert bottle.attrib["pos"] == "0.28 0 0.0"

    key = root.find(".//key[@name='home']")
    assert key is not None
    qpos = [float(value) for value in key.attrib["qpos"].split()]
    # Six arm joints + two fingers, then bottle free-joint xyz + xyzw.
    assert qpos[8:15] == [0.28, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
