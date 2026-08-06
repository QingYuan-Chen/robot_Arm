from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


class _FakeMujoco:
    class mjtObj:
        mjOBJ_KEY = object()

    def __init__(self):
        self.calls = []

    def mj_name2id(self, _model, _obj_type, _name):
        return -1

    def mj_resetDataKeyframe(self, _model, _data, key_id):
        self.calls.append(("keyframe", key_id))

    def mj_resetData(self, _model, _data):
        self.calls.append(("reset", None))

    def mj_forward(self, _model, _data):
        self.calls.append(("forward", None))


class _ModelWithUnnamedKeyframe:
    nkey = 1


def test_reset_keyframe_uses_first_unnamed_key_when_name_is_missing():
    from rebotarm_simulation.mujoco_runner import _reset_keyframe_if_present

    mujoco = _FakeMujoco()

    _reset_keyframe_if_present(mujoco, _ModelWithUnnamedKeyframe(), object(), "0")

    assert mujoco.calls == [("keyframe", 0), ("forward", None)]
