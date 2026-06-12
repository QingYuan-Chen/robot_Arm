from __future__ import annotations

from pathlib import Path
import sys

ROS2_ROOT = Path(__file__).resolve().parents[1]
SRC = ROS2_ROOT / "src" / "rebotarm_voice_control"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rebotarm_voice_control.voice_file_node import handle_audio_file


class _FakeAsrClient:
    def transcribe_file(self, audio_path):
        assert Path(audio_path).name == "cmd.wav"
        return "移动到拍照位"


def test_handle_audio_file_reuses_text_dry_run_pipeline(tmp_path):
    audio_path = tmp_path / "cmd.wav"
    audio_path.write_bytes(b"fake wav bytes")

    result = handle_audio_file(audio_path, SRC / "config", asr_client=_FakeAsrClient())

    assert result["transcript"] == "移动到拍照位"
    assert result["command"]["intent"] == "move_camera_pose"
    assert result["command"]["steps"][0]["target"] == "/rebotarm/move_to_pose"
    assert result["command"]["steps"][0]["dry_run"] is True
