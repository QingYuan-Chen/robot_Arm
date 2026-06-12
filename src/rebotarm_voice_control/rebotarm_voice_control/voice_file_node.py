from __future__ import annotations

from pathlib import Path
import json
import sys

from .asr_client import OnlineAsrClient
from .text_input_node import handle_text_command


def handle_audio_file(
    audio_path: str | Path,
    config_root: str | Path,
    asr_client: OnlineAsrClient | None = None,
) -> dict:
    client = asr_client or OnlineAsrClient()
    transcript = client.transcribe_file(audio_path)
    return {
        "audio_path": str(audio_path),
        "transcript": transcript,
        "command": handle_text_command(transcript, config_root),
    }


def main() -> None:
    if len(sys.argv) != 2:
        print("usage: ros2 run rebotarm_voice_control rebotarm_voice_file <audio-path>")
        raise SystemExit(2)
    package_root = Path(__file__).resolve().parents[1]
    config_root = package_root / "config"
    try:
        result = handle_audio_file(sys.argv[1], config_root)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except Exception as exc:
        print(json.dumps({"error": type(exc).__name__, "message": str(exc)}, ensure_ascii=False))
        raise SystemExit(1)
