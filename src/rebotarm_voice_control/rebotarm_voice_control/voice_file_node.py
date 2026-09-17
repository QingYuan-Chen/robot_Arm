"""离线音频文件 → 文本 → 指令路由的命令行入口。

职责：把一段录音文件先做在线转写，再把得到的文本交给文本指令流水线
（handle_text_command），从而复用完全相同的意图解析、任务模板展开、安全校验
与演练路由逻辑。典型用途：没有麦克风时用音频文件回归验证整条语音链路。

安全语义：与文本入口一致，只产出演练路由，不下发真实运动指令。
"""

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
    """转写音频并路由转写出的文本。

    参数：
        audio_path：本地音频文件路径。
        config_root：语音控制配置目录。
        asr_client：可注入的转写客户端（测试用）；为 None 时按环境变量自建。

    返回：包含 audio_path、transcript 与命令路由结果 command 的字典；保留
    transcript 便于人工核对"是不是听错了"导致的识别偏差。

    异常：转写配置缺失、文件不存在或转写文本无法匹配意图时向上抛出。
    """
    client = asr_client or OnlineAsrClient()
    transcript = client.transcribe_file(audio_path)
    return {
        "audio_path": str(audio_path),
        "transcript": transcript,
        "command": handle_text_command(transcript, config_root),
    }


def main() -> None:
    # 只接受一个位置参数（音频路径）；用法错误返回退出码 2。
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
