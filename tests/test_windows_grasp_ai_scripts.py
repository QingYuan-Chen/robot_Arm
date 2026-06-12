from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"


def _read_script(name: str) -> str:
    return (TOOLS / name).read_text(encoding="utf-8")


def test_windows_yolo_server_script_freezes_daily_camera_and_http_defaults():
    text = _read_script("windows_start_yolo_server.ps1")

    assert r"D:\anaconda3\envs\orbbec_yolo\python.exe" in text
    assert "windows_mjpeg_server.py" in text
    assert "--capture-source orbbec" in text
    assert "--host 0.0.0.0" in text
    assert "--port 8081" in text
    assert "--width 1280" in text
    assert "--height 720" in text
    assert "--depth-width 1280" in text
    assert "--depth-height 800" in text
    assert "--fps 30" in text
    assert "--depth-fps 30" in text
    assert "--depth-downsample-filter 1" in text
    assert "--graspnet-candidates-path" in text
    assert r"D:\tmp\graspnet_candidates.json" in text


def test_windows_graspnet_bridge_script_freezes_model_and_output_defaults():
    text = _read_script("windows_start_graspnet_bridge.ps1")

    assert r"D:\anaconda3\envs\graspnet\python.exe" in text
    assert "windows_graspnet_baseline_bridge.py" in text
    assert '"http://127.0.0.1:8081"' in text
    assert r"D:\tmp\graspnet_candidates.json" in text
    assert r"D:\rebot_ai_models\graspnet-baseline" in text
    assert r"D:\rebot_ai_models\graspnet-baseline\checkpoints\checkpoint-rs.tar" in text
    assert '"--server-url", $ServerUrl' in text
    assert '"--output-path", $OutputPath' in text
    assert '"--model-root", $ModelRoot' in text
    assert '"--checkpoint-path", $CheckpointPath' in text
    assert '"--backend-module", "graspnet_baseline_inference"' in text
    assert '"--device", "cuda:0"' in text
    assert "[int]$MaxGrasps = 50" in text
    assert "[double]$PollHz = 0.5" in text
    assert "[bool]$ManualTrigger" not in text
    assert "[bool]$Open3DVisualize" not in text
    assert "[int]$VisualizeTopN = 10" in text
    assert "[int]$VisualizeMaxPoints = 8000" in text
    assert "[int]$VisualizeEveryN = 10" in text
    assert "[double]$VisualizeZoom = 0.28" in text
    assert "$VisualizeCropRadiusM" not in text
    assert '"--max-grasps", $MaxGrasps' in text
    assert '"--poll-hz", $PollHz' in text
    assert "--manual-trigger" in text
    assert "--open3d-visualize" in text
    assert '"--visualize-top-n", $VisualizeTopN' in text
    assert '"--visualize-max-points", $VisualizeMaxPoints' in text
    assert '"--visualize-every-n", $VisualizeEveryN' in text
    assert '"--visualize-zoom", $VisualizeZoom' in text
    assert '"--visualize-crop-radius-m", 0' in text


def test_windows_grasp_ai_stack_script_starts_both_fixed_entrypoints():
    text = _read_script("windows_start_grasp_ai_stack.ps1")

    assert "windows_start_yolo_server.ps1" in text
    assert "windows_start_graspnet_bridge.ps1" in text
    assert "Start-Process" in text
    assert "Set-Location" in text
    assert "[int]$MaxGrasps = 50" in text
    assert "-MaxGrasps" in text
    assert "-VisualizeTopN" in text
    assert "-VisualizeMaxPoints" in text
    assert "-Open3DVisualize" not in text
    assert "-ManualTrigger" not in text
    assert "-VisualizeEveryN" not in text
    assert "CloudMode" not in text
