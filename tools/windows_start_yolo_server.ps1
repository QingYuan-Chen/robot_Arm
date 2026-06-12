param(
    [string]$PythonExe = "D:\anaconda3\envs\orbbec_yolo\python.exe",
    [string]$RepoRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$ModelPath = "tools\yolo26s-seg.pt",
    [string]$GraspNetCandidatesPath = "D:\tmp\graspnet_candidates.json"
)

$ErrorActionPreference = "Stop"
Set-Location $RepoRoot

& $PythonExe tools\windows_mjpeg_server.py `
    --model-path $ModelPath `
    --capture-source orbbec `
    --host 0.0.0.0 `
    --port 8081 `
    --width 1280 `
    --height 720 `
    --fps 30 `
    --depth-width 1280 `
    --depth-height 800 `
    --depth-fps 30 `
    --depth-downsample-filter 1 `
    --jpeg-quality 80 `
    --conf-threshold 0.25 `
    --iou-threshold 0.45 `
    --detection-fps 15.0 `
    --classes bottle `
    --allowed-classes bottle `
    --graspnet-candidates-path $GraspNetCandidatesPath
