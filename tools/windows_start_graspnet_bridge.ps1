param(
    [string]$PythonExe = "D:\anaconda3\envs\graspnet\python.exe",
    [string]$RepoRoot = (Split-Path -Parent $PSScriptRoot),
    [string]$ServerUrl = "http://127.0.0.1:8081",
    [string]$OutputPath = "D:\tmp\graspnet_candidates.json",
    [string]$ModelRoot = "D:\rebot_ai_models\graspnet-baseline",
    [string]$CheckpointPath = "D:\rebot_ai_models\graspnet-baseline\checkpoints\checkpoint-rs.tar",
    [int]$MaxGrasps = 50,
    [double]$PollHz = 0.5,
    [int]$VisualizeTopN = 10,
    [int]$VisualizeMaxPoints = 8000,
    [int]$VisualizeEveryN = 10,
    [double]$VisualizePointSize = 4.0,
    [double]$VisualizeAxisSize = 0.05,
    [double]$VisualizeZoom = 0.28
)

$ErrorActionPreference = "Stop"
Set-Location $RepoRoot

$bridgeArgs = @(
    "tools\windows_graspnet_baseline_bridge.py",
    "--server-url", $ServerUrl,
    "--output-path", $OutputPath,
    "--model-root", $ModelRoot,
    "--checkpoint-path", $CheckpointPath,
    "--backend-module", "graspnet_baseline_inference",
    "--device", "cuda:0",
    "--max-grasps", $MaxGrasps,
    "--poll-hz", $PollHz,
    "--manual-trigger",
    "--open3d-visualize",
    "--visualize-top-n", $VisualizeTopN,
    "--visualize-max-points", $VisualizeMaxPoints,
    "--visualize-every-n", $VisualizeEveryN,
    "--visualize-point-size", $VisualizePointSize,
    "--visualize-axis-size", $VisualizeAxisSize,
    "--visualize-zoom", $VisualizeZoom,
    "--visualize-crop-radius-m", 0
)

& $PythonExe @bridgeArgs
