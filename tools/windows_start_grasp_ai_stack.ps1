param(
    [string]$RepoRoot = "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main",
    [int]$MaxGrasps = 50,
    [int]$VisualizeTopN = 10,
    [int]$VisualizeMaxPoints = 8000
)

$ErrorActionPreference = "Stop"
Set-Location $RepoRoot

$yoloScript = Join-Path $RepoRoot "tools\windows_start_yolo_server.ps1"
$graspnetScript = Join-Path $RepoRoot "tools\windows_start_graspnet_bridge.ps1"

Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-ExecutionPolicy", "Bypass",
    "-File", $yoloScript
)

Start-Sleep -Seconds 3

Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-ExecutionPolicy", "Bypass",
    "-File", $graspnetScript,
    "-MaxGrasps", $MaxGrasps,
    "-VisualizeTopN", $VisualizeTopN,
    "-VisualizeMaxPoints", $VisualizeMaxPoints
)
