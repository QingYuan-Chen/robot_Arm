# P1 upstream-only closeout

日期：2026-08-07

## 最终决策

- Active MuJoCo runtime 只使用本地化的 `huangbinai/robotarm_ros2`
  `main@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`。
- 切换前的 current 实现只保存在
  `third_party/rebotarm_simulation_current_baseline/`，由
  `ARCHIVE_MANIFEST.json` 校验；没有 active adapter、legacy CLI、snapshot
  wrapper 或 backend selector。
- current joint4-6 tracking/contact 标定没有被宣称为成功；它随 current
  runtime 一同退出 P1 验收范围。
- 上游仓库根 LICENSE 仍未补齐，状态保持 `PROVISIONAL`。用户明确只在本地
  使用并保存到本地 Git，不发布、不推送含上游资产的分支。
- 自动 open → approach → close → lift → retreat 抓取序列属于 P5/P6
  系统闭环，不作为 P1 基础仿真完成声明。

## Active runtime 收口

- `setup.py` 只安装 direct upstream `rebotarm_mujoco_node`、Viewer、health、
  upstream CLI、URDF-to-MJCF 和轻量 RViz simulator；不再安装
  `rebotarm_mujoco_adapter`、`rebotarm_mujoco_legacy_cli` 或
  `rebotarm_upstream_mujoco_node`。
- `mujoco_moveit_sim.launch.py` 直接启动 package-owned
  `rebotarm_mujoco_node`；不再声明 `simulation_backend`，也没有 current
  condition 或 current adapter node。
- colcon symlink-install 不会自动删除已移除的 console scripts；本轮发现并
  精确删除三个陈旧 install scripts 后重建，最终 `ros2 pkg executables`
  不再列出旧入口。

## 最终验证

| 检查 | 结果 |
|---|---|
| upstream-only/source-swap/launch focused tests | `11 passed` |
| `python3 -m pytest tests -q` | `516 passed, 13 skipped` |
| MuJoCo/upstream/source-swap tests（pinned venv） | `62 passed, 7 skipped` |
| package layering | `18 passed` |
| required compileall | passed |
| colcon build | `rebotarm_bringup`, `rebotarm_moveit_config`, `rebotarm_simulation` passed |
| headless physics | 5 steps, finite state, simulation time `0.01` |
| health | `ok=true`, MuJoCo 3.3.0, 8 joints / 8 actuators |
| Viewer smoke | DISPLAY `:1`, 0.2 simulated seconds completed |
| ROS MoveIt wrapper | private `ROS_DOMAIN_ID=104`; only upstream node started |
| ROS trajectory | goal accepted, `error_code=0`, `SUCCEEDED` |
| joint states | 8 joint positions/velocities/efforts published |
| hardware | no hardware node, serial open, enable or real trajectory command |

## 现场与缓存清理

- 已停止本轮 ROS/MuJoCo/MoveIt 进程；复核无 `rebotarm`、`mujoco`、`move_group`
  或 `rviz2` 残留进程。
- 已复核 `/dev/ttyACM0` 无进程占用；本轮没有发送真机命令。
- 已清除源码、测试、Agent、上游快照和归档目录中的 Python bytecode 与
  `__pycache__`，以及仓库内 `.pytest_cache`；清理后计数均为 0。
- 已删除本轮生成的 3 个 `/tmp/p1_upstream_only_*.log` 临时日志。
- 按保留策略未删除 `build/`、`install/`、`log/`、证据文件或上游/当前基线归档。

Viewer smoke 输出过一次 `NV-GLX` extension warning，但 Viewer 已启动并完成
限时物理循环；该 warning 不影响本轮 headless/ROS 物理验收。

## P2 交接

下一阶段从 Gemini 2 设备枚举和只读 RGB/depth 采集开始。P2 不自动授权机械臂
运动；后续真实执行仍需遵守 P0 安全门以及 P5/P6 分级验收。
