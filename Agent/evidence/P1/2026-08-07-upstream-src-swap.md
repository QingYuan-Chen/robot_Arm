# P1 upstream MuJoCo source swap 验证

日期：2026-08-07
分支：`codex/mujoco-sim-landing`
上游：`huangbinai/robotarm_ros2` `main@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`

## 变更边界

- 切换前的 `src/rebotarm_simulation` 以字节保持方式保存到
  `third_party/rebotarm_simulation_current_baseline/`。
- `ARCHIVE_MANIFEST.json` 校验归档文件的路径、大小和 SHA-256；归档不参与
  默认 colcon source。
- 上游 `models/`、`config/`、核心 `mujoco_sim.py`、ROS node、viewer、
  trajectory sampler 和 launch 已进入 active `src/rebotarm_simulation`。
- 旧项目 API 的必要 compatibility modules、`rebotarm_mujoco_adapter` 和
  `simulation_backend:=current` fallback 保留；默认 launch 不再跨进程调用
  `third_party` 快照。

## 验证证据

| 检查 | 结果 |
|---|---|
| `python3 -m pytest tests/test_simulation_source_swap.py -q` | 3 passed |
| `python3 -m pytest tests/test_package_layering.py -q` | 18 passed |
| `python3 -m pytest tests -q` | 516 passed, 13 skipped |
| required compileall（layered packages + bringup launch + simulation） | passed |
| `colcon build --symlink-install --packages-select rebotarm_bringup rebotarm_moveit_config rebotarm_simulation` | 3 packages finished |
| `PYTHONPATH=src/rebotarm_simulation ... -m rebotarm_simulation.mujoco_cli --headless --steps 5` | passed; finite state, simulation time `0.01` |
| `... -m rebotarm_simulation.mujoco_health --skip-renderer` | `ok=true`, MuJoCo `3.3.0`, 8 joints / 8 actuators |
| `ros2 launch rebotarm_simulation mujoco_sim.launch.py` + six-joint action | action `error_code=0`, status `SUCCEEDED`; joint states published |
| `ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py simulation_backend:=current` | current adapter ready; MoveIt started with fake joint states disabled |
| `git diff --check` | passed |

所有 ROS 验证均使用独立 `ROS_DOMAIN_ID`，仅启动仿真节点；没有连接、使能或发送任何实机命令。

## 保留风险

- 上游仓库根目录缺少可直接核验的 LICENSE，授权状态仍为 `PROVISIONAL`，
  因此该集成仅用于当前本地工作区，未经确认不对外再分发。
- 上游原始测试仍有一个 `pytest.approx` 直接比较嵌套 `mapping[str, tuple]`
  的断言缺陷；该问题不在 runtime，未修改上游快照测试。
- `current` fallback 的 joint4-6 tracking/contact/calibration 仍未完成，
  不影响默认 upstream backend 的本轮 source swap 验收。
