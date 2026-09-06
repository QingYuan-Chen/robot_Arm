# 启动解释器显式配置

2026-09-06 起，MuJoCo、视觉、GraspNet 的 ROS launch 不再向上搜索工作区内的
`.venv-*` 或 `third_party/rebotarm_mujoco_venv`，也不向整组节点注入视觉
`site-packages/PYTHONPATH`。每类 Python 节点使用自己的解释器 prefix。

选择优先级：显式 launch 参数 > 对应环境变量 > `PATH` 中的 `python3`。
解释器必须具有该节点所需依赖，并能读取 source ROS/工作区后提供的 ROS 包路径。

| 节点 | 环境变量 | launch 参数 |
| --- | --- | --- |
| MuJoCo | `REBOTARM_MUJOCO_PYTHON` | 仿真包入口 `python_executable`；bringup 混合入口 `mujoco_python_executable` |
| 相机、普通候选、TCP frame、离线 YOLO | `REBOTARM_VISION_PYTHON` | `vision_python_executable` |
| in-process GraspNet | `GRASPNET_PYTHON` | `graspnet_python_executable` |

这些参数分别传递给对应进程；不会修改 MoveIt、控制器或其他节点的 Python 环境。
环境变量设置在启动前完成，路径允许放在工作区外。未设置时，依赖需安装在当前
`PATH` 所选 Python 中；不再自动切换到恰好在邻近目录找到的虚拟环境。

本机现有环境可在终端中明确选择（仅配置环境，不启动节点）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/project/rebot_Arm/install/setup.bash
export REBOTARM_MUJOCO_PYTHON=/home/a/project/rebot_Arm/third_party/rebotarm_mujoco_venv/bin/python
export REBOTARM_VISION_PYTHON=/home/a/project/rebot_Arm/.venv-vision/bin/python
export GRASPNET_PYTHON=/home/a/project/rebot_Arm/.venv-graspnet/bin/python
```

整合前的工作树分别使用 `install_decoupling` 和 `install_coupling_audit` 验证。
整合后主目录独立构建全部包，不再依赖工作树overlay。主目录重建命令为：

```bash
cd /home/a/project/rebot_Arm
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
colcon --log-base log_integration_20260906 build --base-paths src \
  --build-base build_integration_20260906 --install-base install --executor sequential
```

工作树缺失的 `tools/yolo26m-seg-fp16-b1-640-linux.engine` 已通过本地符号链接
引用主目录同名模型，使视觉包能按既有资源规则构建；模型本体未改动，该链接不应
纳入源码提交。新部署仍须按视觉包既有安装要求提供模型文件。

部署到其他机器时，改为那台机器实际安装的解释器路径即可。也可以在每次 launch
中设置表中的参数覆盖环境变量。GraspNet 模型根目录、checkpoint、相机配置和
MuJoCo 模型资源的选择方式保持原有契约；本次只解除解释器的目录布局耦合。

覆盖入口：

- `rebotarm_simulation`: `mujoco_sim.launch.py`、`mujoco_moveit_sim.launch.py`
- `rebotarm_bringup`: `visual_grasp_system.launch.py`、`visual_grasp_perception_preview.launch.py`、`mujoco_offline_perception.launch.py`、`real_perception_sim_execution.launch.py`
- `rebotarm_vision`: `vision.launch.py`、`vision_ubuntu.launch.py`

旧 `mujoco_ros_adapter_node.py` 和快照进程转发器 `upstream_backend.py` 已退出
活动源码包，`rebotarm_simulation` 不再依赖 `rebotarm_motion`。正式 ROS 入口仍是
`rebotarm_mujoco_node = rebotarm_simulation.mujoco_ros_node:main`，ROS action/service
接口不变。历史源码保留在 Git 和既有 `third_party/rebotarm_simulation_current_baseline`
归档中。模型分析、指标和独立离线工具仍使用的公共模块继续保留。

验证结果：全量测试 `761 passed, 8 skipped`，分层20通过；从当前活动源码导入
正式 MuJoCo 后端执行轨迹/停止/取消等测试55通过。三个包独立构建成功，8个已安装
launch均通过 `--show-args`；搬迁目录测试覆盖默认、环境变量和显式参数优先级。
旧模块在新overlay中不可导入。以上均为软件验证，未启动硬件或发送运动命令。
