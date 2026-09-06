# reBotArm 本机环境与配置

本文记录本仓库在 Ubuntu 24.04 / ROS 2 Jazzy 上的本地运行基线。

> 当前安全状态：P0 软件门控和真实机械臂的 joint states、enable/hold/disable、失败 cleanup 已通过验收。`HardwareManager.connect()` 仍只连接并保持失能，动作目标在显式 enable 前会被拒绝。P0 通过不替代 P2-P6 的感知、标定和完整系统验收；完成后续证据前继续优先使用无硬件仿真、视觉独立验证和明确的 plan-only 检查。

## 1. 系统依赖

根目录五份 `requirements-*.txt` 按运行环境和安装方式拆分，不应全部安装到
同一个 Python 环境，也不是五份重复清单：

| 清单 | 用途 | 环境 / 安装入口 |
| --- | --- | --- |
| `requirements-runtime.txt` | 控制器 MotorBridge 基础依赖，随后必须安装本地审查补丁 | 系统 Python；本节命令 |
| `requirements-vision.txt` | 相机、YOLO、OpenCV，固定 NumPy 1.x 兼容 ROS cv_bridge | `.venv-vision`；`tools/setup_ubuntu_vision.sh` |
| `requirements-graspnet.txt` | GraspNet 抓取候选、点云及固定 PyTorch/CUDA 依赖 | `.venv-graspnet`；`tools/setup_ubuntu_graspnet.sh` |
| `requirements-mujoco.txt` | MuJoCo 仿真与模型工具 | `third_party/rebotarm_mujoco_venv`；见 `mujoco_sim.md` |
| `requirements-tensorrt.txt` | TensorRT 推理扩展，单独以 `--no-deps` 安装以避免拉取额外 CUDA 工具链 | 与 vision 共用环境；由视觉安装脚本调用 |

GraspNet 与 MuJoCo 的 PyYAML 固定版本分别为6.0.1和6.0.3；直接合并会产生
版本冲突。TensorRT 是视觉环境的独立安装步骤，不是第五个虚拟环境。
这些文件保留当前安装契约，本次文档清理不改变依赖版本或已安装环境。

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-pinocchio
python3 -m pip install --user --break-system-packages -r requirements-runtime.txt
```

`requirements-runtime.txt` 中固定的 `motorbridge==0.4.6` 只用于 bootstrap /
基础依赖，原始 PyPI 包没有本控制器所需的逐电机反馈 sequence，不能证明读到的是
新反馈帧。先安装 Rust/Cargo、Git、Python venv 等构建工具，然后从仓库根目录执行：

```bash
# 只构建并在临时 venv 验证，不改用户 Python
python3 tools/setup_motorbridge_fresh_feedback.py --build-only

# 显式安装已验证的 0.4.6+rebotarm.2 用户包
python3 tools/setup_motorbridge_fresh_feedback.py --install-user

# 启动 controller 前的 fail-closed 检查；不联网、不构建、不访问硬件
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
```

脚本固定上游 commit
`38b8a5681887514b301dbcab96e01a473cbd7173`，只接受仓库内已审查的 source patch，
并同时构建 `motor_abi`、`ws_gateway` 和 wheel。`--check-installed` 必须报告
`version=0.4.6+rebotarm.2 feedback_sequence=true`；否则不要启动真机 controller。
此版本还校验达妙反馈的 CAN ID、电机 ID 和完整 DLC，并要求置零前主动查询到
新的 status0 反馈；调用 disable 或新建句柄都不能代替状态确认。置零 API 不会
自动失能，调用方仍需完成置零后的新帧验收。构建保留旧版本 wheel 以便回退。
如需回退用户包，可执行
`python3 -m pip install --user --break-system-packages --force-reinstall motorbridge==0.4.6`。

厂商 SDK 使用仓库根目录的 `rebotarm_dependencies.repos` 固定版本：

```bash
mkdir -p third_party
vcs import third_party < rebotarm_dependencies.repos
```

也可以直接克隆：

```bash
git clone https://github.com/huangbinai/rebotarm_control.git \
  third_party/reBotArm_control_py
git -C third_party/reBotArm_control_py checkout \
  6a49302804f25e624995e771acb6d61896d1856d
```

`third_party/` 是本机依赖目录，不提交到主仓库。
仓库保留 `third_party/COLCON_IGNORE`，避免 `colcon` 扫描厂商 SDK 内部附带的
ROS 1 `catkin` 示例包。

## 2. 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. 硬件配置

接入真机前必须按实际设备确认：

- `src/rebotarm_bringup/config/arm.yaml`：通信通道、电机 ID、型号和控制增益
- `src/rebotarm_bringup/config/gripper.yaml`：夹爪 ID、型号和控制增益
- `src/rebotarm_moveit_config/config/joint_limits.yaml`：规划速度、加速度和 jerk 限制
- `src/rebotarm_moveit_config/config/rebotarm.urdf`：几何、关节方向和物理模型
- `src/rebotarm_moveit_config/config/rebotarm.srdf`：规划组、命名姿态和碰撞矩阵

不要在没有核对电机型号和机械限位时修改或提高控制增益。

夹爪闭合端允许等效 `1 mm` 的反馈偏差：按 `5 rad ↔ 90 mm` 换算为
`0..0.0555555556 rad`（约3.18°），原始角度保留，显示开口为0；这不是
软件零点补偿，也不扩大运动目标（仍不大于0）。超过该容差仍拒绝；张开端
只允许既有量化容差。显式置零验收仍为两个反馈刻度约0.000763rad，不能复用
1 mm的闭合反馈容差。

## 4. 视觉配置

Ubuntu 物理机直连 Gemini2 并在本机运行 CUDA YOLO 时，使用：

```bash
./tools/setup_ubuntu_vision.sh
./tools/install_orbbec_udev_rules.sh
./tools/run_ubuntu_vision.sh
```

完整说明见 `docs/ubuntu_vision_setup_zh.md`。

网络备用链路仍使用 `src/rebotarm_vision/config/camera.yaml`，默认连接本机：

```text
http://127.0.0.1:8081
```

如果相机和 AI 服务运行在另一台机器，将该文件中的 URL 改为对应主机地址，
或者复制配置文件并在启动时覆盖：

```bash
ros2 launch rebotarm_vision vision.launch.py \
  camera_config:=/absolute/path/to/my_camera.yaml
```

手眼标定文件也可以独立覆盖：

```bash
ros2 launch rebotarm_vision vision.launch.py \
  handeye_config:=/absolute/path/to/my_handeye.yaml
```

旧的 `ordinary_grasp` 算法不是本仓库依赖，默认不启动。安装对应旧工程后显式启用：

```bash
ros2 launch rebotarm_vision vision.launch.py \
  start_ordinary_grasp:=true \
  ordinary_grasp_root:=/absolute/path/to/rebot_grasp
```

## 5. 安全顺序

1. 先运行 MoveIt/RViz 或 MuJoCo 无硬件演示，确认没有打开串口或 CAN。
2. 在真机旁验证已完成的 `connect()` / `enable()` 解耦：启动驱动不得自动上力。
3. 检查串口或 CAN、关节方向、零位、软限位和 enable 失败回滚。
4. 在失能状态验证 `joint_states`，再显式 enable 并保持当前位置。
5. 使用低速度验证停止服务、单关节小角度和单个安全姿态。
6. 依次开放网页执行、示教回放、视觉 plan-only 和视觉低速执行。

完整视觉抓取的目标安全默认值是：

```text
use_hardware:=false
execution_mode:=plan_only
move_to_visual_ready_on_start:=false
shutdown_safe_home:=false
auto_enable:=false
```

这些默认值已经落地；实机执行仍必须显式选择硬件后端并调用 `/rebotarm/enable`，且必须先完成 P0 实机分级验收。

## 6. P0 Gate B/C：显式 enable、hold 与 disable

先在终端 A 只启动硬件 driver：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup driver_only.launch.py \
  channel:=/dev/ttyACM0 \
  joint_state_rate:=20.0
```

确认启动日志为 `CONNECTED_DISABLED`。终端 B 从仓库根目录运行专用验收工具：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarmcontroller p0_gate_bc_acceptance \
  --hold-seconds 10 \
  --max-position-jump-rad 0.03 \
  --max-abs-velocity-rad-s 0.05
```

工具只有在以下 preflight / 预检全部成立时才继续：

- 六轴 joint names、位置和速度有效；
- `enabled=false`、`control_loop_active=false`、`state_machine=IDLE`；
- 六个电机状态码全为 `0`，controller 没有 error code；
- 当前绝对速度没有超过验收阈值。

现场确认工作区清空、机械臂有失能防坠措施且急停可用后，按工具提示输入精确确认词：

```text
ENABLE_HOLD_TEST
```

随后工具只执行显式 enable、当前位置 hold 监控和 disable，不发送 trajectory、safe-home 或 gripper 命令。位置跳变或速度超限时会请求 `trajectory_stop` 和 `disable`；物理急停始终是软件失效时的第一停止手段。

无论通过、失败还是操作员中止，工具都会在以下目录写入 JSON 证据：

```text
Agent/evidence/P0/gate-bc-YYYYMMDD-HHMMSS.json
```

只有最终状态满足 `enabled=false`、控制循环停止、六个状态码全为 `0`，且 disable 后 joint states 继续刷新，Gate C 才能通过。
