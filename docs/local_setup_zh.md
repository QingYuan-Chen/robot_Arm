# reBotArm 本机环境与配置

本文记录本仓库在 Ubuntu 24.04 / ROS 2 Jazzy 上的本地运行基线。

> 当前安全状态：P0 软件门控和真实机械臂的 joint states、enable/hold/disable、失败 cleanup 已通过验收。`HardwareManager.connect()` 仍只连接并保持失能，动作目标在显式 enable 前会被拒绝。P0 通过不替代 P2-P6 的感知、标定和完整系统验收；完成后续证据前继续优先使用无硬件仿真、视觉独立验证和明确的 plan-only 检查。

## 1. 系统依赖

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-pinocchio
python3 -m pip install --user --break-system-packages -r requirements-runtime.txt
```

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
