# reBotArm 本机环境与配置

本文记录本仓库在 Ubuntu 24.04 / ROS 2 Jazzy 上的本地运行基线。

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

1. 先运行 MoveIt/RViz 无硬件演示。
2. 再检查串口或 CAN、关节方向、零位和软限位。
3. 使用低速度验证 `joint_states`、停止服务和单点轨迹。
4. 最后启用网页执行、示教回放和视觉抓取。
