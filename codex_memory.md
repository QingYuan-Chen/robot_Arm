# reBotArm 工程记忆

本文用于记录本仓库已经完成的工程改造、环境基线、验证结果和后续工作。
后续修改工程前，应先阅读本文件、`AGENTS.md`、`CONTEXT.md` 和
`docs/architecture.md`。

## 当前工程

- 仓库：`https://github.com/QingYuan-Chen/rebot_Arm.git`
- 默认分支：`main`
- 本机目录：`/home/a/project/rebot_Arm`
- 系统：Ubuntu 24.04
- ROS：ROS 2 Jazzy
- Python：3.12
- GPU：NVIDIA GeForce RTX 5060 Laptop GPU，8 GB 显存

## 2026-06-12 已完成

### 1. 创建个人工程版本

- 将原项目整理为个人仓库。
- 配置 GitHub 远程仓库并推送 `main` 分支。
- 当前已推送的关键提交：
  - `921588b Import reBotArm ROS 2 workspace`
  - `960e086 Configure local runtime dependencies`
  - `63c6efe Add Ubuntu native YOLO vision setup`

### 2. ROS 2 和运行依赖

已安装并验证：

- MoveIt 2.12.4
- Pinocchio 3.9.0
- `joint-state-publisher`
- `tf-transformations`
- `vcstool`
- `motorbridge` 0.4.6

厂商控制 SDK 已克隆到本机忽略目录：

```text
third_party/reBotArm_control_py
```

固定提交：

```text
6a49302804f25e624995e771acb6d61896d1856d
```

相关工程文件：

- `requirements-runtime.txt`
- `rebotarm_dependencies.repos`
- `third_party/COLCON_IGNORE`
- `docs/local_setup_zh.md`

### 3. 基础本地化

- 默认网络服务地址改为 `127.0.0.1`。
- 清除了旧工程中的 `/home/u24` 运行路径依赖。
- 旧版 `ordinary_grasp` 改为可选依赖，默认不启动。
- 保留 Windows HTTP 视觉链路作为兼容和备用方案。
- ROS 工作区 12 个包已成功构建。
- MoveIt 无硬件模式已运行到可以开始规划的状态。

### 4. Ubuntu 原生 Gemini2 和 YOLO

已建立 Ubuntu 原生视觉链路：

```text
Gemini2 USB
-> pyorbbecsdk
-> rebotarm_vision_node
-> Ultralytics YOLO/CUDA
-> ROS 2 topics
```

视觉虚拟环境：

```text
/home/a/project/rebot_Arm/.venv-vision
```

该环境通过以下方式创建：

```bash
python3 -m venv --system-site-packages .venv-vision
```

使用 `--system-site-packages` 是为了复用 ROS 2 Jazzy 的 `rclpy` 和
`cv_bridge`，同时隔离 PyTorch、Ultralytics 和 Orbbec SDK。当前不使用
Conda 运行 ROS 视觉节点，以避免 Python、NumPy、OpenCV、Qt 和动态库冲突。

固定的视觉依赖：

- PyTorch 2.11.0+cu128
- Torchvision 0.26.0+cu128
- Ultralytics 8.4.66
- `pyorbbecsdk2` 2.0.18
- NumPy 1.26.4
- OpenCV Python 4.11.0.86
- SciPy 1.11.4

重要兼容结论：

- `pyorbbecsdk2` 2.1.1 在 Python 3.12 上要求 NumPy 2。
- ROS 2 Jazzy 的 apt 版 `cv_bridge` 使用 NumPy 1 ABI。
- 因此当前工程固定 `pyorbbecsdk2==2.0.18` 和 `numpy==1.26.4`。
- 不要直接升级到 NumPy 2，否则 `cv_bridge` 可能导入失败或运行崩溃。

新增的主要文件：

- `requirements-vision.txt`
- `src/rebotarm_vision/config/camera_ubuntu.yaml`
- `src/rebotarm_vision/launch/vision_ubuntu.launch.py`
- `tools/setup_ubuntu_vision.sh`
- `tools/install_orbbec_udev_rules.sh`
- `tools/run_ubuntu_vision.sh`
- `docs/ubuntu_vision_setup_zh.md`
- `tests/test_ubuntu_vision_setup.py`

默认 Ubuntu 视觉配置：

- Gemini2 彩色流：`1280x720 @ 30 FPS`
- Gemini2 深度流：`1280x720 @ 30 FPS`
- 启用硬件深度对齐
- 模型：`src/rebotarm_vision/models/yolo11n-seg.pt`
- 推理设备：NVIDIA GPU `device=0`
- 本地 YOLO 检测开启
- 网络 YOLO 检测关闭
- 标注图发布开启

启动命令：

```bash
./tools/run_ubuntu_vision.sh
```

临时使用 CPU：

```bash
./tools/run_ubuntu_vision.sh yolo_device:=cpu
```

### 5. 已完成验证

- `torch.cuda.is_available()` 返回 `True`。
- PyTorch 已识别 RTX 5060 Laptop GPU。
- CUDA capability 为 `(12, 0)`。
- `yolo11n-seg.pt` 已在 GPU 上完成测试图推理。
- 原 Windows 使用的 `tools/yolo26s-seg.pt` 也已在 GPU 上完成测试图推理。
- `rclpy`、`cv_bridge`、OpenCV、Ultralytics 和 Orbbec SDK 可在同一虚拟环境导入。
- `rebotarm_vision_node` 的安装入口使用 `.venv-vision/bin/python`。
- YOLO 模型已随 `rebotarm_vision` ROS 包安装。
- Ubuntu 启动链路已运行到 Orbbec `Pipeline()`。
- 完整测试结果：`432 passed`。
- 包分层测试结果：`18 passed`。
- Python `compileall` 检查通过。

## 当前未完成和硬件边界

当前没有检测到 Gemini2 USB 相机，因此启动原生视觉节点时最终提示：

```text
RuntimeError: No device found
```

这不是 Python、CUDA 或 ROS 环境错误，而是当前未连接相机。

接入 Gemini2 后还需要执行一次：

```bash
./tools/install_orbbec_udev_rules.sh
```

该命令会写入 `/etc/udev/rules.d/99-obsensor-libusb.rules`，需要人工输入
sudo 密码。完成后应拔插相机，再验证彩色流、深度流、相机内参和检测话题。

## 后续优先事项

1. 接入 Gemini2，安装 udev 规则并确认 `lsusb` 能识别 Orbbec 设备。
2. 验证 `/camera/color/image_raw` 和 `/camera/depth/image_raw` 的帧率与分辨率。
3. 验证 `/grasp/detections` 和 `/camera/color/annotated`。
4. 根据真实相机标定结果更新内参和 `handeye.yaml`。
5. 完成相机到机械臂的手眼标定。
6. 在无硬件运动或低速模式下验证视觉抓取流程。
7. 真机执行前核对电机型号、关节方向、零位、软限位和控制增益。

## 2026-06-16 MuJoCo 仿真落地

### 1. 同型号 MuJoCo 参考资产

已拉取并验证同型号 B601-DM MuJoCo 参考仓库：

```text
third_party/reBotArm_develop_hjx
```

该目录被 `.gitignore` 的 `third_party/*` 规则忽略，作为本机参考资产使用，
不直接纳入主工程源码。

MuJoCo 专用虚拟环境：

```text
third_party/rebotarm_mujoco_venv
```

当前主工程新增固定依赖文件：

```text
requirements-mujoco.txt
```

核心依赖：

```text
mujoco==3.3.0
numpy
pyyaml
jinja2
typeguard
```

其中 `pyyaml`、`jinja2` 和 `typeguard` 用于让当前能看到部分 ROS Python 包的
MuJoCo venv 通过 `pip check`。

### 2. 主工程 MuJoCo 生成和验证工具

MuJoCo 落地代码放入已有仿真包 `rebotarm_simulation`，不放入
`rebotarmcontroller` 或 `rebotarm_interactive_control`。

新增主要文件：

```text
src/rebotarm_simulation/rebotarm_simulation/mujoco_model_profile.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_runner.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_cli.py
docs/mujoco_sim.md
tests/test_mujoco_model_profile.py
tests/test_mujoco_headless.py
```

新增命令入口：

```text
rebotarm_mujoco
```

### 3. visual / collision 分离

主工程不直接修改参考仓库 XML，而是从参考 XML 生成物理版模型。

生成器会：

- 将原 STL mesh geom 标为 visual-only：
  - `contype="0"`
  - `conaffinity="0"`
  - `group="1"`
- 为 base、link1-link6、gripper_base 和 finger pad 新增简化 collision geom。
- finger pad 使用独立 box collision 和独立摩擦/接触参数。
- 将 mesh 和 texture 资源路径改为绝对路径，保证生成 XML 可以从
  `build/mujoco_models/` 加载。

### 4. 电机参数初始保守化

初始 actuator 参数按主工程 URDF effort/range 对齐：

```text
joint1: ctrlrange -2.8 2.8,   forcerange -27 27, kp 270
joint2: ctrlrange -3.14 0,    forcerange -27 27, kp 270
joint3: ctrlrange -3.14 0,    forcerange -27 27, kp 270
joint4: ctrlrange -1.87 1.57, forcerange -7 7,   kp 70
joint5: ctrlrange -1.57 1.57, forcerange -7 7,   kp 70
joint6: ctrlrange -3.14 3.14, forcerange -7 7,   kp 70
gripper: ctrlrange 0 0.045,   forcerange -20 20, kp 600
```

### 5. 已完成验证

参考 XML 基线验证：

```text
reBot-DevArm_fixend.xml: nq=6 nv=6 nu=6 finite=True contacts=1
reBot-DevArm_gripper.xml: nq=8 nv=8 nu=7 finite=True contacts=1
sim_reBot_grasp.xml: nq=15 nv=14 nu=7 finite=True contacts=5
```

主工程生成模型验证：

```text
python3 -m pytest tests/test_mujoco_model_profile.py -q
4 passed
```

生成物理版模型：

```text
build/mujoco_models/reBot-DevArm_gripper_physics.xml
build/mujoco_models/sim_reBot_grasp_physics_robot_include.xml
build/mujoco_models/sim_reBot_grasp_physics.xml
```

生成模型 smoke：

```text
nq=8 nv=8 nu=7 finite=True contacts=2 sim_time=1.000
```

`joint2` 阶跃响应：

```text
target=-0.6000
final=-0.5980
max_abs_error=0.1997
rms_error=0.0389
max_abs_velocity=0.7733
max_abs_actuator_force=27.0000
sim_time=3.000
```

基础抓取场景稳定性：

```text
finite=True
box_height_m=0.01999994753282892
max_contacts=6
final_contacts=6
sim_time=3.000
```

MuJoCo venv 依赖健康检查：

```text
No broken requirements found.
```

### 6. 后续边界

当前已经完成模型生成、visual/collision 分离、保守电机参数、headless smoke、
阶跃响应和基础抓取场景稳定性验证。

尚未完成：

- ROS 2 MuJoCo adapter；
- MoveIt 轨迹接入 MuJoCo `mj_step()`；
- 多关节轨迹 RMS tracking benchmark；
- 真机实测电机响应校准；
- 多物体抓取成功率 benchmark；
- wrist camera RGB-D 输出与 `rebotarm_vision` 对齐。

## 2026-06-16 MuJoCo ROS 2 前四阶段接入

### 1. 新增独立 ROS adapter

已采纳独立节点方案，不破坏原来的 RViz fake sim：

```text
src/rebotarm_simulation/rebotarm_simulation/mujoco_adapter_core.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_metrics.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_ros_adapter_node.py
src/rebotarm_simulation/launch/mujoco_moveit_sim.launch.py
```

新增 console script：

```text
rebotarm_mujoco_adapter
```

新增 launch 会 include 原 MoveIt demo，并传入：

```text
use_fake_joint_states=false
```

因此 MoveIt/RViz 使用 MuJoCo adapter 发布的 `/rebotarm/joint_states`，
再由 `GripperVisualJointStateNode` 合成 `/rebotarm/visual_joint_states`。

### 2. 已接入 ROS 接口

MuJoCo adapter 提供：

```text
/rebotarm/follow_joint_trajectory
/rebotarm/joint_states
/rebotarm/trajectory_stop
/rebotarm/gripper/set
/rebotarm/gripper/state
```

MoveIt 控制器配置仍使用原来的：

```text
/rebotarm/follow_joint_trajectory
```

### 3. 轨迹指标

执行 `FollowJointTrajectory` 后会输出：

```text
build/mujoco_runs/latest/trajectory_metrics.csv
build/mujoco_runs/latest/summary.json
```

当前一次 ROS action 验证结果：

```text
Goal accepted
Goal finished with status: SUCCEEDED
error_string: MuJoCo trajectory finished
sample_count: 2202
joint_count: 6
max_abs_error: 0.19705242043936322
rms_error: 0.030329246215821216
max_abs_velocity: 2.7732680989480762
max_abs_actuator_force: 27.0
```

### 4. 夹爪接入和模型修正

`/rebotarm/gripper/set` 已接入 MuJoCo gripper actuator。

调试发现第一版 finger pad collision 会让左右指腹自碰撞，导致夹爪空载无法闭合。
已在生成器中为以下 body 增加 contact exclude：

```text
left_finger_link <-> right_finger_link
```

夹爪 actuator 当前参数：

```text
ctrlrange 0 0.045
forcerange -20 20
kp 600
kv 60
```

当前服务验证：

```text
ros2 service call /rebotarm/gripper/set ... position=0.03
/rebotarm/gripper/state position=0.030273087383237633
```

### 5. 运行环境注意

MuJoCo adapter 需要同一个 Python 同时看到 ROS 2 和 MuJoCo。当前本机
`third_party/rebotarm_mujoco_venv/pyvenv.cfg` 已改为：

```text
include-system-site-packages = true
```

新建环境应使用：

```bash
python3 -m venv --system-site-packages third_party/rebotarm_mujoco_venv
```

运行前必须：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果手动用 Python 模块运行，不要覆盖 ROS 的 `PYTHONPATH`，应使用：

```bash
PYTHONPATH=src/rebotarm_simulation:$PYTHONPATH
```

## 工程约束

- 新实现必须遵守 `AGENTS.md` 中的包边界。
- 硬件访问放在 `rebotarmcontroller`。
- 轨迹、MoveIt、碰撞和运行时轨迹保护放在 `rebotarm_motion`。
- 示教工作流放在 `rebotarm_teach`。
- 操作输入适配放在 `rebotarm_teleop`。
- Web UI 和 HTTP/SSE 放在 `rebotarm_dashboard`。
- 感知、深度、检测和抓取候选放在 `rebotarm_vision`。
- 不要向 `rebotarm_interactive_control` 添加新实现逻辑。
- `rebotarm_interactive_control` 只保留旧导入路径和旧命令的兼容包装。

## 更新规则

每次完成一项可验证的工程工作后，在本文件中追加：

- 日期
- 修改内容
- 关键文件
- 环境或版本变化
- 执行过的验证
- 尚未验证的硬件边界
- 对应 Git 提交
