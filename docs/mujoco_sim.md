# reBotArm MuJoCo 仿真落地说明

> 2026-09-06 更正：下文保留早期迁移和离线分析记录。正式 ROS 后端已经统一为
> `mujoco_ros_node.py`，旧 `mujoco_ros_adapter_node.py`、`upstream_backend.py`
> 已从活动包移除，不再支持 `simulation_backend:=current`。下文涉及旧后端的
> 命令及指标描述仅供历史追溯。当前入口及解释器配置以
> [启动解释器说明](launch_python_configuration.md) 和
> [仿真包 README](../src/rebotarm_simulation/README_mujoco.md) 为准。

> 当前正式实现位于 `src/rebotarm_simulation`，基于固定上游
> `huangbinai/robotarm_ros2@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`。
> 历史差异报告、旧实现及上游比较副本仅本机保留，不随源码发布。
> 来源与授权边界见 [`mujoco_upstream_sources.md`](mujoco_upstream_sources.md)。

本文记录本工程的 MuJoCo 仿真落地方式。MuJoCo 在本仓库中定位为
物理和离线验证层，用于接触、抓取、轨迹跟踪和策略迭代；它不直接拥有
真机通信，也不替代 `rebotarmcontroller` 的硬件安全边界。

## 目录和边界

当前默认模型来源：

```text
src/rebotarm_simulation/rebotarm_simulation/assets/rebotarm_base.xml
src/rebotarm_simulation/rebotarm_simulation/assets/rebotarm_grasp_scene.xml
src/rebotarm_moveit_config/meshes/
```

MJCF baseline 由本仓库 Apache-2.0 URDF 使用 MuJoCo 3.3.0 编译并整理，mesh
只从本仓库 `rebotarm_bringup` 读取。`third_party/reBotArm_develop_hjx` 因固定
commit 缺少许可证文件，只允许行为级观察，不得作为默认运行依赖，不复制其源码
或资产。完整来源和授权边界见 `mujoco_upstream_sources.md`。

主工程维护的是：

```text
src/rebotarm_simulation/rebotarm_simulation/mujoco_model_profile.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_runner.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_cli.py
tests/test_mujoco_model_profile.py
tests/test_mujoco_headless.py
requirements-mujoco.txt
```

`rebotarm_simulation` 只负责离线仿真、模型生成、headless 验证和后续 ROS
仿真适配。真机 SDK、串口、电机最后安全检查仍属于 `rebotarmcontroller`。

所有 MuJoCo 启动还必须满足：

```text
use_hardware:=false
不启动 rebotarmcontroller
不打开串口或 CAN
不调用真实 enable
同名 FollowJointTrajectory 只存在一个服务端
```

## Virtual RGB-D and offline plan-only / 虚拟 RGB-D 与离线规划

Virtual sensing / 虚拟传感默认关闭。启用时，active `rebotarm_mujoco_node`发布同步RGB、毫米depth、两路CameraInfo、ground-truth detection/mask，以及从`base_link`到ROS optical frame的静态TF。MuJoCo camera的`+X right,+Y up,-Z forward`通过`diag(1,-1,-1)`转换为ROS optical的`+X right,+Y down,+Z forward`。

纯仿真、无运动的组合入口：

```bash
ROS_DOMAIN_ID=100 RMW_FASTRTPS_USE_SHM=0 \
ros2 launch rebotarm_bringup mujoco_offline_perception.launch.py \
  virtual_camera_width:=320 \
  virtual_camera_height:=240 \
  virtual_camera_rate_hz:=5.0
```

该入口直接启动`.venv-graspnet`中的in-process GraspNet ROS进程，不需要8081 HTTP service；
它使用ground-truth object annotation辅助GraspNet，并只启动MoveIt IK/collision filter；
`start_visual_grasp_executor=false`、`start_motion_execution=false`、
`start_sim_trajectory_controller=false`。它不会启动Gemini 2或真实driver，也不会发送trajectory。
当前该入口是plan-only工作台，不等于包含rendered RGB YOLO与MuJoCo执行的完全离线抓取闭环。

## 环境

ROS 2 Jazzy 节点继续使用系统 Python。MuJoCo 使用独立虚拟环境，避免和 ROS
的 `rclpy`、Qt、OpenCV、OpenSSL、`libstdc++` 发生依赖冲突。

当前本机已有环境：

```bash
third_party/rebotarm_mujoco_venv/bin/python -m pip check
```

新机器可按下面方式创建：

```bash
python3 -m venv --system-site-packages third_party/rebotarm_mujoco_venv
third_party/rebotarm_mujoco_venv/bin/python -m pip install -r requirements-mujoco.txt
```

当前固定的核心依赖：

```text
mujoco==3.3.0
numpy==1.26.4
cffi==1.17.1
pyyaml==6.0.3
jinja2==3.1.6
typeguard==4.5.2
```

`pyyaml`、`jinja2`、`typeguard` 用于让当前带 ROS 可见包的 MuJoCo venv
通过 `pip check`。`mink`、QP solver、OpenCV 手势控制等属于后续可选能力，
第一阶段不纳入必需依赖。

## 生成物理版模型

不要修改第三方参考仓库。使用主工程生成器从 package-owned MJCF baseline 派生物理版 XML：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli generate \
  --robot-output build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --scene-output build/mujoco_models/sim_reBot_grasp_physics.xml
```

生成结果位于 `build/mujoco_models/`，属于可重复生成文件，不需要提交。

生成器会做这些事情：

- 将原 STL mesh geom 标记为 visual-only：
  - `contype="0"`
  - `conaffinity="0"`
  - `group="1"`
- 为主要 link 和夹爪指尖新增简化 collision geom：
  - arm link 使用 box/cylinder；
  - finger pad 使用 box；
  - collision geom 使用 `group="4"`。
- 修正 actuator 名称、控制范围和保守力矩：
  - `joint1..joint3` 使用 `forcerange="-27 27"`；
  - `joint4..joint6` 使用 `forcerange="-7 7"`；
  - `left_finger` 使用 `ctrlrange="0 0.045"`、`forcerange="-20 20"`、
    `kp="600"` 和 `kv="60"`。
- 生成两个机器人 XML：
  - 独立机器人 XML 带 `zero/home` keyframe；
  - 场景 include 版机器人 XML 不带 keyframe，避免被抓取场景的 15 维 qpos
    冲突。

## 验证命令

纯 XML 转换测试：

```bash
python3 -m pytest tests/test_mujoco_model_profile.py -q
```

机器人 headless smoke：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli smoke \
  --xml build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --seconds 1.0
```

单关节阶跃响应：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli step-response \
  --xml build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --joint joint2 \
  --target -0.6 \
  --seconds 3.0
```

基础抓取场景稳定性（当前 keyframe 会同步 home `qpos` 与 actuator `ctrl`；该命令不发送抬升轨迹）：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli grasp-benchmark \
  --xml build/mujoco_models/sim_reBot_grasp_physics.xml \
  --seconds 3.0
```

如果当前 Python 环境安装了 MuJoCo，也可以直接运行：

```bash
python3 -m pytest tests/test_mujoco_headless.py -q
```

否则该测试会自动 skip。

## 当前参数策略

第一阶段使用保守参数，不追求最快响应：

```text
joint1: ctrlrange -2.8 2.8,   forcerange -27 27, kp 270
joint2: ctrlrange -3.14 0,    forcerange -27 27, kp 270
joint3: ctrlrange -3.14 0,    forcerange -27 27, kp 270
joint4: ctrlrange -1.87 1.57, forcerange -7 7,   kp 70
joint5: ctrlrange -1.57 1.57, forcerange -7 7,   kp 70
joint6: ctrlrange -3.14 3.14, forcerange -7 7,   kp 70
gripper: ctrlrange 0 0.045,   forcerange -20 20, kp 600
```

调参顺序：

1. 保持 collision 简化模型稳定。
2. 单关节阶跃响应调 `kp/kv/damping/frictionloss`。
3. 多关节轨迹跟踪看 RMS error、最大速度和最大 actuator force。
4. 夹爪抓取单独调 finger pad 的摩擦和接触参数。

## ROS 2 adapter 状态

当前已经在 `rebotarm_simulation` 内增加 ROS 2 MuJoCo adapter：

- 发布 `/rebotarm/joint_states`；
- 接收 `/rebotarm/follow_joint_trajectory`；
- 接收 `/rebotarm/gripper/set`；
- 保留 `/rebotarm/trajectory_stop`；
- 将 MoveIt 规划轨迹送入 MuJoCo 真实 `mj_step()` 跟踪。

adapter 仍然只属于仿真包，不直接调用硬件 SDK。

## Virtual RGB-D / 虚拟RGB-D与真值标注

active `rebotarm_mujoco_node`可选发布`scene.xml`中`fixed_camera`的离屏传感器输出。该能力默认关闭，因此普通motion-only MuJoCo运行不要求EGL；启用命令为：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py \
  enable_virtual_camera:=true \
  virtual_camera_width:=640 \
  virtual_camera_height:=480 \
  virtual_camera_rate_hz:=15.0
```

输出接口：

```text
/camera/color/image_raw       sensor_msgs/msg/Image, rgb8
/camera/depth/image_raw       sensor_msgs/msg/Image, mono16 millimeters
/camera/color/camera_info     sensor_msgs/msg/CameraInfo
/camera/depth/camera_info     sensor_msgs/msg/CameraInfo
/grasp/detections             rebotarm_msgs/msg/Detection2DArray
```

默认frame为`mujoco_fixed_camera_optical_frame`。RGB、depth、CameraInfo和annotations使用同一个MuJoCo simulation timestamp。CameraInfo由MJCF相机的vertical field of view计算，使用理想pinhole/zero-distortion模型；它是仿真内参，不是Gemini 2实机标定结果。

默认annotation body为`test_cube`。bbox和mask来自MuJoCo segmentation buffer ground truth，不是YOLO推理结果。深度按米转毫米，background、无效值和`virtual_camera.max_depth_m`之外像素写0。

EGL renderer由专用worker thread独占，其create/render/destroy不跨线程；render request只保留最新一帧，避免视觉处理反压造成队列累积。该接口关闭虚拟传感器checkbox，但完整offline loop仍需补齐camera-to-base TF、offline YOLO/GraspNet/MoveIt组合与outcome matrix。

## ROS 2 / MoveIt 接入

当前默认使用固定的上游 MuJoCo ROS adapter；本仓库 package-owned adapter
仍作为显式 fallback 保留：

```text
默认：rebotarm_upstream_mujoco_node -> rebotarm_mujoco_node
回退：rebotarm_mujoco_adapter
```

它提供：

```text
/rebotarm/follow_joint_trajectory
/rebotarm/joint_states
/rebotarm/trajectory_stop
/rebotarm/gripper/set
/rebotarm/gripper/state
```

adapter 使用 MuJoCo `data.time` 作为轨迹插值和完成判断的时间基准。ROS timer
只提供 wall-clock 调度，实际每次会根据 MuJoCo `timestep` 和累计 wall delta
补足需要执行的 `mj_step()` 次数，避免模型 `timestep=0.0025` 但 ROS timer
默认 200Hz 时仿真只以 0.5 倍实时速度运行。

`FollowJointTrajectory` 的 stopped / canceled / tolerance 类 stop reason 会映射为
非 `SUCCESSFUL` 结果；`summary.json` 仍记录 stop reason 和误差统计，后续
goal/path tolerance 阈值会基于 step-response 数据继续标定。

当前还支持：

- 单点 delayed trajectory 自动在 `t=0` 插入当前关节状态，避免目标瞬间跳到终点；
- stale generated XML asset 检查，已有 XML 指向不存在 mesh/texture 时会自动重生成；
- `step-response-suite` 批量标定入口，输出每个关节的最终误差、RMS、速度和 actuator force；
- `metrics_sample_stride` 参数，用于长序列下采样写出 CSV；
- `use_mujoco_viewer` 参数，默认 `false`，需要观察 MuJoCo 原生界面时再打开；
- grasp benchmark 会输出 contact/lift/success/status，用于后续抓取质量闭环。

启动前需要 source ROS 和本工作区：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

正式 ROS 仿真使用已安装的活动包，推荐使用 launch：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py \
  use_rviz:=true \
  python_executable:=third_party/rebotarm_mujoco_venv/bin/python
```

该入口只启动 `rebotarm_mujoco_node`，不读取历史比较副本，也不再接受
`simulation_backend:=current`。仿真使用 `/clock`，MoveIt 消费者由 launch
设置 `use_sim_time=true`。不得另起第二个同名 `follow_joint_trajectory`
action server。`python_executable` 应指向本机已安装 MuJoCo 的解释器，
虚拟环境本身不随仓库发布。

无 RViz 验证：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py \
  use_rviz:=false \
  python_executable:=third_party/rebotarm_mujoco_venv/bin/python
```

直接发送一条轨迹：

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, -0.6, -1.0, 0.4, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}"
```

执行后会生成：

```text
build/mujoco_runs/latest/trajectory_metrics.csv
build/mujoco_runs/latest/summary.json
```

夹爪服务：

```bash
ros2 service call /rebotarm/gripper/set \
  rebotarm_msgs/srv/SetGripper \
  "{position: 0.03, max_effort: 1.0}"
```

查看夹爪状态：

```bash
ros2 topic echo /rebotarm/gripper/state --once
```

批量 step-response 标定：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli step-response-suite \
  --xml build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --seconds 2.0 \
  --json-output build/mujoco_runs/latest/step_response_suite.json
```

注意：当前批量标定已经暴露出 joint4 / joint6 在部分较大阶跃目标下最终误差较大。
后续如果要继续调 `forcerange`、`kp`、`kv`、`damping`，必须先决定约束来源：

- 按真实电机/减速器/电流能力保持保守参数；
- 或为了仿真演示提高力矩和增益。

这两条路线会得到不同的仿真可信度，不能混为一谈。
