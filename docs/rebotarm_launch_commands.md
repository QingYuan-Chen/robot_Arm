# reBotArm 除视觉抓取外的功能启动指令

本文是当前工作区除视觉抓取以外的统一启动指令手册，覆盖真机底层、MoveIt、网页工作台、键盘遥操作、示教、标定、独立相机/YOLO 和 MuJoCo 仿真。

视觉抓取入口单独记录在 [`visual_grasp_commands.md`](visual_grasp_commands.md)，不要把本文的仿真或遥操作命令与视觉抓取入口同时启动在同一个 `rebotarm` 命名空间。

## 0. 所有终端的环境准备

每个新终端先执行：

```bash
cd /home/huangbin/robotarm_ros2
source tools/source_local_environment.bash
unset ROS_DOMAIN_ID
```

单套 ROS 2 系统不需要设置 `ROS_DOMAIN_ID`。如果多套系统并行运行，才为每套系统显式设置不同域，并保证同一套系统的所有终端一致。

检查当前串口：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
fuser -v /dev/ttyACM0 /dev/ttyACM1 2>&1 || true
```

串口只能由一个真实控制器占用。启动真机前还应检查 MotorBridge：

```bash
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
```

真机默认失能。读取反馈不需要 Enable；只有真实运动前才需要现场确认、显式 Enable，并持续观察急停和支撑条件。

## 1. 真机底层与状态反馈

### 1.1 只启动硬件控制器

适合底层通信、反馈和服务排查；不启动 MoveIt、网页或键盘：

```bash
ros2 launch rebotarm_bringup hardware_controller.launch.py \
  channel:=/dev/ttyACM0
```

串口号变化时改为实际设备，例如 `/dev/ttyACM1`。启动后检查：

```bash
ros2 topic echo /rebotarm/arm_status --once
ros2 topic echo /rebotarm/gripper/state --once
ros2 topic hz /rebotarm/joint_states
```

期望启动状态是：

```text
enabled: false
state_machine: IDLE
per_joint_status_code: [0, 0, 0, 0, 0, 0]
error_codes: []
```

### 1.2 基础真机 Bringup

增加机器人 TF、夹爪可视化状态桥和可选基础 RViz，但不启动 MoveIt：

```bash
ros2 launch rebotarm_bringup bringup.launch.py \
  channel:=/dev/ttyACM0 \
  use_rviz:=true
```

它适合确认真实关节反馈与 RViz 模型是否一致，不提供 MoveIt 的 Plan/Execute。

## 2. MoveIt 末端拖动与轨迹规划

### 2.1 真机 MoveIt

唯一正式的真机 MoveIt 用户入口：

```bash
ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM0 \
  use_rviz:=true
```

流程：

```text
启动并确认 CONNECTED_DISABLED
→ RViz MotionPlanning 中 Plan
→ 现场安全确认
→ 显式 Enable
→ Execute 小幅已验证轨迹
→ Stop/必要时 Safe Home
→ 确认静止后 Disable
→ Ctrl+C
```

Plan 成功不等于 Execute 成功。真机执行前必须确认唯一的：

```bash
ros2 action info /rebotarm/follow_joint_trajectory
```

不要同时启动 `rebotarm_app.launch.py`、`bringup.launch.py` 或其他会占用同一串口的真机入口。

### 2.2 纯仿真 RViz 末端拖动

使用仿真轨迹控制器，不连接真机：

```bash
ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

该入口在 RViz 中提供 MotionPlanning 的 Plan/Execute，Execute 只发送到仿真控制器。

## 3. Dashboard、网页遥操作和示教

### 3.1 完整 Dashboard 工作台

这是网页、MoveIt、状态 RViz 和示教录制/回放的正式组合入口，不包含键盘节点：

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  channel:=/dev/ttyACM0 \
  web_execute_enabled:=true
```

打开：

```text
http://127.0.0.1:8088/
```

默认：

```text
use_hardware=true
execution_mode=execute
web_execute_enabled=true
```

网页功能包括：

```text
关节 Preview（不动真机）
关节 Execute（真实运动）
Stop / Safe Home / Enable / Disable
夹爪控制
Teach Trajectory 录制、检查、Dry-run 和 Replay
MoveIt 起点对齐与碰撞预检
```

示教正式流程统一使用网页的 `Teach Trajectory` 卡片：

```text
Start Teach → 手动拖动 → Stop Teach
→ Check Trajectory → 质量分析/预处理/碰撞预检/Dry-run
→ 明确授权后 Replay
```

### 3.2 仅键盘遥操作

真机键盘遥操作：

```bash
ros2 launch rebotarm_bringup teleop_keyboard.launch.py \
  use_hardware:=true \
  channel:=/dev/ttyACM0 \
  use_local_rviz:=true
```

无硬件 RViz 姿态测试：

```bash
ros2 launch rebotarm_bringup teleop_keyboard.launch.py \
  use_hardware:=false \
  use_local_rviz:=true
```

键盘仅适合小步调姿和链路检查，不替代 Dashboard 示教录制。真机使用时仍需显式 Enable。

### 3.3 键盘 + Dashboard + 示教组合

不需要 MoveIt 的遥操作/示教组合：

```bash
ros2 launch rebotarm_bringup teleop_system.launch.py \
  use_hardware:=true \
  channel:=/dev/ttyACM0 \
  panel:=true \
  web_execute_enabled:=true
```

需要示教轨迹检查、MoveIt 起点对齐或碰撞预检时，应使用上一节的 `rebotarm_app.launch.py`，因为它包含完整 MoveIt 工作台。

## 4. 标定工作流

标定页面集成在 Dashboard 中。启动真机工作台时显式增加：

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  channel:=/dev/ttyACM0 \
  calibration:=true
```

打开：

```text
http://127.0.0.1:8088/calibration
```

如果 Dashboard 已经运行且没有内置启动标定节点，可在另一个已加载环境的终端只启动标定采集节点：

```bash
ros2 run rebotarm_calibration rebotarm_handeye_capture
```

不要同时启动两份标定节点写同一个会话目录。标定节点本身不自动使能、不自动移动机械臂，也不自动部署计算结果。完整操作步骤见 [`calibration_web_usage.md`](calibration_web_usage.md)。

## 5. 独立相机与 YOLO 检查

只启动 Gemini 2、CameraInfo、YOLO 和检测话题，不启动 GraspNet、MoveIt 或机械臂控制器：

```bash
ros2 launch rebotarm_vision vision_ubuntu.launch.py
```

检查：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /grasp/detections --once
```

`vision.launch.py` 是通用内部组合，日常 Ubuntu 原生环境优先使用已经固定相机、手眼和模型默认值的 `vision_ubuntu.launch.py`。

## 6. MuJoCo 仿真

仿真不会打开真实串口，不会使能真实电机。通常只运行一个仿真入口。

### 4.1 MuJoCo + MoveIt + RViz

适合测试物理仿真、规划和仿真轨迹执行：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py
```

不打开 MuJoCo/RViz 图形界面的服务器模式：

```bash
ros2 launch rebotarm_simulation mujoco_headless.launch.py
```

只使用仿真 RViz 查看组合：

```bash
ros2 launch rebotarm_simulation mujoco_rviz_viewer.launch.py
```

运行解释器需要 MuJoCo 依赖；环境脚本会设置 `REBOTARM_MUJOCO_PYTHON`。需要显式覆盖时：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py \
  python_executable:=/home/huangbin/robotarm_ros2/third_party/rebotarm_mujoco_venv/bin/python
```

### 4.2 MuJoCo 物理节点底层入口

`mujoco_sim.launch.py` 是仿真后端组合的底层入口，适合开发和排查，不是日常首选：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py \
  enable_virtual_camera:=false
```

只有在明确需要虚拟 RGB-D 相机时才打开：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py \
  enable_virtual_camera:=true
```

它不会自动组合 MoveIt 或真实视觉抓取链；需要规划时使用 `mujoco_moveit_sim.launch.py`。

## 7. 不建议直接启动的内部组合

以下文件是被上层入口包含的共享实现，不是普通用户的首选命令：

```text
interactive_system.launch.py
rebotarm_moveit_config/demo.launch.py
rebotarm_vision/vision.launch.py
```

它们负责后端选择、MoveIt、RViz 和状态源组合。为了避免重复控制器、重复状态发布器或重复串口占用，请使用本手册中的用户入口。

## 8. 统一停机顺序

### 真机已 Enable 或可能正在运动

```text
停止当前轨迹
→ 受控回到已确认安全位（需要时调用 /rebotarm/safe_home）
→ 确认机械臂静止并有支撑
→ 调用 /rebotarm/disable
→ 确认 enabled=false
→ 启动终端 Ctrl+C
```

### 真机保持失能、仅做反馈或 plan-only

确认：

```bash
ros2 topic echo /rebotarm/arm_status --once
```

确认 `enabled: false` 后可直接在启动终端按 `Ctrl+C`。

`Ctrl+C` 不是 Safe Home，也不是切断 24 V 电源。健康但回位失败时保持 enabled hold，等待人工处置；通信丢失、电机故障或明确急停遵循控制器保护策略。
