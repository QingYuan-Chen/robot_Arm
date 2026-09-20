# reBotArm 功能开启指令

> 当前所有非视觉抓取入口的统一速查表见 [`rebotarm_launch_commands.md`](rebotarm_launch_commands.md)。本文保留 MoveIt 与网页工作台的详细说明。

当前视觉部署只支持 Ubuntu 原生 ROS 链；不要使用历史 Windows/HTTP/MJPEG 命令。

这个文档只放仍然保留的独立功能启动指令和测试顺序。

> P0-P6 已按用户确认的工程范围关闭，验收依据及未实测边界见 `Agent/PROJECT_STATUS.md`。阶段关闭不是新的运动授权；真机启动和执行前仍需确认本次授权、串口唯一归属及现场安全。连接保持失能，运动必须显式 enable。

## RViz MoveIt 末端拖动

旧的 `interactive_basic.launch.py` 和重复的 `rviz_ee_drag_real.launch.py` 已移除。
真机 RViz 末端拖动统一使用 `moveit_hardware.launch.py use_rviz:=true`，仿真使用
`rviz_ee_drag_sim.launch.py`。

执行前确认已安装 `ros-jazzy-moveit-simple-controller-manager`。
若 Plan 成功而 Execute 报 `controller_manager_ does not exist` 或插件类不存在，
参见[执行插件检查与仿真重启步骤](local_setup_zh.md#moveit-执行插件能-plan-但不能-execute)。
KDL 根惯量警告与这类执行插件缺失不是同一个问题。

两个后端都使用 MoveIt 原生 MotionPlanning：

```text
RViz MotionPlanning
-> MoveIt Plan / Execute
-> FollowJointTrajectory controller
```

不再启动自定义 `ee_target` marker。

真机：

> 仅在本次真机操作已获授权后使用，不得与其他占用同一串口的控制器同时启动。

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM0 use_rviz:=true
```

如果自动串口不对，可以手动指定：

```bash
ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM1 use_rviz:=true
```

仿真 / 不连接真机：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup rviz_ee_drag_sim.launch.py
```

预期现象：

```text
RViz 自动打开 MotionPlanning 面板。
使用 MotionPlanning 的目标姿态 marker 做末端拖动。
点击 Plan 只规划。
真机模式下点击 Execute 才会下发到控制器。
仿真入口同时启动 `rebotarm_sim_trajectory_controller`，提供 `/rebotarm/follow_joint_trajectory`，因此仿真可执行 Plan & Execute。
真机模式启动后保持失能；先 Plan，再人工确认现场安全并显式 Enable，最后 Execute。
真机入口不会启动仿真 fake controller；仿真入口不会连接真机。
```

真机 RViz 推荐顺序：

```text
启动 -> 确认 CONNECTED_DISABLED 和 joint_states -> RViz Plan-only
-> 显式调用 /rebotarm/enable -> 当前位置 hold 检查 -> Execute
-> trajectory_stop -> 必要时显式 safe_home -> 确认静止 -> disable -> Ctrl+C
```

真机入口默认 `shutdown_safe_home:=false`，直接按 `Ctrl+C` 不会自动回 safe_home；需要回位时必须人工确认后显式调用 `safe_home`，再 `disable`。

## 网页遥操作

功能链路：

```text
Web Dashboard
-> rebotarm_dashboard
-> rebotarm_teleop command adapter
-> rebotarmcontroller
-> real arm
```

启动完整网页遥操作：

> 仅在本次真机操作已获授权后使用；此入口包含控制器，不与上面的真机入口并行启动。

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup rebotarm_app.launch.py
```

默认串口：

```text
channel:=auto
1. 优先 /dev/ttyACM0
2. 然后 /dev/ttyACM1
3. 都没找到时回退 /dev/ttyACM0
```

如果自动选择不对，可以手动指定：

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py channel:=/dev/ttyACM1
```

打开网页：

```text
http://127.0.0.1:8088/
```

启动后应同时具备：

```text
网页关节 Preview / Execute / Stop
网页 Safe Home / Enable / Disable
网页夹爪控制和 joint7 状态显示
示教录制、轨迹检查、优化回放
MoveIt 起点对齐和碰撞预检查
RViz 轻量机械臂实时状态显示
```

网页遥操作测试顺序：

```text
1. 点击 Enable，确认网页和 /rebotarm/arm_status 变为 enabled
2. 点击 Safe Home，确认机械臂回安全位
3. 小幅拖动 Preview 滑条，确认只动网页模型
4. 点击 Execute，确认真机小幅运动
5. Execute 过程中点击 Stop，确认轨迹停止且按钮恢复
6. 拉动 gripper，确认网页 joint7 和真机夹爪都有变化
7. 受控回到已确认的安全停放位，核验到位并静止后点击 Disable；健康但回位失败时保持使能，等待人工处置
```

`rebotarm_app.launch.py` 默认不在 `Ctrl+C` 时自动运动到 Safe Home。
正常 `Ctrl+C` 会请求 controller 停止控制循环、失能电机并断开串口；
失能只是撤去电机保持力，不是切断 24 V 整机电源。因此不要用
`Ctrl+C` 代替安全停机操作：先显式 Safe Home，确认静止，再 Disable，
最后按 `Ctrl+C`。失能前应托住可能因重力下坠的机械臂。

辅助检查：

```bash
ros2 topic echo --once /rebotarm/arm_status
ros2 action list | grep follow_joint_trajectory
ros2 service list | grep -E "plan_kinematic_path|check_state_validity"
```

## MoveIt 实机操作与停机

`bringup.launch.py` 仍保留为基础控制器/状态组合入口，不提供 MoveIt 末端拖动。
旧的 `rviz.launch.py` 和 `rviz_ee_drag_real.launch.py` 已移除；需要真机 MoveIt Plan/Execute 时使用
`moveit_hardware.launch.py`，仿真使用 `rviz_ee_drag_sim.launch.py`；需要网页、示教和完整工作台时使用
`rebotarm_app.launch.py`。不要同时启动这些入口。

以下内容接替已清理的根目录 MoveIt 使用说明。使用原生 MotionPlanning，
不依赖旧自定义预览和执行节点。只需要 MoveIt 实机链时，可以选择独立入口：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
fuser -v /dev/ttyACM0
```

版本检查必须成功，串口必须无人占用；若已有控制器，应使用其现有入口或由操作者
正常结束，不能抢占端口。确认本次操作授权后，在同一环境启动：

```bash
ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM0 use_rviz:=true
```

1. 检查 `/rebotarm/arm_status`、新鲜 joint states 及唯一的
   `/rebotarm/follow_joint_trajectory` action server，确认控制器无错误。
2. 在 MotionPlanning 选择 `Planning Group = arm`，设置目标并点击 `Plan`。
3. 核对起始姿态、轨迹和现场净空；确认授权并显式 enable 后，点击 `Execute`。
4. 同时观察实机、RViz 当前姿态与 action 结果；发出目标不等于已经到位。

结束时先停止活动轨迹，确认控制器健康并按本次已验证的基线受控回位；若采用
已确认的网页停放位，可调用 `/rebotarm/safe_home` 并核验成功、实际到位及静止。
随后调用 `/rebotarm/disable`，确认失能后在启动终端正常退出。
回位失败但保持能力正常时继续 enabled hold，等待人工处置；不要用批量
`pkill` 或直接失能代替受控停机。电机故障、通信丢失等严重故障和明确急停
遵循控制器保护策略。
