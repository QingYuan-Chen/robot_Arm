# reBotArm 功能开启指令

这个文档只放仍然保留的独立功能启动指令和测试顺序。

## MuJoCo 无硬件仿真

完整安装、健康检查、Viewer、ROS 2 接口和排障说明见
[MuJoCo 仿真底座](../src/rebotarm_simulation/README_mujoco.md)。快速入口：

```bash
rebotarm_mujoco_cli --headless --duration 5
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

与 MoveIt 联调时必须保持 `use_hardware:=false`，不得启动实机控制器。

## RViz MoveIt 末端拖动

这两个入口保留，但路线是 MoveIt 原生 MotionPlanning：

```text
RViz MotionPlanning
-> MoveIt Plan / Execute
-> FollowJointTrajectory controller
```

不再启动自定义 `ee_target` marker。

真机：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup rviz_ee_drag_real.launch.py
```

如果自动串口不对，可以手动指定：

```bash
ros2 launch rebotarm_bringup rviz_ee_drag_real.launch.py channel:=/dev/ttyACM1
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
仿真模式不连接真机。
```

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
键盘遥操作
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
7. 点击 Disable，确认真机失能
```

辅助检查：

```bash
ros2 topic echo --once /rebotarm/arm_status
ros2 action list | grep follow_joint_trajectory
ros2 service list | grep -E "plan_kinematic_path|check_state_validity"
```
