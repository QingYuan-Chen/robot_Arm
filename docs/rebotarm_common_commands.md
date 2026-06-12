# reBotArm 遥操作使用文档

这份文档只写当前真实可用的遥操作流程，目标是直接复制粘贴使用。  
日常启动不需要选择 `mode`，网页是主入口。

## 1. 启动遥操作系统

终端 A，保持运行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  use_hardware:=true \
  web_execute_enabled:=true
```

这条命令就是默认完整工作台，会同时启动：

```text
reBotArmController 真机控制
MoveIt move_group
RViz
网页状态/遥操作面板
键盘遥操作
示教录制后台节点
```

日常只使用这一条启动命令。打开网站后，网页遥操作、键盘遥操作、示教录制、轨迹检查、MoveIt 起点对齐、碰撞预检查、回放、RViz 显示都应该可用。

默认 `channel:=auto`，会优先选择 `/dev/ttyACM0`，然后尝试 `/dev/ttyACM1`。如果自动选择不对，再手动加：

```bash
ros2 launch rebotarm_bringup rebotarm_app.launch.py channel:=/dev/ttyACM1
```

重要：示教轨迹检查、MoveIt 起点对齐、碰撞预检查、真实 replay 必须用上面的完整 `rebotarm_app.launch.py` 启动。不要只启动单独的状态面板或 `teleop_system.launch.py` 来判断示教回放是否可用，否则网页里会出现：

```text
MoveIt: unavailable
Collision: unknown
```

这通常不是轨迹文件坏了，而是 `move_group` / MoveIt 服务没有启动完整。

打开网页：

```text
http://127.0.0.1:8088/
```

启动后应同时具备：

```text
controller 真机控制
MoveIt 起点对齐和碰撞检查
RViz 三维显示
网页实时状态面板
网页关节预览/执行
网页夹爪控制
网页 Safe Home / Enable / Disable
网页示教轨迹 dry-run / replay
键盘关节小步遥操作
```

## 2. 网页遥操作

网页里可以直接做：

```text
Preview：只动网页模型，不动真机
Execute：发送关节轨迹到真机
Stop：停止网页 Execute 轨迹
Set Gripper：控制夹爪
Safe Home：回到安全位
Enable：使能机械臂
Disable：失能机械臂
Stop Replay：停止示教回放轨迹
```

推荐顺序：

```text
1. 先拖动 Preview 滑条，确认网页模型姿态正确
2. 设置较长 Duration，避免速度过快
3. 点击 Execute
4. 如果运动不对，点击 Stop
```

注意：

```text
Preview 不动真机
Execute 才会动真机
Stop Replay 主要用于示教回放
Stop 主要用于网页 Execute
```

## 3. 键盘遥操作

启动遥操作系统后，键盘遥操作节点也会启动。  
键盘遥操作用于关节小步调姿，不作为示教数据来源。

当前键盘遥操作是：

```text
每次按键对单个关节增加或减少一个小步长
默认小步长约 0.02 rad
长按相当于连续多次按键
```

用途：

```text
小范围调姿
测试 controller 链路
测试 RViz / 网页实时状态
```

## 4. 示教轨迹检查和回放

网页里选择轨迹文件，例如：

```text
large_replay_check.jsonl
```

如果网页显示：

```text
MoveIt: unavailable
Collision: unknown
```

先检查是不是没有启动完整 app。请在另一个终端执行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service list | grep -E "plan_kinematic_path|check_state_validity"
```

预期至少能看到 MoveIt 规划和状态有效性检查相关服务。如果没有输出，停止当前 launch，重新用完整 app 启动：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  web_execute_enabled:=true
```

等终端出现类似：

```text
You can start planning now!
```

再刷新网页重新选择轨迹文件。

先点击：

```text
Check Trajectory
```

确认结果不是：

```text
blocked
red
collision
unavailable
```

再点击：

```text
Replay
```

回放过程中如果不对劲，点击：

```text
Stop Replay
```

当前最近使用过的轨迹文件：

```text
teleop_records/large_replay_check.jsonl
```

## 5. 重力补偿手拖示教录制

现在录制已经集成到网页 `Teach Trajectory` 卡片里。录制文件名可以自己填，但只填文件名，不填路径；后端会自动保存到固定目录：

```text
teleop_records/<你填写的文件名>.jsonl
```

例如网页里填：

```text
my_teach_01
```

实际保存为：

```text
teleop_records/my_teach_01.jsonl
```

网页录制流程：

```text
1. 启动完整 app
2. 打开网页 http://127.0.0.1:8088/
3. 展开 Teach Trajectory 卡片
4. 在 File Name 填入文件名，例如 my_teach_01
5. 点击 Start Teach，系统会进入重力补偿并开始录制
6. 人手拖动机械臂完成示教
7. 点击 Stop Teach 结束录制
8. 点击 Refresh Files 或在文件下拉框选择刚录制的 jsonl
9. 点击 Check Trajectory
10. 检查通过后点击 Replay
```

如果需要不用网页、单独用终端录制，也可以保留下面的命令方式。

录制新轨迹：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup teach_record.launch.py \
  record_path:=teleop_records/new_teach.jsonl \
  auto_start_gravity_comp:=true
```

操作流程：

```text
1. 等待机械臂进入 GRAVITY_COMP
2. 人手拖动机械臂完成示教
3. 按 q 结束录制
4. 回到网页选择 new_teach.jsonl
5. 先 Check Trajectory
6. 再 Replay
```

如果录制后文件大小是 0，说明没有真正写入样本，通常是：

```text
没有进入 GRAVITY_COMP
没有收到 /rebotarm/joint_states
录制时间太短
```

## 6. RViz 显示

启动遥操作系统后，RViz 应该显示：

```text
机械臂本体
新夹爪 base
left_finger
right_finger
```

检查显示层关节：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic echo --once /rebotarm/visual_joint_states
```

预期包含：

```text
joint1
joint2
joint3
joint4
joint5
joint6
left_finger_joint
right_finger_joint
```

## 7. 状态检查

检查 action 和真机状态：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 action list | grep follow_joint_trajectory
ros2 action list | grep gripper
ros2 topic echo --once /rebotarm/arm_status
```

预期能看到：

```text
/rebotarm/follow_joint_trajectory
/rebotarm/gripper/command
enabled: true
```

查看回放状态：

```bash
ros2 topic echo --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /rebotarm/teleop/replay_status
```

## 8. 退出和安全

正常退出：

```text
在启动遥操作系统的终端按 Ctrl+C
```

controller shutdown 会尝试：

```text
safe_home -> disable
```

重要：

```text
Ctrl+C 不是硬件急停。
如果机械臂已经碰撞、夹住东西、姿态危险，优先使用硬件急停/断电/disable。
```

## 9. 重新编译

修改代码后重新编译：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select \
  rebotarm_bringup \
  rebotarm_moveit_config \
  rebotarm_interactive_control \
  rebotarmcontroller

source install/setup.bash
```

## 10. 测试

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 -m pytest tests/test_rebotarm_app_launch.py tests/test_teleop_teach_core.py tests/test_visual_grasp_wiring.py -q
```

预期：

```text
passed
```

## 11. MoveIt + Ruckig teach replay

Current teach replay route:

```text
raw JSONL
-> prepared trajectory
-> filter / resample / jerk-aware retime
-> MoveIt start alignment
-> MoveIt collision precheck
-> /rebotarm/follow_joint_trajectory
```

Important:

```text
1. Real replay executes prepared retimed points. It no longer sends raw teach points directly.
2. MoveIt Ruckig is wired into the MoveIt planning pipeline for MoveIt-planned start alignment paths.
3. Prepared replay currently reports the real time-parameterization method. If Python waypoint Ruckig is not implemented/available, it reports current_jerk_retime instead of pretending to be Ruckig.
4. In the web Check result, read:
   - Time Parameterization: the prepared replay trajectory method
   - MoveIt Ruckig: whether the MoveIt start-alignment path is handled by the MoveIt pipeline
5. Real replay now has a runtime monitor. During replay it compares live `/rebotarm/joint_states` with the active trajectory and requests `trajectory_stop` if tracking error or live velocity stays over the safety limit.
```

Daily full app startup:

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup rebotarm_app.launch.py \
  use_hardware:=true \
  channel:=/dev/ttyACM0 \
  web_execute_enabled:=true
```

Web workflow:

```text
1. Open http://127.0.0.1:8088/
2. Teach Trajectory -> Record a new file or choose an existing file.
3. Check -> Check Trajectory.
4. Confirm Playback Quality is not red, Collision is not collision, and MoveIt is not unavailable.
5. Replay -> Replay Prepared.
6. If motion is wrong, press Stop Replay first.
```

Runtime replay monitor defaults:

```text
replay_monitor_enabled: true
max_tracking_error_rad: 0.25
max_live_velocity_rad_s: 3.0
replay_monitor_start_grace_sec: 1.0
replay_monitor_violation_grace_sec: 0.30
```

If replay is stopped by the monitor, the replay status will show:

```text
state: safety_stop
runtime replay monitor stopped trajectory
reason: tracking_error or live_velocity
```

MoveIt + Ruckig no-hardware smoke test:

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

timeout 25s ros2 launch rebotarm_moveit_config demo.launch.py use_rviz:=false
```

Expected log:

```text
Loaded adapter 'default_planning_response_adapters/AddRuckigTrajectorySmoothing'
You can start planning now!
```
