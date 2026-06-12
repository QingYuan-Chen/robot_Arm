# reBotArm 方案1正式使用说明

这份文档对应当前推荐的正式工作流：

- 使用 MoveIt `MotionPlanning` 面板作为主交互入口
- 在 RViz 中查看规划轨迹和当前机器人姿态
- 使用 MoveIt 的 `Plan` / `Execute` 完成实机轨迹执行
- 使用底层 `safe_home` / `disable` 完成回位和停机

这份文档对应的正式启动入口是：

```bash
ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM0 \
  use_rviz:=true
```

## 1. 适用范围

本说明适用于以下场景：

- 正式演示
- 正式验收
- 实机规划与执行测试

本说明不以自定义 `ee_target` 交互链为主线。

## 2. 正式启动

在 Ubuntu 终端执行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup moveit_hardware.launch.py \
  channel:=/dev/ttyACM0 \
  use_rviz:=true
```

如果实际串口不是 `/dev/ttyACM0`，替换成正确设备号。

## 3. 启动后检查

新开终端执行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 node list | grep -E "reBotArmController|move_group|robot_state_publisher|rviz2"
ros2 action info /rebotarm/follow_joint_trajectory
```

正常应看到：

- `/reBotArmController`
- `/move_group`
- `/robot_state_publisher`
- `/rviz2`

并且：

- `/rebotarm/follow_joint_trajectory` 的 `Action servers` 大于 `0`

## 4. RViz 正式操作流程

在 RViz 中使用 `MotionPlanning` 面板：

1. 选择 `Planning Group = arm`
2. 使用 MoveIt 自带交互 marker 设定目标位姿
3. 点击 `Plan`
4. 观察 `Planned Path`
5. 确认轨迹合理后点击 `Execute`

正式工作流中：

- 轨迹显示由 MoveIt 提供
- 当前姿态模型由真实 `/rebotarm/joint_states` 驱动
- 实机执行通过 `/rebotarm/follow_joint_trajectory` 完成

## 5. 执行时应观察什么

执行前重点观察：

- 规划路径是否合理
- 末端目标是否处于可接受范围
- 机械臂周围是否安全

执行时重点观察：

- 真实机械臂是否按规划轨迹运动
- RViz 中灰色当前姿态模型是否同步更新
- 执行完成后灰色模型是否接近橙色目标姿态

## 6. 安全回位

如果需要让机械臂安全回位，优先使用：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger "{}"
```

预期结果：

- `success=True`
- `message='safe_home complete'`

## 7. 失能停机

如果需要底层失能停机，使用：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

适用场景：

- 结束测试
- 需要停止驱动使能
- 当前不准备继续动作执行

## 8. 结束测试

测试结束后可清理进程：

```bash
pkill -f moveit_hardware.launch.py
pkill -f reBotArmController
pkill -f move_group
pkill -f rviz2
pkill -f robot_state_publisher
pkill -f static_transform_publisher
```

## 9. 与实验链的关系

当前仓库中仍保留以下实验性能力：

- `interactive_system.launch.py`
- `MarkerServerNode`
- `PreviewNode`
- `ExecutionNode`

它们用于：

- 自定义 3D 交互链实验
- 预览门控链验证
- 后续方案 2 扩展

但当前正式主线不再依赖这些节点。

## 10. 当前正式结论

当前方案 1 已完成以下闭环：

- MoveIt 面板设定目标
- MoveIt 规划轨迹显示
- `/rebotarm/follow_joint_trajectory` 实机执行
- `/rebotarm/joint_states` 驱动 RViz 当前姿态更新
- `safe_home` 安全回位

当前推荐对外表述为：

> 系统已实现基于 MoveIt MotionPlanning 面板的机械臂三维规划显示与实机轨迹执行闭环。
