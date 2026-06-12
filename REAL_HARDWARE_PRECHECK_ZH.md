# reBotArm 实机前检查清单

这份文档用于在进入 `use_hardware:=true` 之前，先把真实执行链路、风险点和停机条件确认清楚。目标不是马上让机械臂跑起来，而是先证明“现在已经具备安全进入实机验证的前提”。

## 1. 本阶段目标

本轮只确认以下 4 件事：

- 真实执行链路对应的节点和 action 接口存在
- `ExecutionNode` 的 `real` 分支确实能连到真实后端
- 急停、复位、模式切换接口可用
- 可以在低风险前提下准备第一条小动作验证

## 2. 进入实机前的原则

进入 `use_hardware:=true` 前，先遵守这些原则：

- 首次验证只做小幅度动作
- 首次验证不要带负载
- 机械臂周围不要放障碍物
- 人手不要停留在机械臂工作空间内
- 先验证急停可用，再验证动作执行
- 任何一步不满足预期，都停止继续下探

## 3. 启动前环境准备

先关闭旧进程：

```bash
pkill -f rviz2
pkill -f move_group
pkill -f MarkerServerNode
pkill -f PreviewNode
pkill -f ExecutionNode
pkill -f reBotArmController
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f static_transform_publisher
```

重新加载环境：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 4. 启动真实链路

使用真实硬件模式启动：

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=true \
  use_local_rviz:=true
```

如果你的现场暂时不适合开 RViz，也可以先关掉界面，只保留控制链路：

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=true \
  use_local_rviz:=false
```

## 5. 启动后必须先看什么

启动后，先不要发任何目标，先看以下内容是否存在。

### 5.1 节点检查

```bash
ros2 node list | grep -E "execution_node|preview_node|marker_server|rviz2|move_group|rebotarm"
```

至少应包含：

- `/execution_node`
- `/preview_node`
- `/marker_server`
- `/move_group`

如果真实控制器节点有独立名字，也应能在这里看到。

### 5.2 话题检查

```bash
ros2 topic list | grep interactive_control
```

至少应包含：

- `/rebotarm/interactive_control/pose_target`
- `/rebotarm/interactive_control/preview`
- `/rebotarm/interactive_control/status`

### 5.3 action 检查

```bash
ros2 action list | grep follow_joint_trajectory
ros2 action info /rebotarm/follow_joint_trajectory
```

通过标准：

- `/rebotarm/follow_joint_trajectory` 存在
- `Action clients` 中能看到 `/execution_node`
- `Action servers` 不再是 `0`

如果这里仍然是 `Action servers: 0`，就说明还不能进真实执行。

## 6. 先验证 real 分支是否真正接通

先监听状态：

```bash
ros2 topic echo /rebotarm/interactive_control/status
```

再切换模式：

```bash
ros2 service call /rebotarm/interactive_control/set_mode rebotarm_msgs/srv/SetMode "{mode: 'real'}"
```

通过标准：

- 返回 `success=True`
- 状态里不再出现 `follow_joint_trajectory action unavailable`

如果切到 `real` 后仍然报 action 不可用，就停止后续动作测试。

## 7. 先验证停止和失能接口

当前不再推荐使用软件锁存式 `estop/reset_estop` 作为日常调试入口。实机危险情况优先使用实体急停或断电；ROS 侧主要验证下面三个接口。

### 7.1 交互停止

```bash
ros2 service call /rebotarm/interactive_control/stop std_srvs/srv/Trigger "{}"
```

预期：

- `success=True`
- `message` 类似 `interactive stop requested`

语义：停止当前交互执行，并请求底层停止继续追踪轨迹；不是暂停/继续，也不是精确保持在某一个插补点。

### 7.2 底层轨迹停止

```bash
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

预期：

- `success=True`
- `message` 类似 `trajectory stop requested`

语义：只请求底层停止当前轨迹跟踪，不管理视觉/预览状态。

### 7.3 软件失能

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

预期：

- `success=True`

语义：比 stop 更强，可能让电机不再保持力；不要把它当作平滑暂停。

只有这些接口能正常返回，后面的小动作验证才有意义。

## 8. 第一条动作前的推荐检查

在发第一条真实动作前，再确认一次：

- 机械臂当前姿态接近安全初始位
- 周围没有干涉物
- 供电稳定
- 串口、总线或驱动没有异常报错
- 当前目标点离现位不要太远
- 目标姿态不要过分扭转

推荐第一条目标只测一个近距离、低风险点，例如：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 0.25, y: 0.00, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

先看预览：

```bash
ros2 topic echo /rebotarm/interactive_control/preview --once
ros2 topic echo /rebotarm/interactive_control/status --once
```

只有在 `reachable: true` 且状态正常时，才进入真实执行。

## 9. 第一条小动作验证流程

推荐严格按这个顺序：

1. 启动真实链路
2. 确认 action server 已存在
3. 切换到 `real`
4. 验证急停和复位
5. 发布一个近距离可达目标
6. 确认 preview 成功
7. 人员退到安全位置
8. 调用执行服务

执行命令：

```bash
ros2 service call /rebotarm/interactive_control/execute_preview std_srvs/srv/Trigger "{}"
```

通过标准：

- 返回 `success=True`
- 状态从 `idle` 进入 `executing`
- 机械臂出现与目标一致的小幅动作

## 10. 立即停下来的条件

出现以下任一情况，立即停止继续测试：

- `follow_joint_trajectory` server 不存在
- 模式切到 `real` 后仍提示 action unavailable
- 机械臂起始姿态异常
- 目标点虽然可达，但方向明显不合理
- 执行前就出现关节抖动
- 执行中动作方向与预期不一致
- 执行中速度明显过大
- 急停调用后没有表现出门控效果

## 11. 本阶段输出结论模板

如果本轮只做到“实机前检查”，可以记录为：

- `use_hardware:=true` 启动链路已验证
- `/rebotarm/follow_joint_trajectory` action server 已确认存在或仍缺失
- `ExecutionNode` 的 `real` 分支已确认连通或未连通
- 急停、复位、模式切换接口已验证
- 是否具备进入第一条小动作实机验证的条件，结论明确

## 12. 建议的下一步

如果这份清单走通，下一步再进入：

1. 第一条真实小动作验证
2. 连续两到三个近距离点位验证
3. 再考虑更大范围动作
4. 最后才进入“RViz 规划后直接下发到实机”的闭环测试
