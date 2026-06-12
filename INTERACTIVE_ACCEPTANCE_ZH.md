# reBotArm 交互系统验收清单

这份文档用于收口当前 `RViz + MoveIt 预览 + Interactive Marker` 软件链路，目标是在不上实机的前提下，把交互、预览、门控和规划显示全部验证清楚。

## 1. 当前验收目标

本轮重点确认以下 4 件事：

- RViz 能正常弹出
- 末端交互 marker 能看到并拖动
- 发布 `pose_target` 后 `/rebotarm/interactive_control/preview` 会更新
- 不可达目标会返回失败状态，而不是假成功

## 2. 启动前清理

先关闭旧进程，避免重复节点干扰：

```bash
pkill -f rviz2
pkill -f move_group
pkill -f MarkerServerNode
pkill -f PreviewNode
pkill -f ExecutionNode
pkill -f robot_state_publisher
pkill -f joint_state_publisher
pkill -f static_transform_publisher
```

然后重新加载环境：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 3. 标准启动命令

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=false \
  use_local_rviz:=true
```

启动成功后，优先观察 3 个现象：

- RViz 窗口弹出
- 机械臂模型显示正常
- 能看到末端交互 marker

## 4. 节点与话题检查

新开终端执行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 node list | grep -E "rviz2|marker_server|preview_node|execution_node"
ros2 topic list | grep interactive_control
```

至少应看到这些节点：

- `/rviz2`
- `/marker_server`
- `/preview_node`
- `/execution_node`

至少应看到这些话题：

- `/rebotarm/interactive_control/ee_target/update`
- `/rebotarm/interactive_control/pose_target`
- `/rebotarm/interactive_control/preview`
- `/rebotarm/interactive_control/status`

## 5. 验证 marker 输入链路

在终端中监听状态：

```bash
ros2 topic echo /rebotarm/interactive_control/status
```

然后回到 RViz 拖动末端 marker。

正常现象：

- 拖动过程中会反复出现 `moveit preview solving`
- 停在可达姿态后，可能出现 `preview ready`
- 停在不可达姿态后，可能出现 `moveit preview failed: error_code=...`

这说明链路 `RViz marker -> pose_target -> PreviewNode` 正在工作。

## 6. 验证可达目标

发送一个大概率可达的目标：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 0.25, y: 0.00, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

然后检查：

```bash
ros2 topic echo /rebotarm/interactive_control/preview --once
ros2 topic echo /rebotarm/interactive_control/status --once
```

通过标准：

- `preview` 中 `reachable: true`
- `message` 类似 `moveit ik preview ready`
- `status` 没有不可达报错

## 7. 验证不可达目标

发送一个明显超范围目标：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 1.20, y: 0.00, z: 1.20}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

然后检查：

```bash
ros2 topic echo /rebotarm/interactive_control/preview --once
ros2 topic echo /rebotarm/interactive_control/status --once
```

通过标准：

- `preview` 中 `reachable: false`
- `message` 明确说明不可达或 IK 失败
- 不能仍然显示成功预览

## 8. 验证执行门控

先发送一个可达目标，再调用执行：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 0.25, y: 0.00, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

ros2 service call /rebotarm/interactive_control/execute_preview std_srvs/srv/Trigger "{}"
```

当前软件模式下，预期结果为：

- `success=True`
- `message='execution accepted'`

再发送不可达目标后执行：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 1.20, y: 0.00, z: 1.20}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

ros2 service call /rebotarm/interactive_control/execute_preview std_srvs/srv/Trigger "{}"
```

预期结果为：

- `success=False`
- `message='target pose unreachable'`

这一步用于确认 `ExecutionNode` 不会把不可达目标误判为可执行。

## 9. 在 RViz 中看规划轨迹

如果当前目标是“看机械臂如何到达目标点”，建议用 MoveIt 的 `MotionPlanning` 面板，而不是直接点执行。

推荐操作：

1. 使用 MoveIt 自己的交互 marker
2. 选择规划组 `arm`
3. 将目标拖到可达位置
4. 点击 `Plan`
5. 在 `Planned Path` 中观察规划结果

说明：

- 当前 `use_hardware:=false`
- 因此本阶段重点是 `Plan`
- `Execute` 失败不代表规划有问题，只代表当前没有真实执行后端

## 10. 验收结论模板

如果本轮全部通过，可以直接记录为：

- RViz 可正常启动并显示机械臂模型
- 末端交互 marker 可见，拖动后能触发预览求解
- 可达目标会更新 `/rebotarm/interactive_control/preview` 并返回成功结果
- 不可达目标会明确返回失败状态，不存在假成功

## 11. 下一阶段建议

本轮软件验收通过后，再进入下一阶段：

1. 保持 `Plan` 可视化稳定
2. 启用 `use_hardware:=true`
3. 检查 `/rebotarm/follow_joint_trajectory` 是否真的有 action server
4. 仅做小幅度、低风险的实机验证

在进入实机前，建议补一份单独的“真机前检查清单”。
