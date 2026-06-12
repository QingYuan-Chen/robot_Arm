# 语音控制阶段 4 工程说明

本文档记录 `rebotarm_voice_control` 在阶段 4 的工程边界。当前目标是把 LLM/Realtime 输出的结构化工具调用安全地接到 ROS2 仿真执行链，仍然禁止大模型直接控制关节、电机、电流、CAN 或底层轨迹。

## 当前链路

```text
文本/Realtime function call
  -> ToolCallParser
  -> SafetyGuard
  -> ExecutionModeRouter
  -> /rebotarm/sim/... RouteResult
  -> SimExecutor
  -> RecordedSimExecutor 或 MoveIt2SimExecutor
  -> Ros2ActionTransport
```

## 执行模式

- `dry_run`：只做解析、安全检查和路由，不派发 ROS2 action/service。
- `sim`：路由到 `/rebotarm/sim/...` 命名空间，默认由 `RecordedSimExecutor` 记录，不真实派发 MoveIt2。
- `real`：launch 文件已预留，但 `safety_limits.yaml` 默认 `allow_real_ros_calls: false`，不会直接允许真机调用。

## 配置文件

- `config/llm_config.yaml`：默认大模型 provider 配置，当前默认 `doubao`，API key 环境变量为 `ARK_API_KEY`。
- `config/sim_config.yaml`：仿真执行后端配置，当前默认 `backend: recorded`。
- `config/safety_limits.yaml`：真实调用总开关、工作空间、named pose 白名单等安全限制。

## 主要入口

```bash
ros2 run rebotarm_voice_control rebotarm_llm_tool "向上移动 5 厘米" --provider mock --mode sim
ros2 run rebotarm_voice_control rebotarm_realtime_gateway --event-jsonl events.jsonl --mode sim
ros2 run rebotarm_voice_control rebotarm_sim_executor route.json
ros2 launch rebotarm_voice_control voice_sim.launch.py
ros2 launch rebotarm_voice_control voice_realtime.launch.py event_jsonl:=events.jsonl
ros2 launch rebotarm_voice_control voice_real.launch.py
```

`voice_real.launch.py` 只设置 `execution_mode=real`，不会自动打开 `allow_real_ros_calls`。

## 仿真后端

### recorded

默认后端。它只接受 `/rebotarm/sim/...` 路由，返回：

```json
{
  "accepted": true,
  "dispatched": false,
  "backend": "recorded_sim"
}
```

这表示命令通过了仿真执行层的边界检查，但还没有派发到 MoveIt2。

### moveit2

`MoveIt2SimExecutor` 已预留，并通过 `Ros2ActionTransport` 发送 ROS2 action goal。当前 CLI 在没有 ROS2 node/transport 的情况下会安全拒绝。后续需要在 ROS2 node 内创建 transport，再接入真实 action server。

## 已绑定的 action goal 构建

- `/rebotarm/sim/move_to_pose` -> `rebotarm_msgs/action/MoveToPose`
- `/rebotarm/sim/pick_object` -> `rebotarm_msgs/action/ExecuteGrasp`
- `/rebotarm/sim/place_object` -> `rebotarm_msgs/action/ExecuteGrasp`

`/rebotarm/sim/move_relative` 仍保留为语音层安全路由目标，后续需要决定是转换为末端位姿偏移，还是转换为规划轨迹后再派发。

## 真机前置条件

真机执行前至少需要满足：

- `dry_run` 和 `sim` 链路测试全部通过。
- 明确 safe home、named pose 和 workspace 限制。
- `allow_real_ros_calls` 由人工修改为 `true`。
- 急停、stop service/action、碰撞检查和速度限制完成验证。
- 日志中能追踪原始文本、tool call、intent、安全检查结果、route 和执行结果。
