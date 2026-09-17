# rebotarm_bringup 启动结构与功能

本文描述 `src/rebotarm_bringup/launch` 的当前结构、包含关系和功能边界。所有真机组合共享
同一个硬件底层片段，禁止在其它 bringup 启动文件中再次直接声明
`reBotArmController`。

`driver_only.launch.py` 已删除：它只是无附加行为地包含
`hardware_controller.launch.py`，保留两个名字反而容易让人误以为有两种硬件启动方式。
只启动底层控制器时直接使用：

```bash
ros2 launch rebotarm_bringup hardware_controller.launch.py
```

## 结构总览

```text
src/rebotarm_bringup/launch/
├── hardware_controller.launch.py
├── bringup.launch.py
├── moveit_hardware.launch.py
├── interactive_system.launch.py
├── rebotarm_app.launch.py
├── teleop_keyboard.launch.py
├── teleop_system.launch.py
├── rviz_ee_drag_real.launch.py
├── rviz_ee_drag_sim.launch.py
├── teach_record.launch.py
├── teach_replay.launch.py
├── visual_ready_hold.launch.py
├── visual_grasp_system.launch.py
├── visual_grasp_perception_preview.launch.py
├── mujoco_offline_perception.launch.py
└── real_perception_sim_execution.launch.py
```

## 分层和包含关系

```text
hardware_controller.launch.py                  唯一真实硬件控制器定义
├── bringup.launch.py                          + 状态发布 + 基础 RViz
├── moveit_hardware.launch.py                  + 示教录制 + MoveIt
│   └── rebotarm_app.launch.py                 + Dashboard + 状态 RViz（不含键盘）
├── interactive_system.launch.py               真机/无硬件状态源选择 + MoveIt/RViz
│   ├── rviz_ee_drag_real.launch.py            真机 MotionPlanning 入口
│   ├── rviz_ee_drag_sim.launch.py             仿真 MotionPlanning 入口
│   └── visual_grasp_system.launch.py          完整视觉抓取组合
├── teleop_keyboard.launch.py                  键盘 + 状态发布 + 基础 RViz（无 MoveIt）
│   └── teleop_system.launch.py                + 示教录制 + Dashboard（仍无 MoveIt）
└── visual_ready_hold.launch.py                + 一次摆位 + 常驻摆位服务

visual_grasp_perception_preview.launch.py      只读感知预览，不含控制器
mujoco_offline_perception.launch.py            虚拟 RGB-D + 离线感知/规划
real_perception_sim_execution.launch.py        真实感知 + MuJoCo 执行
teach_record.launch.py                         只观察并录制已有反馈
teach_replay.launch.py                         使用外部唯一执行后端回放
```

`interactive_system.launch.py` 只有在 `use_hardware=true` 时才包含硬件片段；纯预览和仿真
分支不会打开串口。上层入口必须确保同一命名空间只有一个真实或仿真轨迹执行后端。

## 文件功能

| 文件 | 类型 | 功能和边界 |
|---|---|---|
| `hardware_controller.launch.py` | 底层片段 | 唯一直接启动 `reBotArmController` 的文件；统一串口、反馈频率、夹爪保护、仲裁、命名空间和坐标系参数 |
| `bringup.launch.py` | 基础真机入口 | 硬件、夹爪可视化状态桥、TF 和可选基础 RViz；不启动 MoveIt |
| `moveit_hardware.launch.py` | 真机 MoveIt 入口 | 硬件、示教录制、`move_group` 和可选 MoveIt RViz |
| `interactive_system.launch.py` | 共享组合 | 在真机硬件、无硬件状态源和 MoveIt 预览之间做互斥选择 |
| `rebotarm_app.launch.py` | 完整真机入口 | 包含真机 MoveIt 组合，再增加 Dashboard 和状态 RViz；不启动键盘节点 |
| `teleop_keyboard.launch.py` | 遥操作入口 | 可选硬件、键盘关节点动、状态发布和 RViz；默认不接真机 |
| `teleop_system.launch.py` | 遥操作组合 | 包含键盘入口，再增加示教录制和默认只读 Dashboard；不启动 MoveIt，默认不接真机 |
| `rviz_ee_drag_real.launch.py` | 真机规划入口 | 用 MoveIt MotionPlanning 交互目标进行 Plan/Execute |
| `rviz_ee_drag_sim.launch.py` | 仿真规划入口 | 使用仿真轨迹控制器提供 Plan/Execute，不打开真机 |
| `teach_record.launch.py` | 独立工具 | 只启动录制节点，依赖外部已有反馈，不拥有执行后端 |
| `teach_replay.launch.py` | 独立工具 | 分析、预处理并向外部唯一后端回放轨迹 |
| `visual_ready_hold.launch.py` | 真机专用入口 | 硬件 + 一次视觉观察位摆位 + 常驻摆位服务 |
| `visual_grasp_system.launch.py` | 完整视觉组合 | MoveIt/后端、视觉就绪、相机、YOLO、GraspNet、候选过滤和受控执行 |
| `visual_grasp_perception_preview.launch.py` | 只读预览 | 相机到候选可视化；不启动控制器、不规划、不执行 |
| `mujoco_offline_perception.launch.py` | 离线仿真 | MuJoCo 虚拟相机驱动感知与候选规划，默认 plan-only |
| `real_perception_sim_execution.launch.py` | 混合仿真 | 真实相机/感知连接 MuJoCo 执行，明确禁止真机控制器 |

## 两组容易混淆的入口

| 需求 | 应使用 | 不包含 |
|---|---|---|
| 只需真机 MoveIt 规划/执行和示教录制 | `moveit_hardware.launch.py` | Dashboard、键盘遥操 |
| 需要真机 MoveIt + Dashboard 完整工作台 | `rebotarm_app.launch.py` | 键盘遥操 |
| 只需键盘点动和基础 RViz | `teleop_keyboard.launch.py` | MoveIt、Dashboard、示教录制 |
| 需要键盘 + 示教录制 + Dashboard，且不需要 MoveIt | `teleop_system.launch.py` | MoveIt |

因此它们共享部分界面，但不是重复实现。`moveit_hardware.launch.py` 和
`teleop_keyboard.launch.py` 是可独立使用的基础组合；各自的上层入口只叠加自己的功能。
`rebotarm_app.launch.py` 与 `teleop_system.launch.py` 不应在同一命名空间同时启动。

## 硬件公共参数

`hardware_controller.launch.py` 是以下硬件默认值的权威定义：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `channel` | 空串 | 由配置决定硬件通道；上层可传入探测后的设备路径 |
| `shutdown_safe_home` | `false` | 退出时不自动产生回位运动 |
| `joint_state_rate` | `100.0` | 关节状态发布频率，Hz |
| `hardware_feedback_rate_hz` | `50.0` | 硬件反馈刷新频率，Hz |
| `gripper_position_torque_cap_nm` | `1.0` | 夹爪位置模式力矩上限，N·m |
| `gripper_position_max_speed_rad_s` | `1.5` | 夹爪位置模式速度上限，rad/s |
| `gripper_position_timeout_margin_sec` | `1.5` | 夹爪到位超时余量，s |
| `gripper_feedback_stale_timeout_sec` | `0.15` | 夹爪反馈过期阈值，s |
| `grasp_hold_timeout_sec` | `30.0` | 夹持保持时限，s |
| `cmd_arbitration` | `reject` | 轨迹期间拒绝冲突的透传命令 |
| `arm_namespace` | `rebotarm` | 话题、服务和动作命名空间 |
| `frame_id` / `ee_frame_id` | `base_link` / `end_link` | 基座和末端坐标系 |

上层文件可以声明同名参数作为自己的公开接口并转发覆盖值，但不得复制控制器 `Node`。

### 为什么 `DeclareLaunchArgument` 看起来重复

ROS 2 的每个 launch 文件都有自己的命令行接口。子 launch 声明“我接受哪些参数”；
父 launch 若希望用户仍能在最外层传该参数，需要将它作为自己的公开参数并转发。
因此“同名声明 + 转发”是接口的逐层导出，不等于重复启动节点。

本目录按以下规则维护：

1. 控制器节点只由 `hardware_controller.launch.py` 拥有，硬件默认值以它为权威定义；
2. 上层只重新声明需要从该入口命令行对外暴露或本层节点需要使用的参数；
3. 父层可以选择更保守的场景默认值，但必须在文档中说明；
4. 不为了减少几行声明而隐藏顶层入口的 `--show-args` 接口。

## 安全约束

- 真实硬件启动后保持失能；新鲜反馈、现场检查和显式 `/rebotarm/enable` 缺一不可。
- 一个串口和一个机械臂命名空间只能有一个 `reBotArmController`。
- 真机入口、仿真入口和完整工作台入口不能并行启动在同一命名空间。
- 仿真/预览分支不能包含 `hardware_controller.launch.py` 的有效实例。
- MoveIt Plan 成功不是 Execute 成功；执行链仍需要 `moveit_simple_controller_manager`。
- 健康真机发生可恢复任务失败时保持 enabled hold，或在守护下回到已验证基线后再失能。

## 结构验证

结构测试必须保证：

1. 只有 `hardware_controller.launch.py` 包含 `package="rebotarmcontroller"`；
2. 所有需要真机的组合都包含该文件；
3. 只读感知和纯仿真入口不直接引用真实控制器；
4. `setup.py` 持续安装全部 `*.launch.py`，新增片段在安装空间可被解析。
