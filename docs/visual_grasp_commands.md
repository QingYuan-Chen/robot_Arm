# 视觉抓取启动手册

> 当前主路线为单 Ubuntu：Gemini 2、YOLO、GraspNet 和 ROS 2 均在 Ubuntu 本机运行。
> Windows HTTP/MJPEG/GraspNet 中转工具已从仓库删除，不再提供 Windows 启动命令或验收路径。
> 任何真机执行仍需单独授权和现场安全预检。

普通单套 ROS 2 系统无需设置 `ROS_DOMAIN_ID`：不设置时使用默认域 0。
本页启动命令与检查命令应在加载同一工作区环境的终端执行；如果先前终端
已经设置过 `ROS_DOMAIN_ID`，请在所有相关终端先执行 `unset ROS_DOMAIN_ID`，
并安全退出旧 launch 后重新启动，已运行的 ROS 进程不会随终端环境变化而切换域。
仅当多套 ROS 系统需要并行隔离时才显式指定域号，并确保同一套系统的终端使用同一值。

## 数据链路

```text
Gemini 2 USB -> Ubuntu native camera / YOLO
 -> GraspNet ROS process (.venv-graspnet)
 -> /grasp/graspnet_candidates
 -> candidate IK / workspace / collision gates
 -> MoveIt plan -> MuJoCo or explicitly enabled real controller
```

## Ubuntu 视觉只读验证

```bash
cd /home/huangbin/robotarm_ros2
source tools/source_local_environment.bash
./tools/run_ubuntu_vision.sh yolo_model_path:="$PWD/tools/yolo26s-seg.pt" yolo_device:=0
```

检查：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
ros2 topic echo /grasp/detections --once
```

该步骤不启动控制器，不连接或移动机械臂。

## Ubuntu 原生视觉 plan-only

```bash
source tools/source_local_environment.bash
export RMW_FASTRTPS_USE_SHM=0
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false execution_mode:=plan_only execute_gripper:=false \
  move_to_visual_ready_on_start:=false
```

默认 `move_to_visual_ready_on_start=false` 会直接启动视觉链，并由内置的唯一轻量运动学
状态后端向 MoveIt 提供当前关节状态；不需要额外传 `start_visual_ready:=false`。
`plan_only` 下 `max_plan_age_sec` 默认是 10 s：GraspNet 推理和逐候选 IK/碰撞过滤
可能已耗去 3–5 s，再留时间供人工观察后触发纯规划；切换到
`execution_mode:=execute` 时默认使用 4 s。两种模式都可显式传
`max_plan_age_sec:=<秒数>` 覆盖默认值。
10 s 只是无动作的轨迹预览窗口，不能据此验收眼在手相机的实物对齐或真实抓取安全。
如果服务拒绝请求，响应会区分 `no grasp plan received yet`、
`invalid grasp plan`、`grasp plan expired on arrival` 与 `cached grasp plan expired`。
过期响应中的 `age_sec` 是自采集时间戳算起，而不是自 RViz 显示或消息到达时算起。
调用 `/rebotarm/visual_grasp/execute` 后，执行器依次规划预抓取、抓取和可选的沿接近路径反向撤退；
后一段从上一段规划终点继续。只有全部阶段成功后，运动层才用一条
`/display_planned_path` 消息让 RViz 连续播放完整的紫色机器人幻影，阶段失败时不会显示
不完整序列。该预览不发送控制器 Action，也不会改变 `/rebotarm/joint_states`；默认阶段
等待为 0 s，RViz 状态显示间隔为 0.03 s。

检查：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_candidates --once
ros2 topic echo /grasp/filtered_plan --once
```

`valid=true` 只代表软件计划可用，不代表真机抓取成功。该入口不启动
MuJoCo 物理后端；已删除独立的“真实感知 + MuJoCo 执行”顶层组合入口。
此外，相机安装在末端时，无硬件模式只能使用配置的假关节角计算
`base_link -> camera_depth_frame`；除非实体机械臂恰好处于相同角度，否则 RViz 中的
基座坐标不会与实物位置重合。要验收实物定位，必须读取真机关节反馈（保持失能）并重新
核对手眼标定，不能用本模式的假状态作为结论。

## 稳定性 benchmark

软件或 MuJoCo 环境可运行离线 benchmark；它不启动真机控制器：

```bash
ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark \
  --attempts 20 --return-ready-before-each --wait-enter
```

benchmark 输出包含 `failed_stage`，用于区分感知、规划、执行和回位阶段的失败。

仓库仍保留 `rebotarm_hybrid_grasp_sim_benchmark` 可执行程序，但已删除为它提供
“真实感知 + MuJoCo 执行”一键组合的顶层 launch；如需使用，必须由操作者自行组合并确认
唯一的仿真执行后端。该程序仍支持 `--return-ready-after-each`、
`candidate_max_joint6_delta_rad:=1.5708`、
`candidate_joint6_symmetry_enabled:=true` 和 `gripper_grasp_enabled:=false`。

该 `rebotarm_visual_grasp_benchmark` 只统计候选、规划和仿真/软件执行结果；
不要把 benchmark 通过解释为真实夹取成功。

如需单独回到视觉准备位（仍需对应后端已启动）：

```bash
ros2 service call /rebotarm/visual_ready/move std_srvs/srv/Trigger "{}"
```

## MuJoCo 执行验证

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py
```

确认 `/rebotarm/follow_joint_trajectory` 只有一个 Action server 后，在 RViz 先
Plan，再 Execute。停止测试：

```bash
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

## 真机视觉执行边界

真机启动前必须完成 MotorBridge/反馈检查、控制器 `CONNECTED_DISABLED`、视觉
plan-only、现场净空、急停、支撑和串口唯一占用确认。控制器启动后仍须显式使能：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"
```

当前不提供自动 Enable、自动抓取、lift/retreat 或结果分类的默认命令。
真机视觉执行服务为 `/rebotarm/visual_grasp/execute`；当前仅在完成分级验收并取得
明确授权后调用，不在本手册中提供自动触发命令。

## 配置与排错

- Ubuntu 相机：`src/rebotarm_vision/config/camera_ubuntu.yaml`
- Ubuntu GraspNet：`src/rebotarm_vision/config/graspnet_ubuntu.yaml`
- 主视觉 launch：`src/rebotarm_bringup/launch/visual_grasp_system.launch.py`
- 安装与解释器：`ubuntu_vision_setup_zh.md`、`launch_python_configuration.md`
- 真机反馈和 MoveIt Execute 排错：`local_setup_zh.md`

若出现 `MoveItSimpleControllerManager` 不存在，先安装对应 ROS 包；若出现
`verified feedback pending`，先停止其他串口程序并运行 SDK 只读反馈检查。六个
电机 sequence 不要求相等。
