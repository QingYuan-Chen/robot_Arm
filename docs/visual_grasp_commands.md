# 视觉抓取启动手册

> 当前主路线为单 Ubuntu：Gemini 2、YOLO、GraspNet 和 ROS 2 均在 Ubuntu 本机运行。
> Windows HTTP/MJPEG/GraspNet 中转工具已从仓库删除，不再提供 Windows 启动命令或验收路径。
> 任何真机执行仍需单独授权和现场安全预检。

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
export ROS_DOMAIN_ID=100
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

## Ubuntu 原生视觉与 MuJoCo plan-only

```bash
source tools/source_local_environment.bash
export ROS_DOMAIN_ID=101
export RMW_FASTRTPS_USE_SHM=0
ros2 launch rebotarm_bringup real_perception_sim_execution.launch.py \
  use_hardware:=false execution_mode:=plan_only \
  move_to_visual_ready_on_start:=false start_visual_grasp_executor:=false
```

检查：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_candidates --once
ros2 topic echo /grasp/filtered_plan --once
```

`valid=true` 只代表软件计划可用，不代表真机抓取成功。

## 稳定性 benchmark

软件或 MuJoCo 环境可运行离线 benchmark；它不启动真机控制器：

```bash
ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark \
  --attempts 20 --return-ready-before-each --wait-enter
```

benchmark 输出包含 `failed_stage`，用于区分感知、规划、执行和回位阶段的失败。

需要真实视觉输入但只连接仿真执行后端时，使用混合 benchmark：

```bash
ros2 run rebotarm_vision rebotarm_hybrid_grasp_sim_benchmark \
  --attempts 1 --plan-timeout-sec 10 --service-timeout-sec 180 \
  --return-ready-after-each
```

仿真候选链建议保持 `candidate_max_joint6_delta_rad:=1.5708` 和
`candidate_joint6_symmetry_enabled:=true`，并在 benchmark
输出中核对 `plan source=` 和 `failed_stage`。
仿真 benchmark 使用 `gripper_grasp_enabled:=false`，避免把夹爪结果混入运动规划统计。

该 `rebotarm_visual_grasp_benchmark` 只统计候选、规划和仿真/软件执行结果；
不要把 benchmark 通过解释为真实夹取成功。

如需单独回到视觉准备位（仍需对应后端已启动）：

```bash
ros2 service call /rebotarm/visual_ready/move std_srvs/srv/Trigger "{}"
```

## MuJoCo 执行验证

```bash
export ROS_DOMAIN_ID=102
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
