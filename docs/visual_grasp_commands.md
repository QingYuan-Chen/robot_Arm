# reBotArm 视觉抓取启动手册

> 当前安全状态：P0-P6已按用户确认的工程范围关闭，未执行的完整自动抓取、结果分类和剩余实机动作不计为通过。新部署与每轮真机动作仍需独立预检和授权。先使用Ubuntu视觉独立验证、无硬件预览或显式plan-only检查。

安装、模型准备及分环境解释器以 [Ubuntu视觉说明](ubuntu_vision_setup_zh.md) 为准；
以下涉及历史本机模型的命令不能代替新部署的资产准备。

目标主路线是单 Ubuntu：

```text
Ubuntu Gemini 2 / YOLO / depth
-> Ubuntu in-process GraspNet ROS node
-> /grasp/graspnet_candidates
-> candidate_ik_filter
-> MoveIt validation
-> MuJoCo 或显式门控的真机后端
```

当前仍可使用的网络过渡路线是：

```text
Windows 或其他网络主机上的 YOLO / depth / GraspNet
-> Ubuntu /grasp/graspnet_candidates
-> candidate_ik_filter
-> /grasp/filtered_plan
-> RViz 仿真预览或真机执行
```

完整视觉抓取launch现已支持`vision_profile:=ubuntu_native`，并用同一个launch在独立
`.venv-graspnet` Python进程中直接运行GraspNet推理，不再经过localhost `/infer` HTTP；
专用hybrid入口固定接入active `rebotarm_mujoco_node`并关闭RViz-only controller。该运行时
接线通过不等于自动抓取或真实执行已经通过。修改launch、YAML、相机参数或AI后端后，
需要同步更新本文件。

## 0. 当前允许的 Ubuntu 原生视觉验证

只启动 Gemini 2 + YOLO，不启动机械臂：

```bash
cd /home/a/project/rebot_Arm
./tools/run_ubuntu_vision.sh yolo_model_path:="$PWD/tools/yolo26s-seg.pt" yolo_device:=0
```

检查：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once
ros2 topic echo /grasp/detections --once
```

目前看到 `CameraInfo` 不代表 SDK 内参和深度尺度已验收，具体要求见 `ubuntu_vision_setup_zh.md`。

## 1. 网络兼容链路：Windows 启动 YOLO 相机服务

PowerShell 终端 1：

```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main\softare\reBotArmController_ROS2-main"
.\tools\windows_start_yolo_server.ps1
```

浏览器检查：

```text
http://127.0.0.1:8081/video.mjpg
http://127.0.0.1:8081/annotated.mjpg
http://127.0.0.1:8081/depth.png
http://127.0.0.1:8081/camera_info.json
```

## 2. 网络兼容链路：Windows 启动 GraspNet

PowerShell 终端 2：

```powershell
cd "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main\softare\reBotArmController_ROS2-main"
.\tools\windows_start_graspnet_bridge.ps1
```

当前 `windows_start_graspnet_bridge.ps1` 已经固定：

```text
ManualTrigger=true
Open3DVisualize=true
VisualizeCropRadiusM=0
```

默认参数：

```text
MaxGrasps=50
VisualizeTopN=10
VisualizeMaxPoints=8000
```

运行后按 `y + Enter` 推理一次，写入：

```text
D:\tmp\graspnet_candidates.json
```

如果只想临时调整候选数量或显示点云数量：

```powershell
.\tools\windows_start_graspnet_bridge.ps1 `
  -MaxGrasps 50 `
  -VisualizeTopN 10 `
  -VisualizeMaxPoints 12000
```

Windows 侧当前策略：

```text
GraspNet 对整张 scene 点云推理
YOLO mask / bbox 只用于筛选最终 candidates
ROS 读取筛选后的 candidates 做 IK 过滤和执行
```

检查候选：

```powershell
curl http://127.0.0.1:8081/graspnet_candidates.json
```

Open3D 只用于 Windows 调试显示，不参与 ROS 真机执行。

## 3. Ubuntu 清理旧进程

确认机械臂安全静止后执行：

```bash
pkill -f rebotarm_table_collision || true
pkill -f rebotarm_visual_ready || true
pkill -f rebotarm_visual_grasp_markers || true
pkill -f rebotarm_ordinary_grasp_node || true
pkill -f rebotarm_graspnet_baseline_node || true
pkill -f rebotarm_grasp_tcp_frame || true
pkill -f rebotarm_grasp_candidate_ik_filter || true
pkill -f rebotarm_visual_grasp_executor || true
pkill -f rebotarm_motion_execution_node || true
pkill -f rebotarm_sim_trajectory_controller || true
pkill -f GripperVisualJointStateNode || true
pkill -f reBotArmController || true
pkill -f move_group || true
rm -f /dev/shm/fastrtps_port*
```

## 4. Ubuntu-native 真实视觉 + active MuJoCo

该入口使用真实Gemini 2、Ubuntu-native YOLO和由launch直接管理的GraspNet ROS进程生成候选，
不需要单独启动HTTP service。执行backend固定为active upstream `rebotarm_mujoco_node`。
真实controller不会启动；RViz-only `rebotarm_sim_trajectory_controller`也会显式关闭，
避免同namespace action/service冲突。

启动hybrid composition：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup real_perception_sim_execution.launch.py
```

默认自动使用仓库根目录的`.venv-graspnet/bin/python`、`.local-models/graspnet-baseline`
和`.local-models/checkpoints/checkpoint-rs.tar`。新机器或自定义路径可分别通过
`GRASPNET_PYTHON`、`GRASPNET_MODEL_ROOT`和`GRASPNET_CHECKPOINT_PATH`覆盖。

该命令会启动Gemini 2与MuJoCo runtime，使用前应先确认没有真实controller/backend进程。静态launch/test通过不等于该组合已经完成runtime acceptance。

执行1次仿真抓取benchmark：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarm_vision rebotarm_hybrid_grasp_sim_benchmark \
  --attempts 1 \
  --plan-timeout-sec 10 \
  --service-timeout-sec 180 \
  --return-ready-after-each
```

这条仿真 benchmark 已经默认固定：

```text
namespace=rebotarm_sim
min_success_rate=0
wait_enter=true
```

连续 20 次仿真 benchmark：

```bash
ros2 run rebotarm_vision rebotarm_hybrid_grasp_sim_benchmark \
  --attempts 20 \
  --plan-timeout-sec 10 \
  --service-timeout-sec 180 \
  --return-ready-after-each
```

仿真链路应保持：

```bash
gripper_grasp_enabled:=false
candidate_max_joint6_delta_rad:=1.5708
candidate_joint6_symmetry_enabled:=true
```

## 5. 真机实际抓取启动

> 阶段阻断：本节保留原链路参数用于迁移核对，不构成当前可执行操作。完成 P0、P2、P3、P4 和 P5 的对应验收前，不得启动完整真机抓取或调用执行服务。

确认 Windows YOLO 和 GraspNet 都已启动后，Ubuntu 终端 A：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false \
  execution_mode:=plan_only \
  move_to_visual_ready_on_start:=false \
  shutdown_safe_home:=false
```

旧手册曾使用以下真机联调配置；这里只保留用于迁移核对，不能将其视为当前代码事实或推荐值：

```text
use_hardware=true
use_local_rviz=true
execution_mode=execute
start_vision=true
ordinary_depth_quality_enabled=true
start_graspnet_baseline=true
graspnet_source_mode=network
graspnet_candidates_url=http://192.168.145.1:8081/graspnet_candidates.json
graspnet_network_poll_hz=0.5
candidate_ik_input_topic=/grasp/graspnet_candidates
start_candidate_ik_filter=true
executor_input_topic=/grasp/filtered_plan
candidate_pose_policy=preserve_candidate_pose
candidate_max_candidates_per_frame=20
candidate_workspace_gate_enabled=true
candidate_max_joint6_delta_rad=1.5708
candidate_joint6_symmetry_enabled=true
tcp_offset_xyz=[-0.04, 0.0, 0.0]
trajectory_precheck_enabled=true
open_before_approach=true
auto_gripper_width=true
auto_gripper_effort=true
gripper_grasp_enabled=true
gripper_grasp_timeout_sec=8.0
safe_retreat_enabled=true
safe_retreat_min_lift_z_m=0.12
lift_z_m=0.04
moveit_planning_time=8.0
moveit_num_planning_attempts=5
base_pregrasp_distance_m=0.06
safe_home_after_grasp=false
```

完成全部阶段门后，执行服务仍必须由操作者显式调用；当前禁止调用：

```bash
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

手动松开夹爪：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.08, max_effort: 0.25}"
```

## 6. 连续真机稳定性测试

> 当前禁止执行。只有单关节小角度、单个安全姿态、pre-grasp、approach、夹爪、lift 和 retreat 已逐级验收后，才能恢复连续真机测试。

每一轮会先回到 visual_ready，再等待你按 Enter 执行抓取，用于统计 `failed_stage` 和成功率：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark \
  --attempts 20 \
  --return-ready-before-each \
  --wait-enter
```

手动单独回到 visual_ready：

```bash
ros2 service call /rebotarm/visual_ready/move std_srvs/srv/Trigger "{}"
```

## 7. 查看候选和最终计划

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic echo /grasp/filtered_candidates --once
ros2 topic echo /grasp/filtered_plan --once
ros2 topic echo /camera/depth/camera_info --once
ros2 param get /rebotarm_grasp_candidate_ik_filter input_topic
```

期望 IK filter 输入：

```text
String value is: /grasp/graspnet_candidates
```

## 8. 只看感知和 RViz marker

不启动真机控制器，只看真实视觉、GraspNet candidates、IK 过滤结果和 marker：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py
```
