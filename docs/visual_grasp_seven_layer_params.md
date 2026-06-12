# 视觉抓取七层参数设计

当前目标是通用抓取，不限制物体只能平躺或竖放。候选来源统一使用 GraspNet candidates，YOLO 仍可用于目标检测、mask、深度质量辅助，但不再生成规则物体候选，也不再合并两路候选。

主路线：

```text
YOLO / depth / point cloud
GraspNet Baseline 生成候选
candidate_ik_filter 做 IK / workspace / jaw_width / joint6 / table safety 检查
visual_grasp_executor 做 trajectory / gripper / lift / retreat 执行
```

## 第 1 层：Perception Source

作用：决定视觉数据和 GraspNet 候选来源。

```yaml
start_vision: true
ordinary_depth_quality_enabled: true
start_graspnet_baseline: true
graspnet_candidates_topic: /grasp/graspnet_candidates
candidate_ik_input_topic: /grasp/graspnet_candidates
```

- `start_vision`：启动 Ubuntu 视觉节点，接收 Windows 相机、YOLO、depth、camera_info。
- `ordinary_depth_quality_enabled`：开启 YOLO/depth 路线的深度质量检查，默认保持开启。
- `start_graspnet_baseline`：启动 GraspNet 候选输入节点，当前通常是 network 模式读取 Windows bridge 输出。

## 第 2 层：Candidate Source

作用：控制 GraspNet 候选数量和数据来源。

```yaml
graspnet_source_mode: network
graspnet_candidates_url: http://192.168.145.1:8081/graspnet_candidates.json
graspnet_network_poll_hz: 0.5
candidate_max_candidates_per_frame: 20
```

- Windows bridge 每次推理会写入 `graspnet_candidates.json`。
- Windows bridge 当前使用整张 scene 点云做 GraspNet 推理，再用 YOLO mask / bbox 后筛选最终 candidates。
- Ubuntu 侧读取 `/grasp/graspnet_candidates`，最多取 `candidate_max_candidates_per_frame` 个进入 IK filter。
- Open3D 可视化只用于调试，不作为 Ubuntu 抓取执行链路的一部分。

## 第 3 层：Candidate Target

作用：把候选转换成 `pregrasp_pose` 和 `grasp_pose`。

```yaml
candidate_pose_policy: preserve_candidate_pose
base_pregrasp_distance_m: 0.06
tcp_offset_xyz: [-0.04, 0.0, 0.0]
target_base_offset_xyz: [0.0, 0.0, 0.0]
```

- `preserve_candidate_pose`：优先保留 GraspNet 给出的 6D 抓取姿态。
- `base_pregrasp_distance_m`：pregrasp 离 grasp 的退让距离，当前 6cm。
- `tcp_offset_xyz`：夹爪抓取中心相对 `end_link` 的局部偏移，当前恢复为 `[-0.04, 0.0, 0.0]`。
- `target_base_offset_xyz`：在 base_link 下对目标点做全局补偿，当前不使用。

## 第 4 层：Pose Variant / Joint6

作用：限制 joint6 大幅正反转，同时允许夹爪 180 度等价姿态参与选择。

```yaml
candidate_joint6_symmetry_enabled: true
candidate_joint6_symmetry_angle_rad: 3.141592653589793
candidate_max_joint6_delta_rad: 1.5708
candidate_score_joint_distance_weight: 0.15
candidate_score_joint6_weight: 0.35
```

- `candidate_max_joint6_delta_rad: 1.5708` 表示 joint6 相对当前姿态最多允许变化 90 度。
- `candidate_joint6_symmetry_enabled` 会把 180 度等价夹取姿态也送进 IK 候选，不是直接强行改原始姿态。
- `candidate_score_joint_distance_weight` 和 `candidate_score_joint6_weight` 是评分权重，动作越小、joint6 转动越小，分数越高。

## 第 5 层：Workspace / Table Safety

作用：过滤明显不安全或超出机械臂工作空间的候选。

```yaml
candidate_workspace_gate_enabled: true
candidate_workspace_min_xyz: [0.18, -0.35, 0.0]
candidate_workspace_max_xyz: [0.64, 0.35, 0.45]
candidate_min_grasp_z_m: 0.0
candidate_safe_lift_min_z_m: 0.120
safe_retreat_min_lift_z_m: 0.12
```

- 不再使用 `0.12m` 作为抓取点最低硬门槛，低高度物体可以进入候选。
- `candidate_safe_lift_min_z_m` 和 `safe_retreat_min_lift_z_m` 仍用于抬升/撤退安全检查。
- workspace 最大半径不超过机械臂约 64cm 工作范围。

## 第 6 层：Gripper Policy

作用：根据候选 jaw_width 控制夹爪开合和夹取检测。

```yaml
open_before_approach: true
auto_gripper_width: true
auto_gripper_effort: true
open_clearance_m: 0.0
max_allowed_grasp_width_m: 0.082
close_max_effort: 0.4
gripper_grasp_enabled: true
gripper_grasp_close_force: 0.4
gripper_grasp_timeout_sec: 8.0
```

- `auto_gripper_width`：使用 GraspNet 候选里的 `jaw_width` 控制开爪宽度。
- `open_clearance_m: 0.0`：不再额外加开口余量。
- `gripper_grasp_timeout_sec`：夹爪多久没有检测到有效接触，就判定 close 阶段失败。

## 第 7 层：Motion Feasibility / Execution

作用：确认候选能被机械臂实际走完整流程。

```yaml
start_candidate_ik_filter: true
executor_input_topic: /grasp/filtered_plan
trajectory_precheck_enabled: true
moveit_planning_time: 8.0
moveit_num_planning_attempts: 5
safe_retreat_enabled: true
safe_home_after_grasp: false
```

- IK 可达不等于轨迹可执行，所以实机抓取建议保持 `trajectory_precheck_enabled: true`。
- 最终执行链路仍是 `pregrasp -> grasp -> close -> lift -> retreat`。
- `safe_home_after_grasp: false` 表示抓完不自动回 safe_home，便于连续调试。
