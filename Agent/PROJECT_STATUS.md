# reBotArm 项目实施完成度

> Source of truth / 完成度来源：`新项目规划.md`
> 基线日期：2026-08-06
> 说明：这里衡量的是有验收证据的实施完成度，不是代码量、文档完成度或主观进度。

## 计分规则

- 每个 P 阶段具有固定权重，总计 100；
- 阶段完成率 = 已勾选验收项 / 该阶段全部验收项；
- 总完成率 = 各阶段完成率乘以阶段权重后求和；
- checkbox 只有在代码、测试、运行记录或硬件验收证据成立后才能勾选；
- `STATE.json` 由 `update_state.py` 自动计算，不手工填写百分比；
- 硬件未连接或未实测时，相关项保持未完成。

## P0: 机械臂安全启动与执行门控 [weight=20]

- [x] 已存在显式 `/rebotarm/enable` 和 `/rebotarm/disable` ROS service；证据：`rebotarmcontroller/ros_services.py`。
- [x] `HardwareManager.connect()` 只建立通信，不调用 enable 或启动控制循环；证据：P0 hardware manager 测试。
- [x] enable 前主动刷新反馈并检查在线状态、有限值和软限位；证据：严格 feedback refresh、URDF 软限位一致性测试。
- [x] enable 设置当前关节为 hold target，并确认全部电机状态；证据：当前位置 hold 与 status 测试。
- [x] 部分电机使能失败时回滚所有已使能电机；证据：部分失败回滚测试。
- [x] 完整视觉抓取默认 `use_hardware=false`、`plan_only`、不自动 visual-ready/safe-home/enable；证据：launch wiring 测试。
- [x] 通过失能 joint states、显式 enable、当前位置保持、disable 和失败回滚验收；证据：Gate A 记录、`gate-bc-20260806-211949.json` 及此前失败后的安全 cleanup 报告。

## P1: MuJoCo 基线巩固与差距整合 [weight=20]

- [x] 已有 MuJoCo 模型生成、model profile 和 headless smoke 基础设施。
- [x] 已有独立 MuJoCo ROS adapter 和 `FollowJointTrajectory` 接口。
- [x] 已有 trajectory metrics、summary 和 step-response suite。
- [x] 已有 path/goal tolerance 失败记录和返回逻辑。
- [x] 已有 URDF position limit 与 MuJoCo ctrlrange 第一层测试。
- [x] 已有 launch 可选 viewer，并保留 RViz fake sim。
- [x] 固定并分类上游 repository URL、branch/tag、commit SHA 和 license；证据：`docs/mujoco_upstream_sources.md`。无许可证文件的 HJX 来源标记 `NOASSERTION` 并从默认模型依赖移除。
- [x] 完成 current/upstream gap matrix，确认只选择性整合缺失能力；证据：`docs/mujoco_gap_matrix.md`、`tests/test_mujoco_gap_matrix.py`。
- [x] 修正 grasp scene keyframe 的不合理起始姿态；scene home `qpos` 与 `ctrl` 已同步，证据：`Agent/evidence/P1/2026-08-06-grasp-keyframe.md`、`tests/test_mujoco_model_profile.py`。
- [x] 固定 `huangbinai/robotarm_ros2` MuJoCo 快照并完成目标文件与 license evidence 审计；证据：`Agent/evidence/P1/2026-08-06-robotarm-ros2-preflight.md`、`Agent/evidence/P1/2026-08-07-upstream-snapshot-localization.md`，固定 `main@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`，license 结果为 `PROVISIONAL`。
- [x] 独立运行上游 health/headless/model/ROS/motor/collision 测试并形成双基线差异报告；证据：`Agent/evidence/P1/2026-08-07-baseline-comparison.md` 与 JSON。上游为 `219 passed, 1 skipped, 1 failed`，唯一失败已分类为嵌套 `mapping -> tuple` 的测试断言缺陷，未宣称上游全绿。
- [x] 将固定上游 MuJoCo runtime、模型、ROS adapter 和 launch 直接本地化到 `src/rebotarm_simulation`；切换前的 current source 已归档到 `third_party/rebotarm_simulation_current_baseline`。最终 active runtime 收口为 upstream-only，安全边界与单 Action server 约束通过测试和 ROS preflight 验证。证据：`Agent/evidence/P1/2026-08-07-upstream-src-swap.md`、`third_party/rebotarm_simulation_current_baseline/ARCHIVE_MANIFEST.json`。
- [x] 将 timeout 接入执行循环；使用 monotonic wall-clock deadline，超时返回非成功结果并 hold 当前位置，证据：`tests/test_mujoco_adapter_core.py`、`tests/test_mujoco_ros_adapter_launch.py`。
- [x] execute-loop 集成测试覆盖成功、取消、停止、path/goal 容差失败和超时；ROS 2 + MuJoCo 环境 `6 passed`，证据：`Agent/evidence/P1/2026-08-07-execute-loop-integration.md`。
- [x] 补齐 effort、velocity、acceleration 和 jerk 限制一致性；URDF effort / MuJoCo forcerange 一致，MoveIt planner limits 保守且由 `rebotarm_motion` runtime guard 执行，证据：`Agent/evidence/P1/2026-08-07-runtime-limit-consistency.md`。
- [x] 按用户最终决策将 current fallback 退出 active runtime 和 P1 验收范围；未把其 joint4-6 tracking/contact 标定误记为成功。旧实现仅保存在 `third_party/rebotarm_simulation_current_baseline` 归档，active setup/launch 不再暴露 current adapter、legacy CLI 或 backend selector；证据：`Agent/evidence/P1/2026-08-07-p1-upstream-only-closeout.md`。

## P2: Gemini 2 SDK 与真实 RGB-D 验收 [weight=15]

- [x] 已有 Ubuntu 视觉环境、Orbbec SDK 安装和 udev 脚本。
- [x] `Gemini2Driver` 已有 RGB、uint16 depth 和硬件对齐采集代码。
- [ ] 真实 Gemini 2 在当前 Ubuntu 上稳定输出 RGB 和 depth。
- [ ] 从 SDK 读取设备信息和实际 stream profiles。
- [ ] 从 SDK 读取 intrinsics、distortion 和 RGB-depth extrinsics。
- [ ] 获取并正确应用 depth scale，明确 ROS 与内部深度单位。
- [ ] 验证 RGB-D 对齐、分辨率变化和 CameraInfo 一致性。
- [ ] 完成多个已知距离样本的深度误差验收记录。

## P3: 完整单 Ubuntu 视觉链路 [weight=15]

- [x] 已有 `camera_ubuntu.yaml` 和 `vision_ubuntu.launch.py`。
- [x] 已有 Ubuntu 本地 CUDA YOLO 环境和独立启动脚本。
- [ ] 完整 bringup 支持显式 `vision_profile:=ubuntu_native`。
- [ ] 一次安全 launch 发布 RGB、depth、CameraInfo、detections 和 annotated topics。
- [ ] 完整 Ubuntu native 链路不依赖 Windows 路径或 Windows 服务。
- [ ] 时间戳、frame_id、TF 和候选新鲜度检查通过。
- [ ] 视觉失效或过期候选能 fail closed，不触发执行。

## P4: Ubuntu 本地 GraspNet [weight=10]

- [x] 已有 GraspNet candidate topic、network client 和 localhost 配置入口。
- [x] 已有 GraspNet baseline inference/bridge 的参考工具代码。
- [ ] 建立独立 `.venv-graspnet` 和固定依赖。
- [ ] 新增 Ubuntu setup/run 脚本与安装文档。
- [ ] localhost GraspNet 服务提供健康检查并可独立启停。
- [ ] 输入输出 contract 包含 RGB、米制深度、内参、bbox、时间戳和 frame_id。
- [ ] 候选稳定性、过期、无候选和 TF 失败的 fail-closed 验收通过。

## P5: 标定 [weight=10]

- [x] 已有 hand-eye 配置加载与 TF 发布机制，但数据仅视为历史配置。
- [ ] 验证 SDK 相机内参、depth scale 和不同 profile。
- [ ] 对当前实际安装重新完成 hand-eye 标定与多姿态稳定性验收。
- [ ] 完成 `end_link -> grasp_tcp` TCP 标定。
- [ ] 完成 MuJoCo 关节轴、零位、link、mesh、夹爪和场景几何标定。
- [ ] 基于真实规格或实测数据完成 MuJoCo 动力学与接触参数标定。

## P6: 系统集成与最终验收 [weight=10]

- [x] 已有真实感知 + 仿真执行的历史 launch/benchmark 基础。
- [ ] 使用当前 Ubuntu native 感知完成真实视觉 + MuJoCo 验收。
- [ ] 增加并验收 MuJoCo 虚拟 RGB-D、CameraInfo 和物体标注。
- [ ] 完成完全离线 YOLO/GraspNet/MoveIt/MuJoCo 抓取闭环。
- [ ] 完成安全实机 plan-only 验收。
- [ ] 逐级完成单关节、安全姿态、pre-grasp、approach、gripper、lift 和 retreat 低速验收。
- [ ] 完成自动抓取及空抓、滑落、未抬起和成功分类。
- [ ] 无候选、TF、碰撞、规划、容差、超时和通信异常全部进入安全停止。
