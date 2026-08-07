# reBotArm Agent 实时记忆

> 本文件只保存当前仍有效的事实、决策、阻塞和交接信息。详细历史见 `codex_memory.md`、旧 `项目规划.md` 和 Git history。

## 当前焦点

- Active phase / 当前阶段：P1 MuJoCo 基线巩固与差距整合。
- Completed phase / 已完成阶段：P0 机械臂安全启动与执行门控已通过软件及分级真机验收。
- Hardware gate / 硬件门：P0 已解除；完整视觉抓取仍需遵守后续 P2-P6 感知、标定和分级系统验收门。

## 当前事实

- 当前分支：`codex/mujoco-sim-landing`。
- 当前 HEAD 基线：`5bd5510`，`Improve MuJoCo execution validation`。
- `HardwareManager.connect()` 已只建立通信、主动刷新/校验反馈并保持全部电机失能，不再 enable 或启动控制循环。
- 显式 `enable()` 会先验证反馈和软限位、设置当前位置 hold target、确认全部状态；失败时停止循环并失能回滚。
- 动作目标和控制入口在显式 enable 前 fail closed；夹爪初始化不再隐式 enable 或启动循环。
- 完整视觉抓取 launch 当前默认 `use_hardware=false`、`execution_mode=plan_only`，不自动 visual-ready 或 shutdown safe-home。
- 真机串口桥批量发送六个 feedback request 后单次 poll 不能可靠收齐响应；当前实现已改为逐电机 `request -> poll` 并对夹爪重试。
- Gate B 重复报告中的异常只出现在反馈数据：joint4-6 曾返回完全相同的 `-12.467574 rad` 和 `-23.435898 rad/s` 固定边界值，随后恢复基线；现场确认机械臂没有实际运动。
- arm、gripper 和 joint-state publisher 共用串口控制器；反馈刷新现已在 controller `RLock` 内完成整个逐电机事务，enabled hold 期间也主动刷新并校验状态/软限位，异常帧不会发布为 joint states。
- 显式 arm enable 不再启动闲置夹爪 500 Hz loop；夹爪 loop 只在收到真实夹爪 target/grasp 命令时启动。Gate B/C 验收增加 joint-state stale 检测，避免无有效样本时误通过。
- Gate B/C 已提供 `ros2 run rebotarmcontroller p0_gate_bc_acceptance` 专用工具；只有精确输入确认词后才显式 enable，并监控位置跳变/速度、自动 disable 和写入 JSON 证据，不发送轨迹、safe-home 或夹爪命令。
- MuJoCo 基础闭环、ROS adapter、metrics、容差检查、step-response 和 viewer 开关已经存在。
- Ubuntu Gemini 2 + YOLO 独立入口已经存在；SDK 内参、depth scale 和真实 RGB-D 尚未在当前路线完成硬件验收。
- 完整视觉抓取尚未集成 `ubuntu_native` profile；Ubuntu 本地 GraspNet 服务尚未完成。
- hand-eye 配置加载机制存在，但当前安装需要重新标定；TCP 和 MuJoCo 动力学标定未完成。
- P0 收尾完成：driver/controller 已在失能状态正常退出，ROS 2 daemon 已停止，`/dev/ttyACM0` 无进程占用；验收报告和 ROS logs 保留。
- 已清理源码侧 33 个 `__pycache__`、342 个 `.pyc/.pyo`、`.pytest_cache` 和 `/tmp/rebotarm_ros2`；未删除 `build/`、`install/`、`log/` 或验收证据。参考上游中受 Git 跟踪的历史 `.pyc` 已恢复，嵌套仓库保持 clean。
- P1 候选上游本机快照为 `https://github.com/HJX-exoskeleton/reBotArm_develop_hjx.git`，`master@bcd584ce4a64eb116d80f6873c1756eddb8520bf`（2026-06-09，subject `update`）。
- HJX 候选上游固定 commit 的 license status 为 `NOASSERTION`，已拒绝作为代码/资产来源；默认模型不再依赖该目录，只允许行为级观察。
- 可使用来源已固定：MuJoCo 3.3.0 tag commit `8de9b4e...`（Apache-2.0）、本仓库 Apache-2.0 URDF/MJCF/meshes、Seeed 官方资料 commit `c3a75bd...`（硬件 CERN-OHL-W-2.0，软件需逐文件确认 Apache-2.0）。
- MuJoCo venv 直接依赖已固定；当前 `mujoco 3.3.0`、`numpy 1.26.4`、`scipy 1.11.4`、`cffi 1.17.1` 可导入，`pip check` 通过。
- package-owned 模型专项测试 36 项通过；生成模型为 `nq=8`、`nv=8`、`nu=7`，robot smoke 和 grasp scene 均 finite，生成 XML 不含 HJX 路径。
- 当前 1 秒六轴 step-response 仍有 `max_final_abs_error=0.7860 rad`、`max_abs_error=1.5683 rad`，属于后续 tracking/dynamics calibration 工作，不因环境和授权修复而视为通过。
- P1 current/upstream capability gap matrix 已建立于 `docs/mujoco_gap_matrix.md`，并由 `tests/test_mujoco_gap_matrix.py` 校验表头、关键能力行和授权边界；矩阵将 grasp keyframe、timeout/execute-loop、limits consistency 标为当前 P1 implement，将 tracking/contact/calibration 标为后续 defer。
- grasp scene keyframe 已修正：命名 keyframe `0` 现在同时保存 home arm `qpos` 与对应 `ctrl`，避免 reset 后 position actuator 默认回零导致姿态漂移；1 秒 benchmark `finite=True`，但仍为 `contact_without_lift`，因为当前 benchmark 不发送抬升轨迹，接触/抓取质量仍未完成标定。
- P1 迁移策略已按用户最新决定落地：`huangbinai/robotarm_ros2` 的 `main@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4` 作为默认 MuJoCo backend，当前 package-owned MuJoCo 保留为显式 `simulation_backend:=current` fallback/对照基线；上游包含 health/headless、viewer、ROS adapter、URDF-to-MJCF、motor control、collision/contact 和 acceptance 基础设施。
- `robotarm_ros2/src/rebotarm_simulation/package.xml` 声明 Apache-2.0，但固定仓库根 LICENSE 文件尚未核验到；已允许当前工作区本地使用默认 upstream backend，仍禁止在授权确认前对外再分发该快照。
- 用户已提供本机 clean 快照 `/home/a/project/rebot_refer`，remote 为 `https://github.com/huangbinai/robotarm_ros2.git`，固定 `main@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`。目标包文件清单、SHA-256 和授权结果记录于 `Agent/evidence/P1/2026-08-06-robotarm-ros2-preflight.md`。
- 已按 upstream-first 选择性本地化 health check：`rebotarm_simulation.mujoco_health.check_model_health()` 与 `rebotarm_mujoco_health` console script 只检查当前 package-owned XML；TDD、CLI、分层、全量测试和 compileall 均通过。证据：`Agent/evidence/P1/2026-08-07-health-check-localization.md`。
- URDF-to-MJCF 审查确认上游 `robot.xml` 为 `nu=8/nsensor=26` 的独立 torque/finger actuator 模型，而当前生成模型为 `nu=7/nsensor=0` 并由 `sim_gripper` 保持夹爪契约；本轮决定不直接替换模型。证据：`Agent/evidence/P1/2026-08-07-model-converter-audit.md`。
- 用户确认采用 A 方案：将上游 MuJoCo runtime、模型、launch、测试、配置和文档完整保留到当前 Git 的 `third_party/robotarm_ros2_mujoco_snapshot/`，通过独立 comparison workspace 与当前 baseline 做 A/B 对比；现在已通过 `upstream_backend.py` 和 launch selector 将 upstream 设为默认，current 仍可显式回退。
- 上游完整 MuJoCo package 已按固定 commit 原样复制到 `third_party/robotarm_ros2_mujoco_snapshot/`；`diff -qr` 与 `/home/a/project/rebot_refer/src/rebotarm_simulation` 无差异，provenance manifest 和 localization README 已加入。该目录不在默认 `src/`，仍受 `PROVISIONAL` license 边界限制。
- A/B 对比已完成：两套 health/headless 均通过；current `nu=7/nsensor=0`，upstream `nu=8/nsensor=26`；1 秒 step-response 的 max final error 为 `0.7860` vs `0.6443 rad`，max velocity 为 `4.0664` vs `1.9567 rad/s`；两套 grasp smoke 均 contact 但均无 lift success。完整 JSON/解释见 `Agent/evidence/P1/2026-08-07-baseline-comparison.{json,md}`。
- 第二轮 command-level 对比已完成：两套 backend 接收同一六轴目标 `[1.4,-0.785,-0.785,0.710,0.785,1.570] rad` 与 `gripper_width=0.04 m`。Current adapter 的 max final error / max velocity 为 `0.7930 rad` / `5.2087 rad/s`，上游 native runtime 为 `0.3611 rad` / `1.9563 rad/s`；该结果只说明现有控制器响应差异，不能证明模型可直接替换。证据：`Agent/evidence/P1/2026-08-07-baseline-comparison-command-contract.json`，并已合并到主对比 JSON/Markdown。
- 用户确认保留上游左右独立 force actuator 语义，不再把上游夹爪改造成当前单耦合 gripper actuator，也不再为 actuator 对齐本身扩展测试；后续只在统一高层 command、明确 controller contract 的前提下比较结果。
- P1 timeout 已接入 `MuJoCoRosAdapterNode`：以 trajectory duration 加 `execution_timeout_margin_sec` 计算 monotonic wall-clock deadline；timeout 映射为非成功 action result，并与 cancel/stop/tolerance failure 一样将 arm targets hold 在当前位置。端到端 ROS action integration tests 仍待补齐。
- P1 execute-loop integration 已补齐：在 ROS 2 + MuJoCo venv 下 success、cancel、stop、path tolerance、goal tolerance、timeout 六种结果均通过，并验证失败路径 hold 当前姿态；证据：`Agent/evidence/P1/2026-08-07-execute-loop-integration.md`。系统 Python 因缺少 `rebotarm_msgs`/MuJoCo 会跳过该组测试，不能替代显式环境验收。
- P1 limit consistency 已补齐：URDF arm effort 与生成 MuJoCo `forcerange` 为 joint1-3=27、joint4-6=7；MoveIt planner velocity 为 joint1-3=3.0、joint4-6=1.8，均低于 URDF 50/200 硬件上限；acceleration=5.0、jerk=20.0 由 `rebotarm_motion` runtime guard 对实测 velocity/effort 做有限差分检查，违规 abort 并 hold。gripper effort 不做跨 contract 等式比较。证据：`Agent/evidence/P1/2026-08-07-runtime-limit-consistency.md`。
- 3 秒线性 ramp tracking audit 发现当前 joint4/joint5 仍有 `0.3437/0.7843 rad` 最终误差，joint5 峰值速度 `3.7198 rad/s`；上游同一高层命令原生 controller 的最大最终误差 `0.005234 rad`。当前 joint4-6 `forcerange=±7`，上游 arm torque limit 为 `±12.5`；在没有真实硬件证据前不自动提高当前力限或改 gains。证据：`Agent/evidence/P1/2026-08-07-joint4-6-tracking-audit.md`。
- 用户确认上游 arm force limit 可采用；已增加隔离 `UPSTREAM_ARM_MOTOR_PROFILES` / `motor_profile:=upstream_arm`，但在保留当前 position-actuator controller、gains 和 dynamics 的前提下，仅把 joint4-6 XML forcerange 改为 ±12.5，joint4/joint5 tracking residual 仍为 `0.3437/0.7843 rad`。因此上游优势来自完整 cascaded torque controller、gravity compensation、rate limiting/filtering 和 dynamics，不是 XML force ceiling 单项；当前默认仿真已改走完整 upstream runtime。

## 当前决策

- `新项目规划.md` 是当前主规划，旧 `项目规划.md` 是 MuJoCo 历史记录。
- 现有 package-owned MuJoCo 实现保留为 current fallback；固定 upstream 快照通过独立进程边界成为默认 backend，不覆盖快照源码，也不同时启动两个仿真 Action server。
- P0 安全优先于任何实机视觉联调。
- GraspNet 第一阶段采用 `.venv-vision` + `.venv-graspnet` 双环境和 localhost 服务。
- 自动测试、仿真验收和真机验收分别记录，不能互相替代。

## 当前阻塞

- P2：真实 Gemini 2 RGB-D、SDK 标定和 depth scale 缺少当前 Ubuntu 硬件验收证据。
- P4：Ubuntu 本地 GraspNet 环境、依赖和服务尚未建立。
- P5：当前实际安装的 hand-eye 和 TCP 数据尚未标定。
- P1：上游原始快照仍保留一个测试写法缺陷：`test_saved_integration_state_replays_deterministically` 直接对 `mapping[str, tuple]` 使用 `pytest.approx`；该缺陷只存在于测试断言，临时 test-only compatibility copy 已 `220 passed, 1 skipped`，不阻塞本地默认 backend，但不能宣称未修改上游测试原始全绿。
- P1：current fallback 在线性 ramp 下 joint4/joint5 仍有较大 tracking residual；默认 upstream controller 已通过 ROS 轨迹验收，current fallback 的 tracking/collision/contact calibration 仍待后续单独处理。

## 最近验证

- 2026-08-07：完成 upstream 默认 backend 切换：原始快照测试为 `219 passed, 1 skipped, 1 failed`，临时 test-only 断言兼容副本为 `220 passed, 1 skipped`；health/headless 通过；默认 `mujoco_moveit_sim.launch.py` 启动 upstream ROS node、`/clock`、joint states、trajectory action 和 gripper services；1 秒六关节轨迹返回 `error_code=0`。current adapter 仅在 `simulation_backend:=current` 时启动，非法 backend fail closed。退出时 MuJoCo process clean，但两条 backend 的 `move_group` 都超过 5 秒 SIGINT grace 后被 SIGTERM，记录为非阻塞 MoveIt cleanup 问题。证据：`Agent/evidence/P1/2026-08-07-upstream-default-backend.{md,json}`。
- 2026-08-06：P1 授权风险修复：新增 package-owned MJCF baseline/grasp scene，默认 mesh 只解析到本仓库 `rebotarm_bringup`；HJX `NOASSERTION` 来源退出默认运行路径。新模型 `nq=8`、`nv=8`、`nu=7`，1 秒 robot smoke 与 grasp scene 均 finite。
- 2026-08-06：P1 环境风险修复：直接依赖全部固定，NumPy 降至 1.26.4 并补 cffi 1.17.1；`pip check` 无 broken requirements，MuJoCo/NumPy/SciPy/cffi import 通过。
- 2026-08-06：P1 授权/环境修复完整回归：MuJoCo 专项 `36 passed`，仓库完整测试 `486 passed, 3 skipped`，required compileall、`rebotarm_simulation` colcon build、`pip check` 和 diff check 通过。
- 2026-08-06：P1 gap matrix 完成：新增 `docs/mujoco_gap_matrix.md` 与结构测试；只记录 current/upstream 能力差异，不复制 `reBotArm_develop_hjx` 实现或资产。
- 2026-08-06：P1 grasp scene keyframe 完成：scene reset 的 home `qpos/ctrl` 同步测试通过，生成与 1 秒 grasp benchmark 均 finite；`contact_without_lift` 保留为后续接触和执行轨迹标定边界。
- 2026-08-06：P1 规划重构：确认采用 `robotarm_ros2` MuJoCo upstream-first 迁移；新增固定快照、上游独立预检、双基线差异报告和选择性本地化验收项，当前 P1 总项扩展为 16 项。
- 2026-08-06：使用用户提供的 `/home/a/project/rebot_refer` 完成上游独立预检；health/headless 通过，MuJoCo 专项为 `219 passed, 1 skipped, 1 failed`。唯一失败为嵌套 mapping/tuple 的 `pytest.approx` 重放断言，数值最大差约 `7.05e-18`，保留为上游测试缺陷，不宣称全绿。
- 2026-08-07：完成第一个选择性本地化项 health check；新增 finite-state 检查器和 CLI，当前 MuJoCo 专项 `34 passed, 4 skipped`，全量 `489 passed, 4 skipped`。
- 2026-08-07：完成 URDF-to-MJCF/model 接口审查；确认 actuator、sensor、scene contract 不同，暂不整体迁移上游模型。
- 2026-08-07：用户确认完整上游本地快照方案；已提交 `docs/superpowers/specs/2026-08-07-upstream-full-localization-design.md`，等待用户审阅后再写实施计划。
- 2026-08-07：完成 upstream MuJoCo 完整快照本地化；provenance test `2 passed`、package diff clean，准备添加隔离 runner 和 A/B harness。
- 2026-08-07：用户确认方案 A 并完成 source swap：切换前的 `src/rebotarm_simulation` 已字节级归档到 `third_party/rebotarm_simulation_current_baseline/`，`ARCHIVE_MANIFEST.json` 对 20 个源/资源文件做 size/SHA-256 校验；上游模型、核心 runtime、ROS node、config、launch 已直接进入 active `src/rebotarm_simulation`。默认 `mujoco_moveit_sim.launch.py` 启动 `rebotarm_mujoco_node`，旧 current adapter 仅由 `simulation_backend:=current` 启用。
- 2026-08-07：source swap 验证完成：active headless 5-step、health（MuJoCo 3.3.0，8 joints/8 actuators）、upstream ROS action `error_code=0 / SUCCEEDED`、current fallback launch、package layering 18、全量测试 `516 passed, 13 skipped`、compileall、colcon build 和 `git diff --check` 均通过；未执行任何硬件命令。上游原始测试的 1 个 `pytest.approx` 嵌套 mapping/tuple 断言缺陷仍单独记录。
- 2026-08-07：完成隔离 runner 与 A/B harness；current/upstream health、model、1 秒 trajectory、gripper/contact 和 upstream test summary 已写入可复核报告。
- 2026-08-07：完成同一 command contract 复测；保留 actuator/controller contract 差异，未覆盖默认模型或上游快照。
- 2026-08-06：P0 现场已清理：停止 disabled driver/controller 与 ROS 2 daemon，串口无占用；清除源码 Python/pytest 和临时 channel override 缓存，保留构建、安装、日志和验收证据。
- 2026-08-06：P1 preflight 定位本机候选上游 `master@bcd584ce...`，嵌套 repo clean；MuJoCo 3.3.0 import 和 ROS package prefix 正常，但 license 文件缺失且 venv `pip check` 暴露两项依赖问题。
- 2026-08-06：P0 Gate B/C 真机报告 `gate-bc-20260806-211949.json` 为 PASSED：10 秒 hold、222 个 joint samples，最大位置跳变 `0.000381 rad < 0.03 rad`，最大绝对速度 `0.007326 rad/s < 0.05 rad/s`；enable/disable 均成功，最终六轴 status 0、`enabled=false`、控制循环停止。随后独立读取 `/rebotarm/arm_status` 再次确认安全失能。
- 2026-08-06：结合 Gate A、Gate B/C PASSED、此前失败报告的 stop/disable 安全 cleanup 和自动化部分使能回滚测试，P0 全部 7 项验收完成。
- 2026-08-06：重新上电后的 `gate-bc-20260806-211156.json` 仍因 joint6 瞬时 `-12.467574 rad` / `-23.435898 rad/s` 失败；用户确认机械臂没有移动。报告最终六轴状态全 0、`enabled=false`、`control_loop_active=false`，stop/disable cleanup 成功。
- 2026-08-06：针对 shared serial bus 反馈交错，增加整段反馈事务锁、enabled 状态主动反馈校验、闲置夹爪 loop 延迟启动和验收 joint-state stale gate；聚焦测试 21 项、完整测试集 `485 passed, 3 skipped`、compileall 和 `colcon build --packages-select rebotarmcontroller` 通过。构建期间未调用真机 enable。
- 2026-08-06：首次 Gate B 报告 `gate-bc-20260806-210112.json` 为 FAILED：夹爪状态未进入 1；cleanup 的 trajectory_stop/disable 均成功，最终六轴状态全 0、`enabled=false`、控制循环停止。
- 2026-08-06：将夹爪 enable 顺序修正为“失能态 MIT -> arm broadcast enable -> gripper 单电机 enable -> 状态确认”，并增强 arm/gripper 双重 disable 与峰值关节记录；构建后完整测试集 `483 passed, 3 skipped`，尚未再次调用真机 enable。
- 2026-08-06：Gate B/C 验收工具、纯状态判定、console entrypoint、确认门和 fail-safe cleanup 测试通过；构建及 `--help` 验证通过，完整测试集 `482 passed, 3 skipped`，准备阶段未打开串口或调用 enable。
- 2026-08-06：P0 Gate A 真机通过：启动日志为 `CONNECTED_DISABLED`，六轴状态码全 0，`enabled=false`、`control_loop_active=false`、`state_machine=IDLE`，joint states 平均 19.999 Hz 且全部数值有限；全程未调用 enable。
- 2026-08-06：motorbridge 只读扫描确认 joint1-6 和 gripper 的 ID/feedback ID 全部在线；逐电机反馈收集修复后完整测试集 `477 passed, 3 skipped`。
- 2026-08-06：Gate A 失能退出后进程和串口均已释放；ROS executor 在 Ctrl+C 时仍打印 context invalid/KeyboardInterrupt traceback，作为非阻塞清理项保留。
- 2026-08-06：安装精确匹配 `VID=2e88`、`PID=4603`、`serial=00000000050C` 的 udev rule；`/dev/ttyACM0` 已为 `root:plugdev`、`0660`，当前用户读写检查通过。
- 2026-08-06：首次 P0 Gate A 真机启动在打开 `/dev/ttyACM0` 时因权限拒绝退出；设备存在、无其他进程占用，且未到达 enable 流程。
- 2026-08-06：P0 hardware manager、action gate、分层和 launch cleanup 聚焦测试 36 项通过；连接不隐式使能、失能反馈刷新、主动反馈失败、软限位、hold 和回滚均有自动化覆盖。
- 2026-08-06：P0 软件门落地后的完整测试集 `475 passed, 3 skipped`。
- 2026-08-06：Agent 状态生成器、P0-P6 权重和实时字段测试通过；与分层测试合计 20 项通过。
- 2026-08-06：Agent 和分层 Python package `compileall` 通过。
- 2026-08-06：架构与主要 MuJoCo 单元测试 44 项通过。
- 2026-08-06：更新 docs 后，`tests/test_package_layering.py` 18 项通过。
- 2026-08-06：docs 相对链接、Markdown fence 和 `git diff --check` 通过。

## 下一次交接

1. P1 默认仿真 backend 已切到 upstream；继续使用 upstream 模型/controller 做仿真验证，current 仅作显式 fallback 对照；
2. 上游原始重放断言仍是测试缺陷，临时 test-only compatibility copy 已全绿，但不能把未修改上游测试报告写成全通过；
3. HJX 只允许行为级观察，不复制实现或资产；
4. current fallback 的 joint4-6 tracking、collision/contact 和抓取质量标定仍未完成；
5. 每完成一个验收项，更新 `PROJECT_STATUS.md` 并运行 `update_state.py`；
6. 不触碰工作区中与当前任务无关的已有 RViz 修改。
