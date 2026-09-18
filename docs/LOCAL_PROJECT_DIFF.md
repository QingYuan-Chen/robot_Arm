# 本地项目基线替换与差异报告

日期：2026-09-13。范围：代码迁移、环境配置、构建、软件测试和只读差异分析。
没有启动真机控制器，没有打开机械臂串口，没有使能、校零、执行真机轨迹、视觉抓取或示教回放。
本报告中的“有功能”表示源码存在，不等于本机已完成硬件、性能或任务成功率验收。

## 1. 基线、备份与 Git 关系

| 项目 | 结果 |
| --- | --- |
| 唯一开发目录 | `/home/huangbin/robotarm_ros2` |
| 原目录完整备份 | `/home/huangbin/robotarm_ros2_backup_20260913_132122` |
| 原本地分支 / 提交 | `main` / `f2b0095a0da959b56e10140b292295b9ad9b8269` |
| 一次性来源 | `https://github.com/QingYuan-Chen/robot_Arm.git` 的 `main` |
| 采用的新基线提交 | `aa1dcc52c7e7046f8040c2b9becd767df4d07265` |
| 唯一正式 remote | `origin = https://github.com/huangbinai/robotarm_ros2.git` |
| 本地分支 / 跟踪分支 | `main` / `origin/main` |
| 个人仓库旧 main 备份 | `backup-before-rebase-20260913-132122`，指向 `f2b0095a0da959b56e10140b292295b9ad9b8269` |
| 初始状态记录 | `/home/huangbin/robotarm_ros2_migration_20260913_132122.md` |

旧目录通过同文件系统整体移动保留，包括 `.git`、未提交修改、未跟踪/忽略文件和全部生成物；没有 reset、clean 或递归删除旧项目。
源基线首次检出时 `git status --short` 为空。迁移不是 merge，不移回旧业务源码。
网络传输失败后使用 Git 内容寻址对象复用减少重复下载，随后补齐所有可达对象；缺失对象计数为 0，`git fsck --full` 通过，无 alternates 依赖。
源 remote 和 partial-clone 自动获取配置均已移除，以后不会从迁移来源自动 fetch/pull。
来源 URL 仅作为出处保留在文档/历史中，不是 remote。

旧远程分支创建后通过 GitHub API 和 `git ls-remote` 两次核验，再执行：

```bash
git push --force-with-lease=refs/heads/main:f2b0095a0da959b56e10140b292295b9ad9b8269 -u origin main
```

本轮报告、环境加载脚本和 Agent 状态另作普通提交，源基线提交及其历史不修改。
恢复旧历史应从备份分支创建新分支或单独工作树，不要在当前目录直接硬重置。

## 2. 分析方法与目录结构

逐文件清单、每个包的全部 console entry、launch、config 和直接依赖见
[完整清单](LOCAL_PROJECT_DIFF_INVENTORY.md)。清单按实际工作树内容计算，包含旧目录未提交修改，而非仅比较两个 HEAD。
采样时旧目录有 539 个受跟踪文件，新源基线 611 个：267 个新路径、195 个旧路径、123 个同路径变化、221 个同路径相同。
路径消失不直接意味着功能消失，例如网格从 description 迁到 moveit_config。

两边都有 `src/`、`tools/`、`tests/`、`patches/`、`docs/`、独立 Star Arm SDK 跟随目录。
NEW 新增根 README、LICENSE、THIRD_PARTY_NOTICES、AGENTS、CONTEXT、Agent 状态体系和分环境 requirements。
OLD 另有 `models/MANIFEST.sha256`、本机验收记录和架构 CI。

生成物不混入源码增删结论：OLD 的 `build/`、`install/`、`build-venv/`、`install-venv/`、`.venv-ros/`、`.ros-overlay/`、`log/`、缓存和模型下载目录仍在备份中。
NEW 的构建、虚拟环境、MotorBridge 编译和本地依赖均重新生成，未复制旧生成目录。
旧 Git 曾跟踪 `Log/OrbbecSDK.log.txt`，它虽出现在路径清单中，性质仍是运行日志，不是丢失的功能。
未跟踪或被忽略的旧本机资源只保留原状，不逐一宣称为源代码或重新部署。

## 3. ROS 2 包与完整功能范围

| 功能域 / 包 | NEW | OLD |
| --- | --- | --- |
| `rebotarm_msgs` | 共享 msg/srv/action，包括轨迹、夹爪、视觉和任务协议 | 同域；协议文件大部分复用，接口定义本身不证明 server 存在 |
| `rebotarmcontroller` | 六轴/夹爪连接、显式使能、反馈批次、轨迹执行/停止、安全回位、置零、新鲜度和故障保护、P0 验收工具 | 同域；拆出独立硬件状态、轨迹安全和 SDK 适配模块，另有 5 项本地修改 |
| `rebotarm_motion` | MoveIt 规划/位姿执行、预览、重定时、碰撞、对齐、运行约束、失败恢复、visual_ready | 同域，另有 replay_start_policy / teach_sample_processing；已删除 SDK 预览路径 |
| `rebotarm_teach` | JSONL 录制、同批反馈判定、文件处理、prepared trajectory、回放编排、dry-run/执行门 | 同域；已有独立录制所有权和回放，但反馈/工作流实现不同 |
| `rebotarm_teleop` | 键盘、Web 命令适配、夹爪显示桥 | 同域；主要接口继续存在 |
| `rebotarm_dashboard` | HTTP 页面、命令 API、SSE 状态、模型资源服务、teach workflow 调用 | 同域；面板参数及部分工作流封装不同 |
| `rebotarm_moveit_config` | 规范 URDF/mesh、SRDF、IK/OMPL/Pilz/控制器映射、RViz | 规划配置，物理描述另属 description |
| `rebotarm_description` | 不存在；描述能力并未删除，归 moveit_config | 独立 URDF/mesh 和共用电机配置所有者 |
| `rebotarm_calibration` | ArUco 位姿、手眼求解/残差 CLI、TCP 标定工具 | geometry/handeye 配置校验和 TCP；节点已归属 calibration |
| `rebotarm_vision` | Ubuntu 相机+YOLO、原生 ROS GraspNet、候选筛选/IK、视觉抓取状态机、标记/预览、离线检测、ordinary_grasp 适配 | Ubuntu 相机/独立 YOLO/GraspNet ROS 链、候选与抓取状态机；旧 Windows/HTTP 链已移除 |
| `rebotarm_simulation` | 唯一活动 MuJoCo ROS 后端、fake trajectory、模型生成、健康检查、Viewer、指标、虚拟相机和离线检测接线 | 同类基础能力，加 Real2Sim、Sim2Real、Reach/Pick、Gym/vector 环境及批处理/验收工具 |
| `rebotarm_bringup` | 场景组合、互斥后端选择、逐进程解释器配置 | 同域，以 core.launch.py 及分离运行/安全配置组织 |
| 独立 Star Arm 跟随 | SDK 跟随工具，部分 follower/hardware specs 变化 | 也存在；均非主 ROS 启动链，本轮不运行 |

当前 NEW 为 11 包：已退役旧 import/console 兼容包；`rebotarm_voice_control`
已于 2026-09-18 从当前工作树移除，语音控制不再属于当前 ROS 2 部署范围；
description 能力归 `rebotarm_moveit_config`。
两边 ROS 应用主体为 Python，msgs 为 ament_cmake/rosidl 生成接口；没有把生成的 C/C++ 消息代码当成手写算法差异。MotorBridge Rust/C ABI 属第三方构建。

## 4. 新项目有、旧项目没有

- 语音控制包及其文本/语音/Realtime/tool-call 代码、配置、launch 和测试已在后续清理中移除；本报告中仍出现的 voice 描述属于清理前历史快照。
- `Agent/` 状态刷新与活动日志、分层/资源测试、较大根测试集、许可/第三方来源记录。
- `camera_ubuntu.yaml` / `graspnet_ubuntu.yaml`、`vision_ubuntu.launch.py` 和显式解释器选择；不再保留远端网络输入。
- latest-wins 单槽队列和前置候选检查、离线 YOLO 节点、仿真虚拟相机及 `mujoco_offline_perception.launch.py`。
- 手眼求解、残差 CLI、ArUco 位姿工具；motion 自己拥有 visual_ready。
- 仿真 `mujoco_adapter_core`、runtime limit/metrics/profile、paired trajectory analysis 等模块。
- 新控制器中单 writer/中性释放、夹爪目标斜坡、反馈身份、零位验收等具体策略；是否适用本机仍需后续独立验收。

## 5. 旧项目有、当前新项目没有

- 独立 description 包和旧 `tools/check_architecture.py` / JSON 规则 / GitHub Actions 严格门禁。
- `core.launch.py`、`controller_runtime.yaml`、`controller_safety.yaml`、`rviz_real.yaml` 的旧统一启动/配置组织。
- Real2Sim ROS bridge/viewer、Sim2Real 随机化与回放比较、Reach/Pick/Gym/vector/batch 支线及多个验收 CLI。
- `graspnet_viewer_node.py` 独立 ROS 查看器和 `yolo_node.py` 独立检测节点。NEW 有其他点云查看/本机检测能力，不能将所有视觉查看能力判为消失。
- 模型 manifest、GraspNet 原生扩展补丁、旧 bootstrap/构建/模型/真实视觉/仿真验证工具。
- 旧 7 个测试文件与 2026-09-11 本机架构、视觉和回归证据，详见完整清单。

## 6. 两边都有但实现变化

基本功能仍是操作入口 -> motion/teach/vision -> ROS action/service -> 单一硬件或仿真后端。
区别集中在资源归属、配置、控制器内部组织、示教反馈协议、视觉进程划分和仿真扩展范围。
不要用同名文件或相同 API 推断行为等价；也不要把旧的拆分模块逐个覆盖回新版本。

## 7. Node / Launch 差异

NEW bringup 18 个 launch；增加 `interactive_basic.launch.py`、`mujoco_offline_perception.launch.py`，不再有 OLD 的 `core.launch.py`。
共同入口包括 driver_only、bringup、interactive_system、moveit_hardware、app、RViz real/sim、teach、teleop、视觉预览/抓取及真实感知+仿真执行。
NEW simulation 增加 `mujoco_moveit_sim.launch.py`，不提供 OLD 的 `real2sim_bridge.launch.py`。
NEW vision 增加 `vision_ubuntu.launch.py`，不提供 OLD 的 `graspnet_viewer.launch.py`。
清理前快照曾包含 voice_control/real/realtime/sim 四个入口；它们已不属于当前源码。
旧 `rebotarm_rviz_fake_controller` console 别名不在 NEW；`rebotarm_sim_trajectory_controller` 保留。

NEW 部分通用 app/driver 入口仍面向硬件；“视觉安全默认值”不是所有 launch 的总开关。
本轮只解析软件/仿真入口参数，不运行任何真机 launch。

## 8. Config、URDF / SRDF 与 MoveIt

- OLD description 的 arm/gripper YAML 改由 NEW bringup 持有；arm.yaml 检查主要是注释/归属变化，不能仅因路径变化假定电机数值全变。
- 未接入运行链的 driver 参数示例和回放预设配置已删除；启动参数由
  launch 显式管理，键盘、网页与示教分别使用 `keyboard_control.yaml`、`web_teleop.yaml`
  和 `teach_control.yaml`，公共关节定义使用 `operator_common.yaml`。
- 规范 URDF 从 OLD description 移至 NEW `moveit_config/config/rebotarm.urdf`，mesh URI 同步变化；网格仍为同源对象。
- URDF joint2/joint3 上界存在 `0 -> 0.02 rad` 等变化，不能自动应用旧机械限位假设。
- `joint_limits.yaml` 本次逐字节比较一致；SRDF 不一致：NEW 增加 `safe_home=[-pi/2,-0.1,-0.2,0.2,0,0]`，home 改为 `[0,-0.8,-1.2,0.6,0,0]`，移除 MoveIt all-zero 命名态。
- handeye.yaml 对比仅来源注释变化，数值并未因本次替换而变化。其他 grasp/table/gripper/retreat/retry 策略有改动，需逐项复核。
- 标定/限位/固件参数的源仓库现场记录不是本机硬件确认。本轮没有写入设备参数。

## 9. Controller 与安全

OLD 将硬件、SDK 适配、反馈/安全检查拆为较多模块；NEW 的 `hardware_manager.py` 集中承担更多生命周期、反馈批次和夹爪状态处理。
两边都隔离硬件访问、提供轨迹拒绝/停止等门控，不用代码行数判断哪边安全。
NEW 记录健康机械臂的可恢复失败应 stop+enabled hold 或确认回 baseline 后再 disable；严重通信/反馈/电机故障另作保护。
MotorBridge 新仓库要求固定 `0.4.6+rebotarm.2` 和 `feedback_sequence=true`，不能将 bootstrap `0.4.6` 当作合格真机 runtime。
审查补丁文件本次与 OLD 相同，但本轮重新构建验证，没有复用旧编译结果。

OLD 有 5 个未提交修改：rviz_real.yaml、core.launch.py、rviz_ee_drag_real.launch.py、rebotarm_controller.py、source_ubuntu_env.bash。
它们涉及 startup_enable、shutdown_safe_home 和全局 venv PYTHONPATH。原样保留备份，不移入 NEW。
尤其不恢复旧启动自动使能，也不把旧全局视觉依赖注入新多环境节点组。

## 10. Vision / Grasp

OLD 正式链是相机 -> 独立 YOLO -> 同步 GraspNet -> ROS 查看器；NEW 的 Ubuntu 原生 vision 节点整合相机/检测，GraspNet 独立 ROS 进程直接消费匹配的 RGB-D/CameraInfo/detection。
NEW 保留本机 GraspNet backend 和 ordinary_grasp 兼容入口；Windows/HTTP 脚本、bridge、网络客户端和查看器已删除。
NEW latest-wins 队列、候选前置质量/几何/workspace 门和时间策略与 OLD 实现不同；同名视觉抓取执行器和策略也有变化。
NEW 不再提供旧的 Windows 依赖 Open3D 查看器，候选查看使用 ROS 原生可视化节点。
旧模型权重、机器专属 engine、手眼现场记录不回填；新仓库自带 PT 仍在，默认 engine 缺失时应显式选 PT，而非伪装 engine。

## 11. Web / Teleop / Dashboard

两边均有 Web 状态、HTTP/SSE、键盘/Web 目标适配、夹爪控制和模型显示。
NEW Dashboard 的示教生命周期通过 teach 的 `TeachReplayWorkflow` 调用，保留 Web 权限；它仍在同一进程内，不是独立服务化。
OLD 有 panel_config 和此前面板/后端反馈契约修复，不能以 NEW 文件更多推断覆盖了 OLD 全部回归。
本轮没有发 HTTP 控制命令，不开启真实 Dashboard 控制链。

## 12. Teach / Replay

两边均把录制归 teach，不把控制器内置文件写入作为正式实现。
NEW `recording_feedback.py`、稳定批次 identity/header、同批状态新鲜度检查及工作流有显著变化。
OLD 有独立 `teach_sample_processing.py`、`replay_start_policy.py` 等组织方式；NEW 工作流不能直接用旧模块覆盖。
prepared trajectory、重定时、碰撞/对齐与运行跟踪能力两边都有。真实采样率、丢帧和回放效果未在本机验证。

## 13. Command Router / Task Manager / Safety

清理前快照中的 voice `DryRunCommandRouter`、`SafetyGuard`、`TaskPlanner` 和 sim executor 已随包删除。
当前没有独立通用 `task_manager.py` 或跨业务统一任务服务；不能把路由中 endpoint 名称当成所有真机 Action server 已实现。
OLD 当前树无 voice 包，任务消息定义与视觉/示教各自状态机仍在。
语音控制已退出当前范围；不要根据本报告中的历史描述恢复其入口或自动开放真机动作。

## 14. Simulation

NEW 聚焦 package-owned MuJoCo runtime、模型、fake 后端、steady-clock/clock、单一 trajectory server、虚拟相机及指标。
OLD 在相同基础上有较广 Real2Sim/Sim2Real/RL/task/batch 工具。NEW README 明确不宣称已有 RL/Gym 训练环境。
NEW 冻结仿真固件参考到 simulation 配置，不因硬件 YAML 改动而静默重调仿真。
本轮健康检查确认 8 joints / 8 actuators、模型加载、有限物理步进与 EGL renderer，不能据此声称抓取、训练或真机动力学验收成功。

## 15. Requirements / 第三方依赖

NEW 按 runtime、vision、GraspNet、MuJoCo、TensorRT 拆为根目录 5 份 requirements；OLD 根 runtime 加 vision 子包清单及 bootstrap/补丁工具。
NEW 明确 vision NumPy 1.x 与 cv_bridge ABI，GraspNet PyYAML 6.0.1 / MuJoCo 6.0.3，不能全混装。
厂商 SDK 均由新 `rebotarm_dependencies.repos` 固定到 `6a49302804f25e624995e771acb6d61896d1856d`。
旧多依赖 manifest、GraspNet 原生扩展补丁/模型校验能力与 NEW 的模型手动配置边界不同。
新主仓库只有个人 origin；SDK/MotorBridge 自身的独立第三方仓库 remote 是运行依赖，不是迁移来源 remote。

## 16. 本机环境与可复现命令

本机 Ubuntu 24.04.4、ROS 2 Jazzy、系统 Python 3.12.3；标准 `build/install/log` 从空目录新建。
系统已装主要 MoveIt 组件，未装 MoveIt 元包并不等于缺少所有 MoveIt。
sudo 需密码，未改系统权限。新下载 ROS Pinocchio/coal/eigenpy Debian 包解包至 `build_local_dependencies/root`，Pinocchio 4.1.0 导入通过；系统 dpkg/rosdep 仍不会把隔离解包视为 apt 已安装。
新建的 [环境脚本](../tools/source_local_environment.bash) 仅 source 系统 ROS/新 install 和本地 Pinocchio prefix，并设置逐进程解释器变量，不激活视觉环境、不启动节点。

```bash
# 构建时使用未激活 venv 的新终端
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --base-paths src --executor sequential --symlink-install

# 日常只加载环境，不启动硬件
source tools/source_local_environment.bash
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed

# 根目录软件测试，隔离当前系统的第三方 pytest 自动插件
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 \
  "$REBOTARM_MUJOCO_PYTHON" -m pytest tests -q -rs
```

测试必须用全新终端/本脚本，不能 source 备份里的 install。运行时视觉、GraspNet 和 MuJoCo 分进程使用各自解释器。
视觉 `.venv-vision`、GraspNet `.venv-graspnet` 和 MuJoCo `third_party/rebotarm_mujoco_venv` 均全新安装。
视觉确认 NumPy 1.26.4、OpenCV 4.11.0、PyTorch 2.11.0+cu128、TensorRT 10.13.3.9.post1、Orbbec/cv_bridge/YOLO 导入；GraspNet 依赖导入成功；两者均识别 RTX 4060 Laptop GPU/CUDA。
三个 venv 在清空 ROS PYTHONPATH 的环境中 `pip check` 均无冲突。不要在视觉 pip check 中混入 ROS install 路径后，把仅属 MuJoCo 进程的依赖误判为视觉必须混装。
MotorBridge 使用新建 Rust 1.98.1 toolchain、Cargo target 和新编译 wheel，安装为用户包 `0.4.6+rebotarm.2`，反馈契约检查通过。
官方 rustup 下载发生 DNS 故障后，使用 rsproxy.cn 镜像完成 toolchain；APT/ROS 包经本机已有 USTC 源下载。
SDK/MotorBridge 网络中断时仅复用了匹配新依赖清单固定提交的 Git 对象，干净检出后按 NEW 审查补丁重新构建，未搬入旧工作树改动或旧生成物。
本轮未下载/迁回 GraspNet 训练权重、PointNet2/KNN 构建结果或机器专属 engine。

## 17. 构建与测试结果

以下为最终实测结果。软件回归与硬件/模型推理验收分开记录。

| 检查 | 本轮结果 |
| --- | --- |
| 标准 workspace build | 13 packages finished，30.5 秒，成功 |
| 基线 Git 完整性 | 0 missing objects，fsck 成功，无旧目录 alternates |
| 包分层 + 资源 | 26 passed |
| Python compileall / 环境脚本 bash -n | 成功 |
| 系统 Python 根测试（最终，正常插件加载） | 811 passed, 7 skipped，7.50 秒；跳过 6 项 MuJoCo 和 1 项 torch |
| 新 MuJoCo 环境根测试（最终） | 817 passed, 1 skipped，14.16 秒；只跳过 torch 用例 |
| 视觉环境 GraspNet wrapper 文件 | 15 passed，1.02 秒，包含上述唯一 torch 用例 |
| 两环境去重覆盖 | XML 按 classname/name 合并：818 个不同用例通过，0 个未覆盖 skip，0 failures/errors；不是单一混装环境的 818 passed |
| MotorBridge | 新编译 motor_abi/ws_gateway/wheel 及隔离 smoke 成功；用户包安装/版本/feedback_sequence 检查成功，PTY 回归通过 |
| 视觉 / GraspNet / MuJoCo 依赖 | 各自安装、导入、pip check 成功；视觉和 GraspNet CUDA 可用 |
| MuJoCo health | ok=true，模型/步进/EGL 均通过 |
| 三个 launch 参数解析 | mujoco_sim、rviz_ee_drag_sim、vision_ubuntu 的 --show-args 均退出 0；未启动节点 |
| `colcon test` | 12 Python 包因 NO TESTS RAN 退出 5；根 tests 不在包内，不能称通过 |
| `colcon test-result` | 0 tests，不构成软件全绿证据 |
| rosdep check | ament_python key 未解析，系统 apt pinocchio 未安装；本地隔离 prefix 导入已验证 |

完整原始日志保留在新 `log/migration-*`，不会复制旧验收记录冒充本轮结果。
首次自动插件测试在缺失 MotorBridge 的 module-level importorskip 处整批提前退出；禁用自动插件可诊断/继续收集。补齐 patched MotorBridge 后，正常插件加载的系统环境根测试也通过。没有为此修改仓库测试。
阶段中间结果（15 failures、799/19、805/13）均已被本表最终结果替代，原始日志仍留作诊断。

## 18. Tests 差异与证据边界

NEW 根 tests 共 108 个受跟踪文件（含 conftest），OLD 7 个实际测试文件。
NEW 覆盖硬件 fake、MotorBridge PTY 协议、motion/teach/Web/vision/MuJoCo、资源/分层、安装/解释器与状态文件；voice 测试随包删除。
OLD 的严格架构 checker、自有 ROS perception/viewer、RViz execute workflow 与 functional fixes 回归未原样保留。
新测试数量多不等于每项旧回归均被覆盖；A 类迁移建议首先比对断言与契约，不批量复制测试。
本轮跳过所有真实机械臂/相机现场验收、实际轨迹/回放/抓取、标定写入和 motor enable。软件 fake/PTY/仿真结果仅证明软件路径。

## 19. 整体架构变化与 A/B/C 分类

### A. 值得以后重新迁移

1. 旧严格架构 CI/checker 的 SDK、资源依赖环、唯一反馈/服务所有者规则，先与新分层测试合并设计。
2. 独立 ROS GraspNet/Open3D 查看器及相关同帧、错误/缺失输入回归；需适配新视觉数据契约。
3. 模型 manifest、扩展可复现构建及校验工具，避免依赖本机偶然存在的 checkpoint。
4. Real2Sim 只读镜像、Sim2Real/RL/Reach/Pick/batch，仅在用户确立研究范围后重新迁移，不能扰动新唯一仿真后端。
5. OLD 的关闭/停止/录制/后端反馈等回归契约，逐项映射新实现后补测。

### B. 已被新项目替代

1. description 旧资源路径已由 moveit_config 规范模型替代，不恢复重复 URDF/mesh。
2. 旧整体环境复制、全局视觉 PYTHONPATH 与 install-venv 启动方式，由系统构建+逐进程解释器取代。
3. 旧零散启动脚本/配置入口应按新 bringup 入口重新接线，不原样覆盖 core。
4. 旧录制所有权/部分 Web 编排组织已有新 teach workflow 和反馈批次实现，不恢复控制器录制副本。
5. 原本地文档的现场“完成”状态只供追溯，不取代新基线本机验证。

### C. 需要人工判断

1. description 独立建模包 vs MoveIt 包持有模型，两者是架构选择，不能简单判定哪边退步。
2. 两套视觉进程划分、网络兼容是否保留、Open3D 查看器部署方式。
3. 控制器模块化程度、SDK 预览路径、夹爪 neutral/hold/零位规则与现场适配。
4. URDF joint2/joint3 余量、SRDF named states、任务 workspace/TCP/夹爪策略；本轮不修改真实参数。
5. 新语音/任务层是否纳入长期产品范围；旧项目曾删除它，新基线保留它，不自动据此开放真机动作。
6. OLD 5 项未提交变更，特别是 startup_enable/shutdown_safe_home，只做评审，不自动恢复。

## 20. 问题与下一阶段建议

2026-09-13 网络兼容链收口：用户确认删除旧跨主机 HTTP/JSON/MJPEG 路线。已移除
网络相机、检测/GraspNet 客户端、HTTP GraspNet 回退服务及网络 camera.yaml；
GraspNet 节点只消费 ROS RGB-D/CameraInfo/detection，主 launch 默认 ubuntu_native。
本机推理 backend 保留；Dashboard HTTP、ROS DDS 和在线语音服务不在删除范围。
vision/bringup 构建及视觉重点测试通过；没有启动相机或真机。

2026-09-13 后续运行验证补充：用户启动 MuJoCo + MoveIt 后，Plan 成功但
Execute 失败。启动日志确认 `MoveItSimpleControllerManager` 插件类不存在，
当时系统未安装 `ros-jazzy-moveit-simple-controller-manager`，而 MuJoCo 的
trajectory Action server 已存在且为 1。此前 build/pytest/参数解析通过不代表
该执行链通过；此项是迁移时遗漏的运行依赖。本次仅补充
[安装与排查说明](local_setup_zh.md#moveit-执行插件能-plan-但不能-execute)，
未安装依赖、未执行轨迹，不将问题标为修复复测通过。manifest 依赖声明仍待补齐。

2026-09-13 反馈启动误报修复：SDK 只读样本证明六个电机的 sequence 独立推进、
状态码均为 0，但初始强制刷新窗口可能在分批反馈尚未收齐时记录
`verified feedback pending`。`hardware_manager.py` 将反馈重试间隔由 5 ms 调整为
50 ms，并加入分批反馈回归测试；不要求六个 sequence 数值相等，也不改变失能/使能
安全门。控制器安全测试 124 项、分层/资源/启动检查 34 项、完整软件回归 812 项
通过；未连接真机，需现场重新启动 `driver_only` 验证实际日志。

依赖和软件回归已完成本轮收口；下一阶段首先建立最小 CI（根 pytest、分层/资源、compileall、独立 build）。
处理 `colcon test` 与根测试集的集成，但应在新任务中独立修改，不把本次基线替换变成隐式重构。
随后评审 A 类中架构门禁、模型可复现和只读查看器，明确是否需要 Real2Sim/RL/voice 支线。
真机阶段必须另行授权，从配置/设备身份与只读反馈核验开始，不用本报告软件结果替代现场安全确认。
新 GraspNet 环境安装不等于已配置推理模型、编译 PointNet2/KNN 或通过实际推理；缺少模型/engine 的部分需另行准备和审查。
