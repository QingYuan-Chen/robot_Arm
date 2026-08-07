# MuJoCo 上游来源与授权边界

> 核验日期：2026-08-06。所有上游输入必须固定到 commit；README 自述不能替代实际许可证文件。

## 已核验或候选来源

### robotarm_ros2 MuJoCo migration candidate

- Repository：`https://github.com/huangbinai/robotarm_ros2.git`
- Branch：`main`
- Commit：`fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`（2026-07-13，`fix: tune MuJoCo gripper force and viewer jog`）
- Local snapshot：用户已提供的 `/home/a/project/rebot_refer`，remote 与 commit 已核对且工作区 clean；当前 Git 的 `third_party/robotarm_ros2_mujoco_snapshot/` 为字节保持快照，已完整本地化到 `src/rebotarm_simulation/` 作为默认 ROS 仿真 backend。
- Source swap：切换前的 package-owned 实现以字节保持方式归档到 `third_party/rebotarm_simulation_current_baseline/`，由 `ARCHIVE_MANIFEST.json` 校验每个文件的 size/SHA-256；它只作为显式 `simulation_backend:=current` fallback 和回滚对照，不参与默认 colcon source。
- Candidate files：`src/rebotarm_simulation/`、`models/rebotarm/`、`config/`、`launch/`、MuJoCo tests and runbooks。
- Observed capabilities：health/headless CLI、native viewer、ROS 2 adapter、URDF-to-MJCF generator、motor control、gravity compensation、collision/contact acceptance and model contract tests。
- License evidence：`src/rebotarm_simulation/package.xml` declares `Apache-2.0`，但固定仓库根目录未发现可直接核验的 `LICENSE` 文件；license status 仍记为 `PROVISIONAL`。允许当前工作区本地运行，授权确认前不对外再分发快照。
- Decision：上游 package 直接作为 P1 默认 MuJoCo backend 使用；当前 package-owned 实现保留在归档与 active source 的 compatibility modules 中，并通过 `simulation_backend:=current` 显式 fallback，两个 backend 通过 launch 条件互斥。

### MuJoCo physics engine

- Repository：`https://github.com/google-deepmind/mujoco.git`
- Release/tag：`3.3.0`
- Commit：`8de9b4e4e373caf01035c6c13efd363347d89032`
- License：Apache-2.0
- Evidence：该固定 commit 存在完整 `LICENSE`，Python package metadata 也声明 Apache License 2.0。
- Allowed use：Python runtime/API、官方文档描述的 URDF/MJCF compiler 行为和 API。

### 本仓库模型与 meshes

- Repository：`https://github.com/QingYuan-Chen/rebot_Arm.git`
- Baseline commit：`5bd5510`，后续修改由本仓库 Git history 追踪。
- License：根目录 `LICENSE` 为 Apache-2.0；`rebotarm_bringup`、`rebotarm_moveit_config` 和 `rebotarm_simulation` 的 package metadata 均声明 Apache-2.0。
- Inputs：`rebotarm_moveit_config/config/rebotarm.urdf` 与 `rebotarm_bringup/description/meshes/`。
- Generated model：`rebotarm_simulation/assets/rebotarm_base.xml`，由本仓库 URDF 使用 MuJoCo 3.3.0 编译后整理。

### 官方机器人资料参考

- Repository：`https://github.com/Seeed-Projects/reBot-DevArm.git`
- Branch：`main`
- Commit：`c3a75bd83305c7fdc4d63a7f21ba9e12166c1092`
- Root license：CERN-OHL-W-2.0，用于硬件 covered source；README 对 software SDK 另行声明 Apache-2.0。
- Policy：只在明确识别文件所属硬件/软件许可证后使用；当前默认 MJCF 不复制该仓库文件。

## 拒绝作为代码或资产来源

### HJX-exoskeleton/reBotArm_develop_hjx

- Repository：`https://github.com/HJX-exoskeleton/reBotArm_develop_hjx.git`
- Branch：`master`
- Commit：`bcd584ce4a64eb116d80f6873c1756eddb8520bf`
- Observed claim：README badge 和文字声称 MIT。
- Verified tree：没有 `LICENSE`、`COPYING` 或 `NOTICE` 文件；README 的 `./LICENSE` 链接无对应文件。
- Decision：license status 记为 `NOASSERTION`。不得复制、修改、分发或在默认运行路径加载其源码、XML、meshes、textures；只允许记录可独立观察的行为和接口，用于编写 current/upstream capability gap，不逐行翻译实现。

## 自动边界

- `mujoco_model_profile.py` 的默认 XML 必须位于 `rebotarm_simulation/assets`。
- 默认模型不得包含 `reBotArm_develop_hjx` 路径。
- build 生成物的 mesh 路径必须解析到 `rebotarm_bringup/description/meshes`。
- 引入新上游文件前必须补充 URL、branch/tag、commit、license file 和目标文件清单。
