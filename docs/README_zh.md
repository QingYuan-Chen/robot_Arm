# reBotArm 文档索引

本文档用于区分当前有效说明、目标规划和历史实施记录，避免把计划中的能力误认为已经实现。

## 当前依据

- Agent 实时状态、完成度和阻塞：[`../Agent/README.md`](../Agent/README.md)
- 已关闭阶段的验收清单与依据：[`../Agent/PROJECT_STATUS.md`](../Agent/PROJECT_STATUS.md)
- package ownership / 包职责和依赖边界：[`architecture.md`](architecture.md)
- 项目术语：[`../CONTEXT.md`](../CONTEXT.md)
- coding agent / 编码代理规则：[`../AGENTS.md`](../AGENTS.md)

如文档之间发生冲突：

1. 安全约束以 `AGENTS.md`、`architecture.md` 和当前代码事实为准；
2. 新任务优先级和范围以用户最新明确要求为准；已完成阶段以 `Agent/PROJECT_STATUS.md` 为准；
3. 操作命令必须同时满足当前代码参数和文档中的阶段状态；
4. 历史设计文档与 Git 中保留的旧规划仅供追溯，不覆盖当前状态。

项目实施完成度不在本文手工维护，读取 [`../Agent/STATE.json`](../Agent/STATE.json)。

## 当前状态说明

P0-P6 已按用户确认的工程范围关闭，等待用户提供新规划，不自行创建 P7。
根目录两份旧规划已清理，历史内容保留在 Git 中。各阶段验收依据、豁免和
未实测边界见 `Agent/PROJECT_STATUS.md`，后续维护事实见 `Agent/MEMORY.md`。

## 架构与环境

- [`architecture.md`](architecture.md)：ROS 2 package ownership、依赖方向和执行权限。
- [`node_topology.mermaid`](node_topology.mermaid)：当前与目标后端选择拓扑。
- [`local_setup_zh.md`](local_setup_zh.md)：Ubuntu 24.04 / ROS 2 Jazzy 本机环境、构建和安全顺序。
- [`ubuntu_vision_setup_zh.md`](ubuntu_vision_setup_zh.md)：Gemini 2 + YOLO 的 Ubuntu 原生环境和当前验收缺口。

## MuJoCo

- [`mujoco_sim.md`](mujoco_sim.md)：现有 MuJoCo 模型、adapter、benchmark 和启动方式。
- [`mujoco_upstream_sources.md`](mujoco_upstream_sources.md)：固定上游版本、许可证证据和禁止复制边界。
- [`mujoco_sim_to_real_params.md`](mujoco_sim_to_real_params.md)：仿真参数到真机参数的映射边界。

## 操作手册

- [`rebotarm_common_commands.md`](rebotarm_common_commands.md)：网页、键盘、示教和状态检查命令。
- [`rebotarm_feature_commands.md`](rebotarm_feature_commands.md)：RViz MoveIt 和网页遥操作入口。
- [`visual_grasp_commands.md`](visual_grasp_commands.md)：视觉抓取的当前过渡链路、安全预览和阶段门。
- [`voice_control_stage4_zh.md`](voice_control_stage4_zh.md)：语音控制阶段 4 说明。

所有涉及真机的命令都必须先阅读对应文档顶部的安全状态说明。

## 视觉抓取设计

- [`visual_grasp_seven_layer_params.md`](visual_grasp_seven_layer_params.md)：视觉候选、IK、workspace、夹爪和执行参数分层。

参数设计文档描述结构和约束，不代表其中所有默认值都已经通过真机标定。

## 历史计划与规格

历史 implementation plan / 实施计划、design spec / 设计规格和阶段实验报告仅本机留存，不随当前源码发布。过去已提交的版本可从 Git 历史追溯，不作为当前运行手册。
