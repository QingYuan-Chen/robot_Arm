# reBotArm 文档索引

本文档用于区分当前有效说明、目标规划和历史实施记录，避免把计划中的能力误认为已经实现。

## 当前依据

- Agent 实时状态、完成度和阻塞：[`../Agent/README.md`](../Agent/README.md)
- 总体路线、阶段优先级和验收门：[`../新项目规划.md`](../新项目规划.md)
- package ownership / 包职责和依赖边界：[`architecture.md`](architecture.md)
- 项目术语：[`../CONTEXT.md`](../CONTEXT.md)
- coding agent / 编码代理规则：[`../AGENTS.md`](../AGENTS.md)

如文档之间发生冲突：

1. 安全约束以 `AGENTS.md`、`architecture.md` 和当前代码事实为准；
2. 项目优先级以 `新项目规划.md` 为准；
3. 操作命令必须同时满足当前代码参数和文档中的阶段状态；
4. `docs/superpowers/` 和根目录旧 `项目规划.md` 仅作为历史证据，不覆盖当前主规划。

项目实施完成度不在本文手工维护，读取 [`../Agent/STATE.json`](../Agent/STATE.json)。

## 当前状态说明

截至 2026-08-06：

- MuJoCo 基础闭环已经存在，后续采用固定上游版本的 gap analysis / 差距分析，不整体覆盖当前实现；
- Gemini 2 + YOLO 的 Ubuntu 原生独立入口已经存在，但真实 RGB-D、SDK 内参和深度尺度仍待硬件验收；
- 完整视觉抓取入口尚未完成 `ubuntu_native` profile 集成；
- Ubuntu 本地 GraspNet 服务尚未完成；
- P0 软件安全门和分级真机验收已完成：`HardwareManager.connect()` 只连接并保持失能，显式 enable、当前位置 hold、disable 和失败 cleanup 均有证据；完整视觉抓取仍需通过 P2-P6 后续门。

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

`superpowers/plans/` 和 `superpowers/specs/` 保存过去功能的 implementation plan / 实施计划与 design spec / 设计规格。它们用于追溯设计理由，不作为当前运行手册。
