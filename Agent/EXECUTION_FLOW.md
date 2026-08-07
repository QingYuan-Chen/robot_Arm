# Agent 更新与执行流程

## 当前执行队列

1. [已完成] P1：固定 `huangbinai/robotarm_ros2` MuJoCo 快照并完成目标文件与 license evidence 审计；证据：`Agent/evidence/P1/2026-08-06-robotarm-ros2-preflight.md`。
2. [已完成] P1：独立运行上游 health/headless/model/ROS/motor/collision 测试，形成双基线差异报告；报告已形成，上游重放断言保留为 `test_defect`，不宣称上游全绿。
3. [进行中] P1：选择性本地化上游模型、URDF-to-MJCF、motor control、health check 和 viewer 能力；health check 已本地化，模型/转换器已完成接口审查，隔离 runner、快照和同一 command contract 对比已完成，默认模型暂不替换，motor/viewer 仍待逐项审查。
4. P1：将 timeout 接入执行循环；
5. P1：补 execute-loop 成功、取消、停止、容差失败和超时集成测试；
6. P1：补齐 effort、velocity、acceleration、jerk 限制一致性；
7. P1：完成 joint4-6 跟踪、collision、夹爪接触和抓取质量标定。

## 每次任务开始

1. 阅读 `AGENTS.md`、`docs/architecture.md` 和 `CONTEXT.md`；
2. 阅读 `Agent/MEMORY.md`、`Agent/PROJECT_STATUS.md` 和 `Agent/STATE.json`；
3. 检查 Git 分支、工作区已有修改和当前阻塞；
4. 确认任务属于哪个 P 阶段和 package owner；
5. 运行 `update_state.py --event start` 记录任务目的；
6. 只在用户授权范围内执行，不因“当前阶段”自行扩大任务范围。

## 执行中

1. 先验证问题前提，不把规划描述直接当作当前代码事实；
2. 将实现拆到正确 package，保持仿真和实机后端互斥；
3. 每完成一个独立检查点，运行相关最小测试；
4. 发现新事实、决策或阻塞时立即更新 `MEMORY.md`；
5. 只有验收证据成立时更新 `PROJECT_STATUS.md` checkbox；
6. 阶段或完成度变化后运行 `update_state.py --event checkpoint`。

## 阻塞处理

阻塞出现时：

1. 在 `MEMORY.md` 的“当前阻塞”记录事实、影响和解除条件；
2. 记录已尝试的只读检查和替代路径；
3. 不用猜测补齐硬件数据、上游 commit 或用户决策；
4. 运行 `update_state.py --event blocked`；
5. 向用户报告需要的最小输入或外部状态变化。

## 完成与交接

1. 运行与风险相称的 unit、integration、layering 和 compile 检查；
2. 更新 `PROJECT_STATUS.md` 中有证据支持的项目；
3. 更新 `MEMORY.md` 的当前事实、最近验证、阻塞和下一步；
4. 运行 `update_state.py --event verified` 或 `--event complete`；
5. 检查 `STATE.json` 的完成度、分支和工作区状态；
6. 最终回复说明完成内容、验证结果、未验证边界和下一步。

## P0-P6 阶段门

```text
P0 实机安全
→ P1 MuJoCo 基线巩固
→ P2 Gemini 2 RGB-D 验收
→ P3 单 Ubuntu 完整视觉
→ P4 Ubuntu 本地 GraspNet
→ P5 hand-eye/TCP/MuJoCo 标定
→ P6 仿真与实机分级系统验收
```

P0 已通过。P1 纯仿真工作可以开始，但 P0 通过不自动授权完整实机视觉执行；后续阶段的局部软件开发可以提前进行，不能越过 P2-P6 对应验收门宣称系统完成。
