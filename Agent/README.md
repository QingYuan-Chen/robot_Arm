# reBotArm Agent 工作区

`Agent/` 是仓库内所有 coding agent / 编码代理共享的持久化工作区，用于保存当前执行状态，而不是保存大段聊天记录。

## 文件职责

| 文件 | 职责 | 更新方式 |
| --- | --- | --- |
| `PROJECT_STATUS.md` | 主规划 P0-P6 的验收清单和完成依据 | Agent 在证据成立后更新 checkbox |
| `EXECUTION_FLOW.md` | 每次任务的开始、执行、验证、交接流程 | 流程变化时人工更新 |
| `MEMORY.md` | 当前事实、决策、阻塞和交接信息 | 每次状态变化立即更新 |
| `ACTIVITY_LOG.md` | 按时间追加的工作事件 | 由 `update_state.py` 追加 |
| `STATE.json` | 机器可读的实时快照和加权完成度 | 只由 `update_state.py` 生成 |
| `update_state.py` | 重新计算完成度、Git 状态和最新事件 | 任务开始、检查点和结束时运行 |

根目录 `codex_memory.md`、`项目规划.md` 和 `INTERACTIVE_SYSTEM_STATUS_ZH.md` 是历史资料，不再作为实时状态来源。

## 状态来源优先级

1. 当前代码、测试和硬件验收证据；
2. `AGENTS.md` 与 `docs/architecture.md` 的安全和包边界；
3. `新项目规划.md` 的项目优先级；
4. `Agent/PROJECT_STATUS.md`、`Agent/MEMORY.md` 和 `Agent/STATE.json` 的当前执行状态；
5. 历史文档只用于追溯。

## “实时更新”的含义

本仓库不运行后台常驻 Agent。实时更新指以下事件发生后，在同一个任务内立即同步：

- 开始一个新任务；
- 当前阶段、执行队列或阻塞发生变化；
- 完成一项可验证的验收条件；
- 测试结果、硬件边界或关键事实发生变化；
- 任务完成或需要交接。

## 快速使用

读取当前状态：

```bash
python3 Agent/update_state.py
cat Agent/STATE.json
```

记录任务开始：

```bash
python3 Agent/update_state.py \
  --event start \
  --actor Codex \
  --note "开始处理 P0 连接与使能解耦"
```

记录验证检查点：

```bash
python3 Agent/update_state.py \
  --event verified \
  --actor Codex \
  --note "完成 enable 失败回滚测试" \
  --verification "python3 -m pytest tests/test_hardware_enable.py -q"
```

更新 checkbox 后重新计算完成度：

```bash
python3 Agent/update_state.py \
  --event checkpoint \
  --actor Codex \
  --note "P0 验收清单已更新"
```

## 约束

- 真机处于健康 enabled 状态时，视觉、规划、标定采集等可恢复的任务级失败不得直接 disable：必须 stop 后保持 enabled hold，或受控回到本轮已记录 baseline、确认到位后再 disable。受控回位失败但电机/控制器仍健康时继续保持并等待人工处置；只有 holding torque 已不可信的硬件/通信严重故障或明确急停请求才允许在非 baseline 姿态保护性失能；
- 没有测试、运行记录或明确代码证据时，不得把 checkbox 改为 `[x]`；
- 自动测试通过不等于真机验收通过；
- 只完成部分工作时，在 `MEMORY.md` 写清剩余项，不把整项标成完成；
- `STATE.json` 是生成物，不手工修改；
- `ACTIVITY_LOG.md` 只追加，不回写历史；
- 不在这些文件中保存密码、token、设备私密凭据或用户隐私数据。
