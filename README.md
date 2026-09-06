# reBotArm ROS 2

面向 reBot B601 的 ROS 2 Jazzy 工作空间，包含机械臂与夹爪控制、MoveIt 规划、
网页遥操作、示教、RGB-D 视觉抓取和 MuJoCo 仿真。

## 从这里开始

| 内容 | 入口 |
| --- | --- |
| 环境、依赖与 MotorBridge 安装 | [本机环境说明](docs/local_setup_zh.md) |
| 网页遥操作、RViz 与实机操作 | [功能操作手册](docs/rebotarm_feature_commands.md) |
| 控制器接口 | [ROS SDK 说明](README_zh.md) |
| 仿真 | [MuJoCo 使用说明](docs/mujoco_sim.md) |
| 包职责与依赖边界 | [架构说明](docs/architecture.md) |
| 当前阶段及验收依据 | [Agent 状态](Agent/README.md) |
| 其他文档 | [文档索引](docs/README_zh.md) |

## 工程结构

- `src/`：13 个分层 ROS 2 包，硬件、规划、示教、交互、视觉和仿真分别归属。
- `tools/`：环境安装、验证和离线工具。
- `tests/`：软件回归与架构检查。
- `patches/`：固定版本的 MotorBridge 反馈与校零安全补丁。
- `Agent/`：实施状态、决策和验收记录；历史完成度不等于新的真机动作授权。
- `third_party/`：仅本机参考快照、SDK 和虚拟环境；不纳入版本管理。
- `star_arm_102_rebot_b601_follow/`：独立 Star Arm 跟随工具，按其自身说明使用。

标准本机构建目录为 `build/`、`install/`、`log/`，不提交到 Git。
模型下载、TensorRT engine、本地运行数据及额外实验原始证据也不自动纳入提交。
已有受版本管理的模型和上游参考资产保留，来源见 [第三方说明](THIRD_PARTY_NOTICES.md)。

## 环境与构建

基线为 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。先按环境说明安装所需组件；
五份 `requirements-*.txt` 按控制器、视觉、GraspNet、MuJoCo、TensorRT 拆分，
不要合并安装到同一个环境。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --executor sequential
source install/setup.bash
python3 -m pytest tests -q
```

完整视觉包构建还要求本机提供 `tools/yolo26m-seg-fp16-b1-640-linux.engine`；
该机器相关产物不在 Git 中，相关说明见 [部署与解释器配置](docs/launch_python_configuration.md)。
纯源码克隆不包含所有本机模型、SDK、虚拟环境和实验数据，不能据此假定全部功能可直接运行。

## 真机边界

连接不等于使能；运动必须明确授权并显式使能。串口只允许一个控制器占用。
主控制器要求 `motorbridge 0.4.6+rebotarm.2` 及逐电机反馈序号能力，启动前运行：

```bash
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
```

可恢复的任务失败应停止运动并保持使能，或受控回到已验证基线后再失能；
不要用批量结束进程代替安全停机。软件测试通过不代表本轮完成了硬件验收。

## 许可

项目许可见 [LICENSE](LICENSE)，第三方资产分别遵守其来源与许可记录。
