# reBotArm MuJoCo 仿真落地说明

本文记录本工程的 MuJoCo 仿真落地方式。MuJoCo 在本仓库中定位为
物理和离线验证层，用于接触、抓取、轨迹跟踪和策略迭代；它不直接拥有
真机通信，也不替代 `rebotarmcontroller` 的硬件安全边界。

## 目录和边界

当前使用同型号参考仓库作为原始资产来源：

```text
third_party/reBotArm_develop_hjx
```

该目录被 `.gitignore` 忽略，不作为主工程源码直接维护。主工程维护的是：

```text
src/rebotarm_simulation/rebotarm_simulation/mujoco_model_profile.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_runner.py
src/rebotarm_simulation/rebotarm_simulation/mujoco_cli.py
tests/test_mujoco_model_profile.py
tests/test_mujoco_headless.py
requirements-mujoco.txt
```

`rebotarm_simulation` 只负责离线仿真、模型生成、headless 验证和后续 ROS
仿真适配。真机 SDK、串口、电机最后安全检查仍属于 `rebotarmcontroller`。

## 环境

ROS 2 Jazzy 节点继续使用系统 Python。MuJoCo 使用独立虚拟环境，避免和 ROS
的 `rclpy`、Qt、OpenCV、OpenSSL、`libstdc++` 发生依赖冲突。

当前本机已有环境：

```bash
third_party/rebotarm_mujoco_venv/bin/python -m pip check
```

新机器可按下面方式创建：

```bash
python3 -m venv --system-site-packages third_party/rebotarm_mujoco_venv
third_party/rebotarm_mujoco_venv/bin/python -m pip install -r requirements-mujoco.txt
```

当前固定的核心依赖：

```text
mujoco==3.3.0
numpy
pyyaml
jinja2
typeguard
```

`pyyaml`、`jinja2`、`typeguard` 用于让当前带 ROS 可见包的 MuJoCo venv
通过 `pip check`。`mink`、QP solver、OpenCV 手势控制等属于后续可选能力，
第一阶段不纳入必需依赖。

## 生成物理版模型

不要直接修改参考仓库的 MJCF。使用主工程生成器从参考 XML 派生物理版 XML：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli generate \
  --robot-output build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --scene-output build/mujoco_models/sim_reBot_grasp_physics.xml
```

生成结果位于 `build/mujoco_models/`，属于可重复生成文件，不需要提交。

生成器会做这些事情：

- 将原 STL mesh geom 标记为 visual-only：
  - `contype="0"`
  - `conaffinity="0"`
  - `group="1"`
- 为主要 link 和夹爪指尖新增简化 collision geom：
  - arm link 使用 box/cylinder；
  - finger pad 使用 box；
  - collision geom 使用 `group="4"`。
- 修正 actuator 名称、控制范围和保守力矩：
  - `joint1..joint3` 使用 `forcerange="-27 27"`；
  - `joint4..joint6` 使用 `forcerange="-7 7"`；
  - `left_finger` 使用 `ctrlrange="0 0.045"`、`forcerange="-20 20"`、
    `kp="600"` 和 `kv="60"`。
- 生成两个机器人 XML：
  - 独立机器人 XML 带 `zero/home` keyframe；
  - 场景 include 版机器人 XML 不带 keyframe，避免被抓取场景的 15 维 qpos
    冲突。

## 验证命令

纯 XML 转换测试：

```bash
python3 -m pytest tests/test_mujoco_model_profile.py -q
```

机器人 headless smoke：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli smoke \
  --xml build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --seconds 1.0
```

单关节阶跃响应：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli step-response \
  --xml build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --joint joint2 \
  --target -0.6 \
  --seconds 3.0
```

基础抓取场景稳定性：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli grasp-benchmark \
  --xml build/mujoco_models/sim_reBot_grasp_physics.xml \
  --seconds 3.0
```

如果当前 Python 环境安装了 MuJoCo，也可以直接运行：

```bash
python3 -m pytest tests/test_mujoco_headless.py -q
```

否则该测试会自动 skip。

## 当前参数策略

第一阶段使用保守参数，不追求最快响应：

```text
joint1: ctrlrange -2.8 2.8,   forcerange -27 27, kp 270
joint2: ctrlrange -3.14 0,    forcerange -27 27, kp 270
joint3: ctrlrange -3.14 0,    forcerange -27 27, kp 270
joint4: ctrlrange -1.87 1.57, forcerange -7 7,   kp 70
joint5: ctrlrange -1.57 1.57, forcerange -7 7,   kp 70
joint6: ctrlrange -3.14 3.14, forcerange -7 7,   kp 70
gripper: ctrlrange 0 0.045,   forcerange -20 20, kp 600
```

调参顺序：

1. 保持 collision 简化模型稳定。
2. 单关节阶跃响应调 `kp/kv/damping/frictionloss`。
3. 多关节轨迹跟踪看 RMS error、最大速度和最大 actuator force。
4. 夹爪抓取单独调 finger pad 的摩擦和接触参数。

## ROS 2 adapter 状态

当前已经在 `rebotarm_simulation` 内增加 ROS 2 MuJoCo adapter：

- 发布 `/rebotarm/joint_states`；
- 接收 `/rebotarm/follow_joint_trajectory`；
- 接收 `/rebotarm/gripper/set`；
- 保留 `/rebotarm/trajectory_stop`；
- 将 MoveIt 规划轨迹送入 MuJoCo 真实 `mj_step()` 跟踪。

adapter 仍然只属于仿真包，不直接调用硬件 SDK。

## ROS 2 / MoveIt 接入

当前已经新增独立 MuJoCo ROS adapter 节点：

```text
rebotarm_mujoco_adapter
```

它提供：

```text
/rebotarm/follow_joint_trajectory
/rebotarm/joint_states
/rebotarm/trajectory_stop
/rebotarm/gripper/set
/rebotarm/gripper/state
```

adapter 使用 MuJoCo `data.time` 作为轨迹插值和完成判断的时间基准。ROS timer
只提供 wall-clock 调度，实际每次会根据 MuJoCo `timestep` 和累计 wall delta
补足需要执行的 `mj_step()` 次数，避免模型 `timestep=0.0025` 但 ROS timer
默认 200Hz 时仿真只以 0.5 倍实时速度运行。

`FollowJointTrajectory` 的 stopped / canceled / tolerance 类 stop reason 会映射为
非 `SUCCESSFUL` 结果；`summary.json` 仍记录 stop reason 和误差统计，后续
goal/path tolerance 阈值会基于 step-response 数据继续标定。

当前还支持：

- 单点 delayed trajectory 自动在 `t=0` 插入当前关节状态，避免目标瞬间跳到终点；
- stale generated XML asset 检查，已有 XML 指向不存在 mesh/texture 时会自动重生成；
- `step-response-suite` 批量标定入口，输出每个关节的最终误差、RMS、速度和 actuator force；
- `metrics_sample_stride` 参数，用于长序列下采样写出 CSV；
- `use_mujoco_viewer` 参数，默认 `false`，需要观察 MuJoCo 原生界面时再打开；
- grasp benchmark 会输出 contact/lift/success/status，用于后续抓取质量闭环。

启动前需要 source ROS 和本工作区：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果直接用 Python 模块运行 adapter，本地源码路径要追加到 ROS 的
`PYTHONPATH`，不能覆盖它：

```bash
PYTHONPATH=src/rebotarm_simulation:$PYTHONPATH \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_ros_adapter_node
```

推荐使用 launch：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py \
  use_rviz:=true \
  python_executable:=third_party/rebotarm_mujoco_venv/bin/python
```

无 RViz 验证：

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py \
  use_rviz:=false \
  python_executable:=third_party/rebotarm_mujoco_venv/bin/python
```

直接发送一条轨迹：

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.0, -0.6, -1.0, 0.4, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}"
```

执行后会生成：

```text
build/mujoco_runs/latest/trajectory_metrics.csv
build/mujoco_runs/latest/summary.json
```

夹爪服务：

```bash
ros2 service call /rebotarm/gripper/set \
  rebotarm_msgs/srv/SetGripper \
  "{position: 0.03, max_effort: 1.0}"
```

查看夹爪状态：

```bash
ros2 topic echo /rebotarm/gripper/state --once
```

批量 step-response 标定：

```bash
PYTHONPATH=src/rebotarm_simulation \
third_party/rebotarm_mujoco_venv/bin/python \
  -m rebotarm_simulation.mujoco_cli step-response-suite \
  --xml build/mujoco_models/reBot-DevArm_gripper_physics.xml \
  --seconds 2.0 \
  --json-output build/mujoco_runs/latest/step_response_suite.json
```

注意：当前批量标定已经暴露出 joint4 / joint6 在部分较大阶跃目标下最终误差较大。
后续如果要继续调 `forcerange`、`kp`、`kv`、`damping`，必须先决定约束来源：

- 按真实电机/减速器/电流能力保持保守参数；
- 或为了仿真演示提高力矩和增益。

这两条路线会得到不同的仿真可信度，不能混为一谈。
