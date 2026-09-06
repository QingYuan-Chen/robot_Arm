# Star Arm 102-LD → reBot B601 Python SDK 工具

该工具通过 Python SDK 同时读取 Star Arm 102-LD 引导臂和 reBot B601-DM 从臂，
并以从臂关节坐标为基准检查 J1～J6 的方向映射。

`snapshot`、`calibrate-directions` 和 `rviz-preview` 三个命令严格只读：不会使能、失能、
置零、切换模式或发送位置、速度、力矩、轨迹指令。

`follow` 是独立的真机运动命令。默认不带确认参数时只做静态预检；只有显式传入
`--confirm-live-motion` 才会使能从臂并执行 J1～J6 实时跟随。夹爪不参与跟随，也不会
收到模式、使能、失能或运动指令。

## 已验证硬件参数

| 设备 | 串口 | 波特率 | 节点 |
| --- | --- | ---: | --- |
| Star Arm 102-LD 引导臂 | `/dev/ttyUSB0` | `1_000_000` | FashionStar ID `0..6` |
| reBot B601-DM 从臂 | `/dev/ttyACM0` | `921_600` | 达妙 ID `0x01..0x07` |

候选方向以从臂为准：

```text
joint1  joint2  joint3  joint4  joint5  joint6
  -1      -1      +1      +1      +1      -1
```

夹爪需要单独标定旋转角度与开合行程，本工具不会把夹爪标记为已验证。

## 安装

在独立仓库根目录执行：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r \
  Python_SDK/rebot_b601_mapping/requirements-dev.txt
```

确认设备存在且没有被其他进程占用：

```bash
ls -l /dev/ttyUSB0 /dev/ttyACM0
fuser /dev/ttyUSB0 /dev/ttyACM0
```

如果 `fuser` 输出 PID，先在对应线程中正常停止占用者。本工具只会拒绝启动，不会自动
终止进程、抢占串口或修改设备权限。

## 运行 20 样本只读快照

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m rebot_b601_mapping.cli snapshot \
  --leader-port /dev/ttyUSB0 \
  --follower-port /dev/ttyACM0 \
  --config Python_SDK/rebot_b601_mapping/mapping.example.json \
  --baseline-samples 5 \
  --samples 20 \
  --interval-s 0.02 \
  --output /tmp/rebot-b601-mapping-snapshot.json
```

也可以运行不会被普通 `pytest` 收集的真机冒烟入口：

```bash
PYTHONPATH=Python_SDK .venv/bin/python \
  Python_SDK/rebot_b601_mapping/tests/hardware_snapshot_smoke.py \
  --leader-port /dev/ttyUSB0 \
  --follower-port /dev/ttyACM0 \
  --samples 20 \
  --output /tmp/rebot-b601-mapping-snapshot.json
```

证据文件包含两侧原始反馈、从臂状态码、相对基线以及根据候选符号计算的六关节虚拟
从臂位置。虚拟位置只写入 JSON，不会发送给从臂。

## 逐关节验证方向

先复制一份本机配置；模板被程序保护，不能直接写入：

```bash
cp Python_SDK/rebot_b601_mapping/mapping.example.json \
  Python_SDK/rebot_b601_mapping/mapping.local.json
```

每次只验证一个关节，例如 `joint1`：

```bash
PYTHONPATH="$PWD/Python_SDK${PYTHONPATH:+:$PYTHONPATH}" \
  .venv/bin/python -m rebot_b601_mapping.cli \
  calibrate-directions \
  --joint joint1 \
  --leader-port /dev/ttyUSB0 \
  --follower-port /dev/ttyACM0 \
  --config Python_SDK/rebot_b601_mapping/mapping.local.json \
  --output /tmp/rebot-b601-joint1-direction.json
```

操作步骤：

1. 保持两台机械臂稳定，程序采集五组基线。
2. 按提示托住失能机械臂。
3. 只移动两台机械臂各自对应的当前关节，其他关节必须保持不动。
4. 保持新位置稳定并按回车，让程序采集五组方向样本。
5. 核对两侧原始增量和推断符号；确认正确后完整输入“确认”。

只有推断符号与候选符号一致且操作员完整输入“确认”时，程序才会原子更新
`mapping.local.json` 中当前关节的 `verified` 和证据字段。其他关节及夹爪不会改变。

依次把 `--joint` 改为 `joint2` 到 `joint6`。任一关节出现其他轴运动、幅度不足、符号
不一致、状态码不为零或反馈过期时，应停止本关节验收并检查机械状态。

## 使用 RViz 虚拟验证方向

该模式只占用 Star Arm 102-LD 的 `/dev/ttyUSB0`，不会打开真实 reBot 从臂的
`/dev/ttyACM0`。为避免与其他正在运行的 ROS 任务冲突，两个终端必须使用相同且独立的
`ROS_DOMAIN_ID`。先在终端一启动 reBot 模型：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/project/rebot_Arm/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch rebotarm_bringup rviz.launch.py arm_namespace:=mapping_preview
```

再在终端二启动只读映射发布器：

```bash
cd /home/a/project/Star-Arm-102-sdk-test
source /opt/ros/jazzy/setup.bash
source /home/a/project/rebot_Arm/install/setup.bash
export ROS_DOMAIN_ID=42
PYTHONPATH=Python_SDK .venv/bin/python -m rebot_b601_mapping.cli \
  rviz-preview \
  --leader-port /dev/ttyUSB0 \
  --config Python_SDK/rebot_b601_mapping/mapping.example.json \
  --topic /mapping_preview/joint_states \
  --rate-hz 20
```

启动时保持引导臂不动，程序采集五个样本作为相对基线。随后每次只缓慢拨动 J1～J6
中的一个关节，确认 RViz 中 reBot 模型只有对应关节运动，且方向符合从臂坐标定义。
按 `Ctrl+C` 结束发布器。虚拟从臂初始位置采用各关节软限位中点，预览过程中不会启动
MoveIt、MuJoCo 或真实从臂控制器。

## 实时跟随参数

J1～J6 使用已经在 RViz 中目视确认的方向：

```text
[-1, -1, +1, +1, +1, -1]
```

程序以两台机械臂各自的启动位置建立相对基线，不复制引导臂绝对角度：

```text
q_target = qF0 + sign * radians(q_leader - qL0)
```

默认控制参数如下：

| 参数 | 数值 |
| --- | ---: |
| 目标控制频率 | `50 Hz` |
| 默认速度 | `0.5 rad/s` |
| 可配置速度硬上限 | `1.5 rad/s` |
| 最大加速度 | `5.0 rad/s²` |
| 最大加加速度 | `20.0 rad/s³` |
| 跟踪误差与持续时间 | `0.25 rad / 0.30 s` |
| 引导臂/从臂反馈超时 | `0.5 s / 0.25 s` |

J1～J6 的位置安全边界只使用网页遥操作的关节上下限，不再叠加相对启动基线最大偏移。
完整配置位于 `live_follow.example.json`。运行时速度不得超过 `1.5 rad/s`。

## 静态预检

两台机械臂上电后，先确认另一个任务已经正常停止 ROS 控制器。程序不会自动终止进程、
抢占串口或修改设备权限：

```bash
fuser -v /dev/ttyUSB0 /dev/ttyACM0
```

不带运动确认参数运行 `follow`，只读取两侧反馈和状态：

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m rebot_b601_mapping.cli follow \
  --leader-port /dev/ttyUSB0 \
  --follower-port /dev/ttyACM0 \
  --mapping-config Python_SDK/rebot_b601_mapping/mapping.example.json \
  --live-config Python_SDK/rebot_b601_mapping/live_follow.example.json \
  --log /tmp/stararm-rebot-live-follow-preflight.jsonl
```

预检要求从臂 J1～J6 和夹爪均为 `status_code=0`。成功输出必须明确显示“未使能、未发送
运动命令”。机械臂未上电时不要运行这项硬件预检，只运行下方自动化测试。

## J1～J6 实时跟随

确认工作空间无障碍物、从臂有可靠支撑并且操作员可以触及急停后，显式授权真机运动：

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m rebot_b601_mapping.cli follow \
  --leader-port /dev/ttyUSB0 \
  --follower-port /dev/ttyACM0 \
  --mapping-config Python_SDK/rebot_b601_mapping/mapping.example.json \
  --live-config Python_SDK/rebot_b601_mapping/live_follow.example.json \
  --log /tmp/stararm-rebot-live-follow.jsonl \
  --speed-rad-s 0.5 \
  --confirm-live-motion
```

启动后，从臂先以当前反馈 `qF0` 为首帧保持目标；从臂稳定使能后才重新采集引导臂基线
`qL0`，避免使能期间的引导臂移动形成待追赶的跳变。启动阶段不会写 J1～J6 的任何
寄存器，也不会调用可能改写模式的 `ensure_mode()`；程序只读核对 RID 10 的
`POS_VEL` 模式，并确认 RID 25～28 的现有增益均为有限非负数。现有增益与主项目 YAML
数值不要求精确相等；寄存器读取失败、模式错误或增益无效时才在使能前停止并报告。

- 第一次 `Ctrl+C`：保持当前位置，受控返回
  `[-1.5493631363, 0.0165939331, -0.0200271606, -0.0085830688, 0.1039524078, 0.0013351440]`，
  验证到位后失能。该姿态只作为网页停放安全位使用；启动时直接保持从臂当前姿态，
  不要求或命令 ready 姿态。
- 回位失败但从臂通信和电机健康：保持使能，持续保持当前位置，等待输入 `retry` 或
  `emergency_stop`。
- 第二次 `Ctrl+C` 或输入 `emergency_stop`：请求紧急停机；如果通信已经失效，程序会
  如实报告失能结果未知，而不会宣称安全关闭。

JSONL 日志记录原始目标、整形命令、反馈、跟踪误差、主从滞后、状态转换、安全位和失能
验证。默认写到 `/tmp`，不污染仓库。

## 安全边界

三个只读命令的硬件路径只允许以下操作：

- 引导臂：`send_sync_servo_monitor()`；
- 从臂：注册电机、`request_feedback()`、`poll_feedback_once()`、`get_state()`；
- 退出：电机句柄和控制器句柄的 `close()`。

实时 `follow` 路径直接使用 `motorbridge` 的六轴方法，并显式只读核对模式和增益，再检查
使能、命令、反馈、回位和失能结果。它不使用 `RobotArm.disconnect()`、`Controller.enable_all()`、
`Controller.disable_all()` 或控制器上下文管理器，避免隐藏生命周期动作。夹爪始终只读。
J1～J6 位置范围直接对齐网页遥操作加载的 URDF 边界，不另设位置余量；原始目标越过该
范围时停止跟随，不静默裁剪。旧配置中的 `joint_margin_rad` 已不再受支持，加载时会明确
拒绝，避免静默忽略造成安全语义误判。安全回位的起点和逐周期反馈也必须位于同一闭区间；
反馈越界时不得判定安全位到达或自动失能，而是使用最后一次已发送的界内命令保持使能并
进入人工恢复。使能结果会区分“实际发送的启动命令”和“发送后的反馈”，因此使能后反馈
突然越界也不会污染备用保持命令。从臂最终命令出口在使能保持及每次周期写入前再次检查
同一组边界；正常跟随的每周期反馈也必须处于该闭区间，否则立即使用最后界内命令保持并
转入人工恢复。

操作员必须始终掌握物理断电手段，并为从臂提供必要支撑。可恢复异常不得直接失能：
程序先保持，再受控回安全位；回位失败但状态健康时继续使能保持。只有严重通信/电机故障
或明确紧急停止才允许离开安全位请求保护性失能。

## 自动化验证

普通测试不会打开串口：

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests -q
PYTHONPATH=Python_SDK .venv/bin/python -m compileall \
  Python_SDK/rebot_b601_mapping -q
```
