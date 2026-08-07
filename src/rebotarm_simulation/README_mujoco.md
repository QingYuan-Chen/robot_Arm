# reBotArm MuJoCo 仿真底座

本目录提供可独立使用的 MuJoCo 物理仿真核心、桌面 Viewer 和 ROS 2
适配层。已验证的目标环境是 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。
本阶段尚未实现强化学习训练、Gymnasium 环境或奖励函数；云端训练是后续工作。

## 安装与构建

在 Ubuntu VM 的工作区根目录执行。先 source ROS，确保虚拟环境能看到 Jazzy
的 `rclpy`；`--system-site-packages` 是必需的。colcon 当前需要兼容版本的
setuptools，因此限定为 `setuptools>=68,<80`。

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
python3 -m venv .venv-mujoco-ros --system-site-packages
source .venv-mujoco-ros/bin/activate
python -m pip install --upgrade pip 'setuptools>=68,<80'
python -m pip install -r src/rebotarm_simulation/requirements-mujoco.txt
.venv-mujoco-ros/bin/python -m colcon build --symlink-install --packages-select rebotarm_simulation
source install/setup.bash
```

每个新终端都按 `source /opt/ros/jazzy/setup.bash`、激活 venv、再
`source install/setup.bash` 的顺序初始化。

从源码调试时应显式使用 `.venv-mujoco-ros/bin/python -m ...`；colcon 安装后
的 `rebotarm_mujoco_*` console script 会在构建时写入解释器 shebang。因此若
更换或重建 venv，必须重新 colcon build，不能假定旧脚本会自动改用当前 Python。
以下是从工作区源码直接运行的完整等价命令，不依赖已安装的 console script：

```bash
MUJOCO_GL=egl PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_health --renderer-timeout 30
PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_cli --headless --duration 5
PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_viewer --duration 30
```

## 健康检查与无界面运行

EGL 渲染检查在隔离子进程中运行，并有超时保护，避免驱动挂起拖死主进程：

```bash
MUJOCO_GL=egl rebotarm_mujoco_health --renderer-timeout 30
```

预期 JSON 关键字段如下；版本和路径允许不同，但关节/执行器均应为 8，
`ok`、模型加载和有限步进应为 true。无 EGL 设备时物理健康仍可为 true，
同时 `renderer_available` 为 false 并给出 `renderer_error`。

```json
{"ok": true, "model_loaded": true, "physics_step_finite": true,
 "joint_count": 8, "expected_joint_count": 8,
 "actuator_count": 8, "expected_actuator_count": 8,
 "headless": true, "renderer_available": true}
```

不依赖窗口的定时运行：

```bash
rebotarm_mujoco_cli --headless --duration 5
```

输出同时含 `"requested_duration"` 和 `"achieved_duration"`；后者会按物理
时间步向上取整。交互 CLI 还支持 `state`、`joint`、`joints`、`jog`、
`gripper`、`step`、`contacts`、`reset`、`pause`、`resume` 和 `quit`。

## 桌面 Viewer

在 Ubuntu 图形桌面的终端运行，而不是普通无显示 SSH 会话：

```bash
rebotarm_mujoco_viewer --duration 30
```

按 `1`–`6` 选择关节，按住 `J/K` 连续正/反向移动当前关节，按住 `C/O`
连续闭合/打开夹爪，`G` 进入重力补偿，`H` 保持当前位置，`P` 进入 POS_VEL
位置目标控制，空格暂停，`.` 单步，`R` 回零位，`T` 回 home 姿态，`Q` 退出。
终端状态栏会显示当前模式、选中关节、实际关节角和目标关节角。MuJoCo 右侧
`control` 面板显示的是底层 actuator 的 torque/force，不是关节位置；日常控制请用
键盘 jog 或 ROS/API 发送目标。若从 SSH 启动，需正确配置 X11 转发，且
`DISPLAY` 与 `XAUTHORITY` 必须指向当前桌面会话；否则请改用 headless CLI。

模型使用同一套原始 STL 作为 visual 与 collision。为避免两个完全重合的网格在
Viewer 中互相闪烁或遮挡，collision 网格保留参与碰撞，但显示为半透明调试层；
正常观察以 visual 网格为准。场景只 include `models/rebotarm/robot.xml`，没有再叠加
旧的手写机械臂模型。

## ROS 2 适配层

只启动 MuJoCo 后端：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

节点提供五个 reBotArm 接口并额外发布仿真时钟：

- Action：`/rebotarm/follow_joint_trajectory`
- Topic：`/rebotarm/joint_states`
- Service：`/rebotarm/gripper/set`
- Topic：`/rebotarm/gripper/state`
- Service：`/rebotarm/trajectory_stop`
- Topic：`/clock`

MuJoCo 物理步进定时器固定使用 steady clock，不依赖节点的 ROS 时钟；消费仿真状态的
MoveIt、RViz 和状态发布节点使用 `use_sim_time:=true` 跟随 `/clock`。这样 `/clock` 的
发布者不会反过来等待自己尚未发布的仿真时间。

另开已 source 的终端发送一个六关节目标：

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.1, -0.2, -0.2, 0.2, 0.0, 0.0], time_from_start: {sec: 3}}]}}" --feedback

ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: 0.05, max_effort: 10.0}"

ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

MoveIt 的控制器映射保持为 `/rebotarm/follow_joint_trajectory`。联调时所有
bringup 必须显式使用 `use_hardware:=false`，且只保留一个该 Action 的服务端。

### MuJoCo 与 MoveIt 的安全双终端接线

终端 1 先启动唯一的 MuJoCo 轨迹服务端：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

终端 2 启动 MoveIt 预览，并明确关闭硬件、fake joint state publisher 和
passive joint state publisher：

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py use_moveit_preview:=true use_hardware:=false use_moveit_fake_joint_states:=false start_passive_joint_state_publisher:=false use_sim_time:=true
```

如需本地 RViz，可在第二条命令末尾追加 `use_local_rviz:=true`。关闭 fake 和
passive 发布器是为了让 `/rebotarm/joint_states` 只有 MuJoCo 这一份权威物理状态，
避免多个 joint state publisher 竞争。MoveIt 控制器仍映射到
`/rebotarm/follow_joint_trajectory`，由终端 1 的 MuJoCo Action 服务端执行。

## Python API（面向后续训练/推理复用）

核心类不依赖 ROS 2：

```python
from rebotarm_simulation.mujoco_sim import RebotArmMujoco

with RebotArmMujoco() as sim:
    sim.reset(seed=7)
    sim.set_joint_position_targets([0.1, -0.2, -0.2, 0.2, 0.0, 0.0])
    sim.set_gripper_width(0.05)
    sim.set_object_pose("test_cube", [0.45, 0.0, 0.44], [0.0, 0.0, 0.0, 1.0])
    saved = sim.save_state()
    state = sim.step(10)
    sim.restore_state(saved)
    contacts = sim.get_contacts()
```

`get_state()` 返回关节位置/速度、执行器力、末端位姿、夹爪宽度、物体位姿
和仿真时间。公开的末端与物体四元数顺序都是 XYZW；MuJoCo 内部 WXYZ 已在
API 边界转换。`save_state()`/`restore_state()` 只允许同一模型实例，
`set_object_pose()` 只接受带 free joint 的物体，四元数也使用 XYZW。

后续架构是云服务器运行无界面 MuJoCo 并行训练，本地 Ubuntu VM 做模型验证、
ROS 2 联调和策略推理。现在只提供可复用物理/API 底座，不宣称训练可用。

模型在 `end_link` 下提供命名坐标系 `wrist_camera_mount`，作为后续手眼/末端 RGB-D
相机的稳定安装基准。当前阶段只定义安装位，不绑定具体相机型号、内参或渲染传感器。

## 双端同步规则

Windows 仓库是受版本控制的主副本，Ubuntu VM 是构建与运行环境。每次同步前
先备份 VM 上将被覆盖的明确文件；只传输本次文件清单，传后比较 SHA-256 哈希。
禁止使用带 `--delete` 的目录镜像，也不反向同步 `.venv-mujoco-ros`、
`build/`、`install/`、`log/` 或缓存。

## 排障

- **重复节点**：若 Action 提示重复服务端，先用 `ros2 node list` 检查重复节点。
  仅结束 launch 父进程可能留下 `rebotarm_mujoco_node` 子进程；先用
  `pgrep -af rebotarm_mujoco_node` 定向查找候选 PID，再用
  `ps -fp <confirmed-pid>` 核对完整命令。只在确认它是本次仿真的遗留进程后，执行
  `kill -TERM <confirmed-pid>`，再复查节点列表。禁止宽泛 kill，
  也禁止同时运行旧 RViz-only 控制器。
- **SSH 无窗口**：检查 `DISPLAY`、`XAUTHORITY` 和 X11 转发；物理验证优先用
  `--headless`。
- **EGL 超时/崩溃**：健康检查的 renderer 子进程会在 `--renderer-timeout`
  后报告失败；可先用 `--skip-renderer` 单独确认物理模型。
- **轨迹未成功**：检查配置中的 goal tolerance（位置、速度和时间），当前采用
  节点级容差，不接受每个 goal 的覆盖值。
- **规划或接触异常**：MoveIt self-collision 矩阵与 MuJoCo 接触过滤是不同层，
  分别检查 SRDF self-collision 与 MJCF 的 `contype/conaffinity`。

## 无硬件安全边界

本流程不得设置 `use_hardware:=true`，不得启动 `rebotarmcontroller`，不得打开
CAN、串口或真实夹爪，不得发送实机使能、回零或轨迹命令。发现任何硬件节点、
CAN 设备或串口已被占用时，停止联调并确认环境；MuJoCo 测试不需要连接实机。

## 从 URDF 生成本地 MJCF

机器人结构的唯一权威来源是
`src/rebotarm_moveit_config/config/rebotarm.urdf`，MuJoCo 日常运行读取本地文件
`src/rebotarm_simulation/models/rebotarm/robot.xml`。后者是自动生成并随仓库提交的
MJCF，不需要联网加载。当前支持范围固定为 `mujoco>=3.3,<4`。

URDF 或其引用的原始 STL 更新后，在仓库根目录重新生成：

```bash
rebotarm_urdf_to_mjcf --repo-root .
```

只检查仓库中的 MJCF 是否仍与 URDF 一致，不写文件：

```bash
rebotarm_urdf_to_mjcf --repo-root . --check
```

转换使用 MuJoCo 官方 URDF 解析器。原始 STL 同时保留为视觉网格和碰撞网格，不执行
VHACD。执行器、双指联动、传感器、末端 site 和接触过滤由确定性后处理补入。不要手工
修改 `robot.xml`；应修改 URDF 或生成器后重新生成。

Windows 与 Ubuntu VM 同步后，可分别检查受控文件哈希：

```powershell
Get-FileHash src/rebotarm_simulation/models/rebotarm/robot.xml -Algorithm SHA256
```

```bash
sha256sum src/rebotarm_simulation/models/rebotarm/robot.xml
```

生成、检查和运行均不连接实机，不探测 CAN、串口或机械臂控制器；ROS 2 联调继续明确
使用 `use_hardware:=false`。
