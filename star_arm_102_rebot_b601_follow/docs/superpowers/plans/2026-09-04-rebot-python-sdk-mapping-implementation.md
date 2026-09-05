# Star Arm 102-LD → reBot B601 Python SDK 方向映射实施计划

> **供自动化执行代理使用：** 必须逐项执行本计划；开始实施时使用
> `superpowers:executing-plans`，每个任务严格遵循测试驱动开发流程。

**目标：** 在独立仓库中实现一个无需 ROS、严格只读的 Python 工具，读取 Star Arm
102-LD 引导臂与 reBot B601-DM 从臂，并以从臂坐标为基准验证 J1～J6 的方向映射。

**架构：** 工具分为不可变数据模型、纯映射与验收逻辑、两个只读硬件适配器和命令行
编排层。硬件适配器通过依赖注入支持无硬件单元测试；命令行层负责端口占用检查、资源
关闭、交互确认和 JSON 证据输出。方向标定只写配置证据，不向机械臂发送控制指令。

**技术栈：** Python 3.12、`pyserial==3.5`、
`fashionstar-uart-sdk==1.3.12`、`motorbridge==0.4.6`、`pytest==7.4.4`。

**设计依据：**
`docs/superpowers/specs/2026-09-04-rebot-python-sdk-mapping-design.md`

## 全局约束

- 所有新增代码只能位于 `Python_SDK/rebot_b601_mapping/`；不得修改
  `/home/a/project/rebot_Arm`。
- 引导臂固定使用 `/dev/ttyUSB0`、`1_000_000` 波特率、ID `0..6`；只允许调用
  `send_sync_servo_monitor()`。
- 从臂固定使用 `/dev/ttyACM0`、`921_600` 波特率；机械臂 ID 为
  `0x01..0x06`，夹爪 ID 为 `0x07`，反馈 ID 为 `0x11..0x17`。
- 从臂只能调用 `add_damiao_motor()`、`request_feedback()`、
  `poll_feedback_once()`、`get_state()` 和无指令的 `close()`。
- 不得使用 `RobotArm`，因为其 `disconnect()` 会调用 `disable()`；也不得使用
  `Controller` 上下文管理器，因为其 `__exit__()` 会调用 `shutdown()`。
- 不得调用使能、失能、置零、清错、模式切换、参数写入、位置、速度、力矩或轨迹接口。
- J1～J6 候选符号固定为 `[-1, -1, +1, +1, +1, -1]`，初始比例均为 `1.0`；
  夹爪保持未验证，不参与机械臂方向验收。
- 单元测试不得打开真实串口；真机脚本不得被普通 `pytest` 自动收集。
- `/home/a/project/rebot_Arm` 的工作区状态属于另一线程，所有读写、暂存和提交均限定在
  `/home/a/project/Star-Arm-102-sdk-test`。

---

### 任务 1：建立数据模型和可审计配置

**文件：**

- 新建：`Python_SDK/rebot_b601_mapping/__init__.py`
- 新建：`Python_SDK/rebot_b601_mapping/models.py`
- 新建：`Python_SDK/rebot_b601_mapping/mapping.example.json`
- 新建：`Python_SDK/rebot_b601_mapping/.gitignore`
- 新建：`Python_SDK/rebot_b601_mapping/requirements.txt`
- 新建：`Python_SDK/rebot_b601_mapping/requirements-dev.txt`
- 新建：`Python_SDK/rebot_b601_mapping/tests/test_models.py`

**接口：**

- 产出：`JointSpec`、`Thresholds`、`MappingConfig`、`LeaderSample`、
  `MotorFeedback`、`FollowerSample`、`Baseline`、`MappingResult`、
  `DirectionEvidence` 不可变数据类。
- 产出：`load_mapping_config(path: Path) -> MappingConfig`，严格校验配置结构、
  关节顺序、候选符号和夹爪未验证状态。
- 后续任务只使用这些公开数据类，不在读取器或命令行层重复定义字典结构。

- [ ] **步骤 1：编写配置加载失败测试**

```python
def test_load_mapping_config_preserves_follower_authority(tmp_path):
    example = Path(__file__).parents[1] / "mapping.example.json"
    path = tmp_path / "mapping.json"
    path.write_text(example.read_text(encoding="utf-8"), encoding="utf-8")
    config = load_mapping_config(path)
    assert [joint.follower_name for joint in config.arm_joints] == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    ]
    assert [joint.sign for joint in config.arm_joints] == [-1, -1, 1, 1, 1, -1]
    assert all(joint.scale == 1.0 for joint in config.arm_joints)
    assert config.gripper.follower_name == "gripper"
    assert config.gripper.sign is None
    assert config.gripper.verified is False


def test_load_mapping_config_rejects_verified_gripper(tmp_path):
    example = Path(__file__).parents[1] / "mapping.example.json"
    data = json.loads(example.read_text(encoding="utf-8"))
    data["mapping"][6]["verified"] = True
    path = tmp_path / "mapping.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="夹爪.*未验证"):
        load_mapping_config(path)
```

- [ ] **步骤 2：运行测试并确认因模块不存在而失败**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_models.py -q
```

预期：测试收集阶段出现 `ModuleNotFoundError`。

- [ ] **步骤 3：实现不可变模型和严格配置加载器**

`models.py` 中使用 `@dataclass(frozen=True)`，并提供以下完整数据结构：

```python
@dataclass(frozen=True)
class JointSpec:
    leader_id: int
    follower_name: str
    sign: int | None
    scale: float | None
    lower_rad: float | None
    upper_rad: float | None
    verified: bool


@dataclass(frozen=True)
class Thresholds:
    max_sample_age_s: float
    baseline_max_velocity_rad_s: float
    min_direction_delta_rad: float
    other_joint_max_delta_rad: float
    sign_window_size: int


@dataclass(frozen=True)
class MappingConfig:
    leader_ids: tuple[int, ...]
    arm_joints: tuple[JointSpec, ...]
    gripper: JointSpec
    thresholds: Thresholds


@dataclass(frozen=True)
class LeaderSample:
    timestamp_s: float
    angles_deg: tuple[float, ...]


@dataclass(frozen=True)
class MotorFeedback:
    name: str
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    status_code: int


@dataclass(frozen=True)
class FollowerSample:
    timestamp_s: float
    motors: tuple[MotorFeedback, ...]


@dataclass(frozen=True)
class Baseline:
    captured_at_s: float
    leader_angles_deg: tuple[float, ...]
    follower_positions_rad: tuple[float, ...]


@dataclass(frozen=True)
class MappingResult:
    positions_rad: tuple[float, ...]
    leader_deltas_rad: tuple[float, ...]


@dataclass(frozen=True)
class DirectionEvidence:
    follower_name: str
    observed_at_s: float
    leader_delta_rad: float
    follower_delta_rad: float
    inferred_sign: int
    candidate_sign: int
    consistent: bool
    confirmed: bool
    verified: bool
```

配置加载器必须拒绝重复 ID、重复名称、非有限阈值、错误的六关节顺序、非 `±1` 的机械臂
符号、非正比例、夹爪符号不为 `null` 或夹爪 `verified=true`。

- [ ] **步骤 4：填写示例配置与依赖清单**

`mapping.example.json` 必须包含七个映射项、从臂软限位，以及以下阈值：

```json
{
  "max_sample_age_s": 0.25,
  "baseline_max_velocity_rad_s": 0.05,
  "min_direction_delta_rad": 0.05,
  "other_joint_max_delta_rad": 0.02,
  "sign_window_size": 5
}
```

依赖文件固定版本；开发依赖文件通过 `-r requirements.txt` 引入运行依赖并增加
`pytest==7.4.4`。`.gitignore` 只忽略本机验收产物：

```gitignore
mapping.local.json
evidence/
```

- [ ] **步骤 5：运行模型测试并提交**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_models.py -q
```

预期：全部通过。

提交：

```bash
git add Python_SDK/rebot_b601_mapping
git commit -m "feat: 添加方向映射数据模型"
```

---

### 任务 2：实现纯映射、基线和方向验收逻辑

**文件：**

- 新建：`Python_SDK/rebot_b601_mapping/mapping.py`
- 新建：`Python_SDK/rebot_b601_mapping/tests/test_mapping.py`

**接口：**

- 消费：任务 1 的不可变模型和 `MappingConfig`。
- 产出：`capture_baseline()`、`map_virtual_follower()`、
  `validate_paired_sample()`、`infer_direction()`、`apply_confirmation()`。
- `map_virtual_follower()` 只返回 J1～J6 的虚拟目标，绝不生成夹爪目标。

函数签名固定为：

```python
def validate_paired_sample(
    leader: LeaderSample,
    follower: FollowerSample,
    config: MappingConfig,
    *,
    now_s: float,
) -> None: ...

def capture_baseline(
    paired_samples: Sequence[tuple[LeaderSample, FollowerSample]],
    config: MappingConfig,
    *,
    now_s: float,
) -> Baseline: ...

def map_virtual_follower(
    leader: LeaderSample,
    baseline: Baseline,
    config: MappingConfig,
) -> MappingResult: ...
```

`capture_baseline()` 至少需要 `sign_window_size` 组有效样本；逐轴检查从臂速度不超过
`baseline_max_velocity_rad_s`，然后分别取引导臂角度和从臂位置的中位数。

- [ ] **步骤 1：编写相对基线和候选符号测试**

```python
def test_virtual_mapping_uses_relative_baseline_and_follower_signs(config):
    baseline = Baseline(
        captured_at_s=1.0,
        leader_angles_deg=(10, 20, 30, 40, 50, 60, 70),
        follower_positions_rad=(1, 2, 3, 4, 5, 6, 0.5),
    )
    sample = LeaderSample(
        timestamp_s=1.1,
        angles_deg=(20, 30, 40, 50, 60, 70, 80),
    )
    result = map_virtual_follower(sample, baseline, config)
    delta = math.radians(10)
    assert result.positions_rad == pytest.approx(
        (1-delta, 2-delta, 3+delta, 4+delta, 5+delta, 6-delta)
    )
    assert len(result.positions_rad) == 6
```

- [ ] **步骤 2：编写失败关闭测试**

覆盖以下情况并要求抛出带关节名的 `ValueError`：样本过期、`nan`/`inf`、从臂状态不为零、
位置越过软限位、基线速度超过阈值、非选定关节增量超过阈值、选定关节增量不足、
窗口内推断符号不一致。

- [ ] **步骤 3：运行测试并确认失败**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_mapping.py -q
```

预期：因 `mapping.py` 尚不存在而失败。

- [ ] **步骤 4：实现最小纯函数**

方向推断接口固定为：

```python
def infer_direction(
    baseline: Baseline,
    paired_window: Sequence[tuple[LeaderSample, FollowerSample]],
    selected_joint: str,
    config: MappingConfig,
    *,
    now_s: float,
) -> DirectionEvidence:
    ...
```

每个窗口样本分别计算：

```text
leader_delta = radians(current_leader_deg - baseline_leader_deg)
follower_delta = current_follower_rad - baseline_follower_rad
inferred_sign = sign(follower_delta / leader_delta)
```

必须先验证两侧选定关节增量均达到 `min_direction_delta_rad`，再验证其他 J1～J6 的两侧
增量不超过 `other_joint_max_delta_rad`，最后要求最近 `sign_window_size` 个样本推断符号相同。

`apply_confirmation(evidence, confirmed)` 只在 `confirmed is True` 且推断符号等于配置中的
候选符号时返回 `verified=True` 的新证据；否则不改变配置状态。

- [ ] **步骤 5：运行映射测试并提交**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_mapping.py -q
```

预期：全部通过。

提交：

```bash
git add Python_SDK/rebot_b601_mapping/mapping.py \
  Python_SDK/rebot_b601_mapping/tests/test_mapping.py
git commit -m "feat: 添加只读方向映射逻辑"
```

---

### 任务 3：实现串口身份和占用防护

**文件：**

- 新建：`Python_SDK/rebot_b601_mapping/ports.py`
- 新建：`Python_SDK/rebot_b601_mapping/tests/test_ports.py`

**接口：**

- 产出：`PortIdentity.capture(path: str) -> PortIdentity`。
- 产出：`assert_same_port(expected: PortIdentity) -> None`。
- 产出：`assert_ports_unoccupied(paths: Sequence[str], proc_root=Path("/proc")) -> None`。
- 端口防护只检查和拒绝，不终止进程、不修改权限、不自动抢占设备。

- [ ] **步骤 1：用临时 `/proc` 结构编写占用检测测试**

```python
def test_assert_ports_unoccupied_reports_pid_and_port(tmp_path):
    device = tmp_path / "ttyACM0"
    device.touch()
    fd_dir = tmp_path / "4242" / "fd"
    fd_dir.mkdir(parents=True)
    (fd_dir / "7").symlink_to(device)
    with pytest.raises(RuntimeError, match=r"ttyACM0.*4242"):
        assert_ports_unoccupied([str(device)], proc_root=tmp_path)
```

另测路径解析后设备号变化时 `assert_same_port()` 拒绝继续。

- [ ] **步骤 2：运行测试并确认失败**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_ports.py -q
```

预期：因 `ports.py` 尚不存在而失败。

- [ ] **步骤 3：实现只读端口检查**

`PortIdentity` 保存解析后的绝对路径、`st_rdev` 和设备类型。占用扫描遍历
`/proc/[0-9]*/fd/*`，只比较解析后的目标路径；无法读取的进程目录跳过，命中时一次性报告
全部 PID 和端口。

- [ ] **步骤 4：运行测试并提交**

运行任务 3 测试，预期全部通过，然后提交：

```bash
git add Python_SDK/rebot_b601_mapping/ports.py \
  Python_SDK/rebot_b601_mapping/tests/test_ports.py
git commit -m "feat: 添加串口占用与身份防护"
```

---

### 任务 4：实现 Star Arm 102-LD 只读读取器

**文件：**

- 新建：`Python_SDK/rebot_b601_mapping/leader_reader.py`
- 新建：`Python_SDK/rebot_b601_mapping/tests/test_leader_reader.py`

**接口：**

- 消费：`LeaderSample`、`PortIdentity`、`assert_same_port()`。
- 产出：`LeaderReader.open()`、`read_sample()`、`close()`。
- 构造函数注入 `serial_factory`、`manager_factory` 和 `clock`，使单元测试不导入或打开
  真实串口。

- [ ] **步骤 1：编写只允许监视查询的测试**

```python
def test_read_sample_uses_monitor_query_only(fake_serial, fake_manager):
    reader = LeaderReader(
        "/dev/fake-leader",
        serial_factory=lambda **kwargs: fake_serial,
        manager_factory=lambda serial_obj: fake_manager,
        clock=lambda: 12.5,
        identity_checker=lambda identity: None,
    )
    reader.open()
    sample = reader.read_sample()
    assert fake_manager.calls == [
        ("send_sync_servo_monitor", (0, 1, 2, 3, 4, 5, 6), True)
    ]
    assert sample.timestamp_s == 12.5
    assert sample.angles_deg == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
```

另测 ID 缺失、`angle_monitor=None`、非有限角度、重复打开、读取异常和 Ctrl+C 后
`close()` 只关闭串口。

- [ ] **步骤 2：运行测试并确认失败**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_leader_reader.py -q
```

- [ ] **步骤 3：实现读取器**

真实串口参数固定为：

```python
serial.Serial(
    port=port,
    baudrate=1_000_000,
    parity=serial.PARITY_NONE,
    stopbits=1,
    bytesize=8,
    timeout=0.05,
    exclusive=True,
)
```

每次读取只调用：

```python
states = manager.send_sync_servo_monitor(SERVO_IDS, realtime=True)
```

读取器类不得定义或转发 `stop_on_control_mode`、`reset_multi_turn_angle`、`ping` 或任何
写入方法。`__exit__()` 若实现，只能调用 `close()`。

- [ ] **步骤 4：运行测试并提交**

运行任务 4 测试，预期全部通过，然后提交：

```bash
git add Python_SDK/rebot_b601_mapping/leader_reader.py \
  Python_SDK/rebot_b601_mapping/tests/test_leader_reader.py
git commit -m "feat: 添加引导臂只读读取器"
```

---

### 任务 5：实现 reBot B601-DM 只读读取器

**文件：**

- 新建：`Python_SDK/rebot_b601_mapping/follower_reader.py`
- 新建：`Python_SDK/rebot_b601_mapping/tests/test_follower_reader.py`

**接口：**

- 消费：`FollowerSample`、`MotorFeedback`、`PortIdentity`、`assert_same_port()`。
- 产出：`FollowerReader.open()`、`read_sample()`、`close()`。
- 构造函数注入 `controller_factory` 和 `clock`；默认工厂惰性导入 `motorbridge` 并调用
  `Controller.from_dm_serial(port, 921_600)`。

- [ ] **步骤 1：编写精确电机注册和反馈测试**

```python
def test_follower_registers_exact_b601_motors_and_reads_feedback(fake_controller):
    reader = FollowerReader(
        "/dev/fake-follower",
        controller_factory=lambda port, baud: fake_controller,
        clock=lambda: 20.0,
        identity_checker=lambda identity: None,
    )
    reader.open()
    sample = reader.read_sample()
    assert fake_controller.add_calls == [
        (0x01, 0x11, "4340P"), (0x02, 0x12, "4340P"),
        (0x03, 0x13, "4340P"), (0x04, 0x14, "4310"),
        (0x05, 0x15, "4310"), (0x06, 0x16, "4310"),
        (0x07, 0x17, "4310"),
    ]
    assert [motor.request_count for motor in fake_controller.motors] == [1] * 7
    assert fake_controller.poll_count == 1
    assert [motor.name for motor in sample.motors] == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"
    ]
```

- [ ] **步骤 2：编写禁止生命周期调用测试**

伪控制器的 `shutdown()`、`disable_all()`、`enable_all()` 以及伪电机的 `enable()`、
`disable()`、`ensure_mode()`、`set_zero_position()`、`send_*()` 均实现为立即抛错。测试
正常关闭、读取异常和 Ctrl+C 三条路径，证明只调用每个电机句柄的 `close()`，最后调用
控制器的 `close()`。

- [ ] **步骤 3：运行测试并确认失败**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_follower_reader.py -q
```

- [ ] **步骤 4：实现直接 `motorbridge` 读取器**

`read_sample()` 必须按顺序执行七次 `request_feedback()`、一次
`poll_feedback_once()`、七次 `get_state()`。任一状态缺失、非有限或
`status_code != 0` 时立即拒绝样本。

`close()` 的固定顺序为：逐个 `motor.close()`，再调用 `controller.close()`。不得使用
`with Controller...`，不得调用 `controller.shutdown()`。

- [ ] **步骤 5：运行测试并提交**

运行任务 5 测试，预期全部通过，然后提交：

```bash
git add Python_SDK/rebot_b601_mapping/follower_reader.py \
  Python_SDK/rebot_b601_mapping/tests/test_follower_reader.py
git commit -m "feat: 添加从臂只读反馈读取器"
```

---

### 任务 6：实现 `snapshot` 命令和结构化证据

**文件：**

- 新建：`Python_SDK/rebot_b601_mapping/cli.py`
- 新建：`Python_SDK/rebot_b601_mapping/tests/test_cli_snapshot.py`

**接口：**

- 消费：两个读取器、配置加载器、基线与映射纯函数、端口占用检查。
- 产出：`main(argv: Sequence[str] | None = None) -> int`。
- 产出命令：`python -m rebot_b601_mapping.cli snapshot ...`。

- [ ] **步骤 1：编写快照命令测试**

通过注入伪读取器工厂，验证命令先检查两端口、采集稳定基线、连续读取指定数量样本、
输出原始引导臂角度、从臂弧度值、六个虚拟从臂目标和 JSON 证据，并确保所有映射项仍为
`verified=false`。

关键断言：

```python
assert result.exit_code == 0
assert evidence["mode"] == "snapshot"
assert evidence["sample_count"] == 20
assert len(evidence["samples"][-1]["virtual_follower_rad"]) == 6
assert all(item["verified"] is False for item in evidence["mapping"])
assert leader.closed is True
assert follower.closed is True
```

- [ ] **步骤 2：编写异常资源关闭测试**

分别让引导臂读取、从臂读取和证据序列化抛出异常，并注入 `KeyboardInterrupt`；每条路径
都必须断言两个读取器已关闭，且未调用任何生命周期或运动方法。

- [ ] **步骤 3：运行测试并确认失败**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_cli_snapshot.py -q
```

- [ ] **步骤 4：实现命令行编排**

支持以下参数：

```text
snapshot
  --leader-port /dev/ttyUSB0
  --follower-port /dev/ttyACM0
  --config Python_SDK/rebot_b601_mapping/mapping.example.json
  --baseline-samples 5
  --samples 20
  --interval-s 0.02
  --output /tmp/rebot-b601-mapping-snapshot.json
```

`--output` 为必填参数。执行顺序固定为：检查两端口未占用 → 打开两个读取器 → 采集稳定
基线 → 连续读取 → 映射并输出 → 在 `finally` 中先关闭从臂、再关闭引导臂。输出目录只能
按用户明确参数创建。

- [ ] **步骤 5：运行测试并提交**

运行任务 6 测试，预期全部通过，然后提交：

```bash
git add Python_SDK/rebot_b601_mapping/cli.py \
  Python_SDK/rebot_b601_mapping/tests/test_cli_snapshot.py
git commit -m "feat: 添加只读关节快照命令"
```

---

### 任务 7：实现单关节方向标定和确认后持久化

**文件：**

- 修改：`Python_SDK/rebot_b601_mapping/mapping.py`
- 修改：`Python_SDK/rebot_b601_mapping/cli.py`
- 新建：`Python_SDK/rebot_b601_mapping/tests/test_cli_calibrate.py`

**接口：**

- 产出命令：`python -m rebot_b601_mapping.cli calibrate-directions --joint jointN ...`。
- 产出：`persist_verified_direction(config_path, evidence, *, confirmed)`，使用同目录临时文件
  加 `os.replace()` 原子更新，仅修改选定关节的 `verified` 和验证证据字段。

- [ ] **步骤 1：编写单关节隔离和拒绝持久化测试**

覆盖：未知关节、`gripper`、其他关节发生明显运动、运动幅度不足、窗口符号不一致、推断
符号与候选符号不一致、用户输入不是完整的 `确认`。所有拒绝路径都断言配置文件字节内容
完全未变。

- [ ] **步骤 2：编写确认成功测试**

```python
def test_calibration_persists_only_selected_joint_after_confirmation(...):
    result = run_calibration(joint="joint1", input_text="确认")
    updated = json.loads(config_path.read_text(encoding="utf-8"))
    assert result.exit_code == 0
    assert updated["mapping"][0]["sign"] == -1
    assert updated["mapping"][0]["verified"] is True
    assert [item["verified"] for item in updated["mapping"][1:]] == [False] * 6
    assert updated["mapping"][0]["evidence"]["leader_delta_rad"] != 0.0
    assert updated["mapping"][0]["evidence"]["follower_delta_rad"] != 0.0
```

- [ ] **步骤 3：运行测试并确认失败**

运行：

```bash
PYTHONPATH=Python_SDK python3 -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_cli_calibrate.py -q
```

- [ ] **步骤 4：实现交互式单关节流程**

命令必须显示：当前关节、候选符号、两侧基线、只允许移动该关节的要求，以及“同时托住
失能机械臂”的提示。流程在用户按回车后采集窗口，显示原始增量和推断符号；仅当用户
完整输入 `确认` 时才调用原子持久化函数。

命令一次只接受一个 `--joint joint1..joint6`，显式拒绝夹爪。任何异常都保留原配置，
并通过 `finally` 只关闭通信句柄。

- [ ] **步骤 5：运行测试并提交**

运行任务 7 测试，预期全部通过，然后提交：

```bash
git add Python_SDK/rebot_b601_mapping/mapping.py \
  Python_SDK/rebot_b601_mapping/cli.py \
  Python_SDK/rebot_b601_mapping/tests/test_cli_calibrate.py
git commit -m "feat: 添加单关节方向标定流程"
```

---

### 任务 8：补齐使用文档、真机冒烟入口和最终验证

**文件：**

- 新建：`Python_SDK/rebot_b601_mapping/README.md`
- 新建：`Python_SDK/rebot_b601_mapping/tests/hardware_snapshot_smoke.py`

**接口：**

- 真机脚本只调用 `snapshot` 的同一编排接口，不复制硬件访问逻辑。
- 文件名不以 `test_` 开头，因此普通 `pytest` 不会自动执行真机测试。

- [ ] **步骤 1：编写简体中文使用文档**

文档必须包含：创建独立虚拟环境、安装固定依赖、确认串口、运行 20 样本快照、逐关节方向
标定、证据文件说明、禁止命令清单、Ctrl+C 行为，以及“通过方向映射不代表允许真机跟随”
的边界。

用于方向验收时，先执行：

```bash
cp Python_SDK/rebot_b601_mapping/mapping.example.json \
  Python_SDK/rebot_b601_mapping/mapping.local.json
```

后续 `calibrate-directions` 只更新被 `.gitignore` 排除的 `mapping.local.json`，不修改已提交
的候选配置模板。

- [ ] **步骤 2：添加显式真机冒烟入口**

入口命令固定为：

```bash
PYTHONPATH=Python_SDK .venv/bin/python \
  Python_SDK/rebot_b601_mapping/tests/hardware_snapshot_smoke.py \
  --leader-port /dev/ttyUSB0 \
  --follower-port /dev/ttyACM0 \
  --samples 20 \
  --output /tmp/rebot-b601-mapping-snapshot.json
```

脚本启动时打印“只读、不会使能或控制机械臂”，然后调用正式 `snapshot` 路径。

- [ ] **步骤 3：运行完整自动化验证**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests -q
PYTHONPATH=Python_SDK .venv/bin/python -m compileall \
  Python_SDK/rebot_b601_mapping -q
git diff --check
```

预期：所有单元测试通过、编译通过、无空白错误，且真机脚本未被 `pytest` 收集。

- [ ] **步骤 4：静态审计禁止调用**

```bash
rg -n "stop_on_control_mode|reset_multi_turn_angle|shutdown\(|disable(_all)?\(|enable(_all)?\(|set_zero|ensure_mode|send_(mit|pos_vel|vel|force_pos)|write_register" \
  Python_SDK/rebot_b601_mapping \
  -g '*.py' -g '!test_*.py'
```

预期：正式代码无命中；如文档或测试伪对象出现这些名称，不计入正式代码审计。

- [ ] **步骤 5：运行只读真机快照**

先用 `/proc/*/fd` 占用检查确认两个串口未被其他线程或 ROS 进程持有，再运行 20 样本真机
冒烟命令。验收证据必须显示两端均为 20 个连续有效样本、七个从臂状态始终为 `0`，且
夹爪仍为未验证。

- [ ] **步骤 6：逐关节方向验收**

在操作员现场配合下，按 `joint1` 到 `joint6` 顺序逐个运行
`calibrate-directions`。每次只移动当前指定关节；记录引导臂和从臂的前后原始值、推断
符号和操作员确认。若任一关节隔离检查失败，则停止该关节验收，不进入真机跟随。

- [ ] **步骤 7：提交文档和真机入口**

```bash
git add Python_SDK/rebot_b601_mapping/README.md \
  Python_SDK/rebot_b601_mapping/tests/hardware_snapshot_smoke.py
git commit -m "docs: 添加只读映射测试说明"
```

## 完成判据

- 自动化测试、编译和静态禁止调用审计全部通过。
- 真机 `snapshot` 连续采样不少于 20 次，所有从臂状态均为 `0`。
- J1～J6 均有独立、可追溯的原始增量和人工确认；夹爪仍未验证。
- 所有提交仅存在于 `/home/a/project/Star-Arm-102-sdk-test` 的
  `codex/python-sdk-mapping-audit` 分支。
- 未修改、暂存或提交 `/home/a/project/rebot_Arm` 的任何文件。
- 完成该计划只证明数据采集和方向映射，不授权发送真机跟随控制指令。
