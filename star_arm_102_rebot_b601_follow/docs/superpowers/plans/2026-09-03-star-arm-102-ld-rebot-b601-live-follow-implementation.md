# Star Arm 102-LD → reBot B601 Python SDK 实时主从跟随 Implementation Plan

> 2026-09-04 更新：启动阶段的寄存器写入与 `prepare_pos_vel()` 已由
> `2026-09-04-follow-read-only-controller-precheck.md` 取代；以下 Task 4 以更新后的
> 只读校验契约为准。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在独立 Python SDK 工具中实现 Star Arm 102-LD 引导臂到 reBot B601-DM 从臂的 J1～J6、50 Hz、相对基线实时跟随，以及可验证的保持、回安全位和失能流程。

**Architecture:** FashionStar 采样线程只发布最新完整引导臂快照；唯一的从臂控制循环独占 `motorbridge` 串口，以从臂启动反馈为基线进行映射、目标整形、命令发送和反馈检查。纯函数目标整形与安全状态机和 SDK 适配层分离，所有错误显式上抛，由编排器区分可恢复保持/回位与严重保护停机。

**Tech Stack:** Python 3.12、NumPy 1.26.4、pyserial 3.5、fashionstar-uart-sdk 1.3.12、motorbridge 0.4.6、pytest 7.4.4、JSON/JSONL。

**Spec:** `docs/superpowers/specs/2026-09-03-star-arm-102-ld-rebot-b601-live-follow-design.md`

## Global Constraints

- 只修改 `/home/a/project/Star-Arm-102-sdk-test`；`/home/a/project/rebot_Arm` 只允许读取，不修改、不提交。
- 不使用 ROS 2 控制真实从臂；运行前必须正常停止占用 `/dev/ttyACM0` 的 ROS 控制器，程序不得自动抢占或终止占用者。
- 方向和比例固定为 `[-1,-1,+1,+1,+1,-1]` 与 `[1,1,1,1,1,1]`，坐标以从臂为准；绝不直接复制引导臂绝对角度。
- 启动不自动回安全位；首个从臂命令必须等于启动反馈 `qF0`，最终引导臂基线 `qL0` 在从臂稳定使能保持后采集。
- 默认速度 `0.5 rad/s`，硬上限 `1.5 rad/s`，最大加速度 `5.0 rad/s²`，最大加加速度 `20.0 rad/s³`。
- J1～J6 直接使用网页遥操作的真实关节限位，不另设位置余量或相对启动基线最大偏移。
- 跟踪误差 `0.25 rad` 持续 `0.30 s`、引导臂超时 `0.5 s`、从臂反馈严重超时 `0.25 s`。
- `safe_home` 对齐网页遥操作的操作员记录姿态 `[-1.549363136291504,0.01659393310546875,-0.02002716064453125,-0.00858306884765625,0.10395240783691406,0.00133514404296875]`；到位误差 `<0.02 rad`、速度 `<0.05 rad/s` 并稳定 `1.0 s`，回位超时 `30.0 s`。
- 可恢复异常必须先保持，再受控返回 `safe_home`；健康状态下回位失败必须保持使能并进入人工恢复，绝不能落入通用自动失能。
- 只有从臂严重故障、反馈/通信失效或明确紧急停止才允许离开 `safe_home` 请求保护性失能；失能结果不能验证时必须报告“未知”。
- 夹爪只读且保持启动状态，任何新代码都不得向夹爪发送模式、使能、失能或运动命令。
- 普通 `pytest` 不得打开真实串口；真机跟随不安排逐关节最小运动测试，静态门禁通过后直接进行 J1～J6 全轴实时跟随。
- 每个实现任务遵循测试先行；不得用吞掉 `CallError`、空 `except` 或通用 `finally: disable` 隐藏失败。

## File Structure

### 新建文件

- `Python_SDK/rebot_b601_mapping/hardware_specs.py`：统一保存七个电机的 ID、型号、六轴/夹爪划分和六轴 POS_VEL 增益。
- `Python_SDK/rebot_b601_mapping/live_config.py`：实时跟随配置数据类、JSON 加载、映射验收和所有数值边界校验。
- `Python_SDK/rebot_b601_mapping/live_follow.example.json`：规格批准的默认参数和 `rviz_visual_accepted` 验收级别。
- `Python_SDK/rebot_b601_mapping/command_shaper.py`：原始目标边界验证与速度/加速度/加加速度连续整形。
- `Python_SDK/rebot_b601_mapping/safety_supervisor.py`：运行监视器、安全事件、状态和动作转换。
- `Python_SDK/rebot_b601_mapping/follower_controller.py`：低层 `motorbridge` 六轴生命周期与周期命令/反馈适配器。
- `Python_SDK/rebot_b601_mapping/live_follow.py`：采样线程、停止令牌、JSONL 记录、跟随和回位编排。
- `Python_SDK/rebot_b601_mapping/tests/test_live_config.py`
- `Python_SDK/rebot_b601_mapping/tests/test_command_shaper.py`
- `Python_SDK/rebot_b601_mapping/tests/test_safety_supervisor.py`
- `Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py`
- `Python_SDK/rebot_b601_mapping/tests/test_live_follow.py`
- `Python_SDK/rebot_b601_mapping/tests/test_cli_follow.py`

### 修改文件

- `Python_SDK/rebot_b601_mapping/follower_reader.py`：改为导入共享硬件规格，保持只读 API 和行为不变。
- `Python_SDK/rebot_b601_mapping/cli.py`：新增 `follow` 子命令和真机专用错误/信号语义，不改变三个只读子命令。
- `Python_SDK/rebot_b601_mapping/README.md`：补充静态门禁、实时跟随、停止/异常语义和证据路径。

---

### Task 1: 共享硬件规格与严格实时配置

**Files:**
- Create: `Python_SDK/rebot_b601_mapping/hardware_specs.py`
- Create: `Python_SDK/rebot_b601_mapping/live_config.py`
- Create: `Python_SDK/rebot_b601_mapping/live_follow.example.json`
- Create: `Python_SDK/rebot_b601_mapping/tests/test_live_config.py`
- Modify: `Python_SDK/rebot_b601_mapping/follower_reader.py`
- Test: `Python_SDK/rebot_b601_mapping/tests/test_follower_reader.py`

**Interfaces:**
- Produces: `MotorSpec`、`MOTOR_SPECS`、`ARM_MOTOR_SPECS`、`GRIPPER_SPEC`、`POS_VEL_GAINS_BY_NAME`。
- Produces: `LiveFollowConfig`、`load_live_follow_config(path: Path) -> LiveFollowConfig`、`validate_live_mapping(mapping: MappingConfig, live: LiveFollowConfig) -> None`。
- Preserves: `FollowerReader` 的构造函数、`open()`、`read_sample()`、`close()` 以及七电机只读顺序。

- [ ] **Step 1: 写共享规格和配置加载的失败测试**

在 `test_live_config.py` 写明批准值和拒绝条件：

```python
from pathlib import Path
import json
import pytest

from rebot_b601_mapping.live_config import (
    load_live_follow_config,
    validate_live_mapping,
)
from rebot_b601_mapping.models import load_mapping_config

ROOT = Path(__file__).parents[1]
LIVE = ROOT / "live_follow.example.json"
MAPPING = ROOT / "mapping.example.json"


def test_example_config_matches_approved_live_limits():
    config = load_live_follow_config(LIVE)
    assert config.control_rate_hz == 50.0
    assert config.default_speed_rad_s == 0.5
    assert config.max_speed_rad_s == 1.5
    assert config.max_acceleration_rad_s2 == 5.0
    assert config.max_jerk_rad_s3 == 20.0
    assert config.safe_home_rad == pytest.approx((-1.549363136291504, 0.01659393310546875, -0.02002716064453125, -0.00858306884765625, 0.10395240783691406, 0.00133514404296875))
    validate_live_mapping(load_mapping_config(MAPPING), config)


@pytest.mark.parametrize(
    ("field", "value"),
    [("default_speed_rad_s", 1.6), ("leader_stale_timeout_s", 0.0), ("mapping_acceptance", "hardware_verified")],
)
def test_live_config_rejects_unapproved_values(tmp_path, field, value):
    data = json.loads(LIVE.read_text(encoding="utf-8"))
    data[field] = value
    path = tmp_path / "live.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError):
        load_live_follow_config(path)
```

在 `test_follower_reader.py` 增加断言，证明共享规格重构后注册顺序仍是 ID `0x01..0x07`。

- [ ] **Step 2: 运行测试并确认因模块不存在而失败**

Run:

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_live_config.py \
  Python_SDK/rebot_b601_mapping/tests/test_follower_reader.py -q
```

Expected: FAIL，首个错误包含 `ModuleNotFoundError: rebot_b601_mapping.live_config`。

- [ ] **Step 3: 实现共享规格和配置类型**

`hardware_specs.py` 提供不可变数据，不导入 `motorbridge`：

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class MotorSpec:
    name: str
    motor_id: int
    feedback_id: int
    model: str

@dataclass(frozen=True)
class PosVelGains:
    vel_kp: float
    vel_ki: float
    pos_kp: float
    pos_ki: float

MOTOR_SPECS = (
    MotorSpec("joint1", 0x01, 0x11, "4340P"),
    MotorSpec("joint2", 0x02, 0x12, "4340P"),
    MotorSpec("joint3", 0x03, 0x13, "4340P"),
    MotorSpec("joint4", 0x04, 0x14, "4310"),
    MotorSpec("joint5", 0x05, 0x15, "4310"),
    MotorSpec("joint6", 0x06, 0x16, "4310"),
    MotorSpec("gripper", 0x07, 0x17, "4310"),
)
ARM_MOTOR_SPECS = MOTOR_SPECS[:6]
GRIPPER_SPEC = MOTOR_SPECS[6]
POS_VEL_GAINS_BY_NAME = {
    **{name: PosVelGains(0.0125, 0.004, 150.0, 0.5) for name in ("joint1", "joint2", "joint3")},
    **{name: PosVelGains(0.0008, 0.002, 50.0, 1.0) for name in ("joint4", "joint5", "joint6")},
}
```

`live_config.py` 定义以下完整字段，加载时要求 `schema_version == 1`、所有数值有限且为正、
`default_speed_rad_s <= max_speed_rad_s == 1.5`、六轴 `safe_home` 有限、
`mapping_acceptance == "rviz_visual_accepted"`，并要求映射符号和比例精确匹配规格：

```python
@dataclass(frozen=True)
class LiveFollowConfig:
    mapping_acceptance: str
    control_rate_hz: float
    default_speed_rad_s: float
    max_speed_rad_s: float
    max_acceleration_rad_s2: float
    max_jerk_rad_s3: float
    max_tracking_error_rad: float
    tracking_error_grace_s: float
    leader_stale_timeout_s: float
    follower_stale_timeout_s: float
    safe_home_rad: tuple[float, ...]
    safe_home_tolerance_rad: float
    safe_home_velocity_tolerance_rad_s: float
    safe_home_stable_s: float
    safe_home_timeout_s: float
    deadline_miss_limit: int

    @property
    def follow_ready_rad(self) -> tuple[float, ...]:
        return self.safe_home_rad
```

`live_follow.example.json` 使用规格中的精确值，并将 `deadline_miss_limit` 设为 `3`。
`follower_reader.py` 删除本地 `_MotorSpec/MOTOR_SPECS`，改为从 `hardware_specs` 导入。

- [ ] **Step 4: 运行目标测试并确认通过**

Run: 使用 Step 2 的相同命令。

Expected: PASS，且 `test_follower_reader.py` 仍证明只读路径没有生命周期或运动调用。

- [ ] **Step 5: 提交配置和共享规格**

```bash
git add Python_SDK/rebot_b601_mapping/hardware_specs.py \
  Python_SDK/rebot_b601_mapping/live_config.py \
  Python_SDK/rebot_b601_mapping/live_follow.example.json \
  Python_SDK/rebot_b601_mapping/follower_reader.py \
  Python_SDK/rebot_b601_mapping/tests/test_live_config.py \
  Python_SDK/rebot_b601_mapping/tests/test_follower_reader.py
git commit -m "feat: 添加实时跟随安全配置"
```

### Task 2: 六轴原始目标验证与运动整形

**Files:**
- Create: `Python_SDK/rebot_b601_mapping/command_shaper.py`
- Create: `Python_SDK/rebot_b601_mapping/tests/test_command_shaper.py`

**Interfaces:**
- Consumes: `LiveFollowConfig`、六轴 `(lower, upper)`、启动基线 `qF0`。
- Produces: `TargetViolation(ValueError)`。
- Produces: `ShapedCommand(position_rad: tuple[float, ...], velocity_rad_s: tuple[float, ...], acceleration_rad_s2: tuple[float, ...])`。
- Produces: `CommandShaper.reset(position_rad)`、`CommandShaper.step(raw_target_rad, dt_s) -> ShapedCommand`。

- [ ] **Step 1: 写边界和三阶约束的失败测试**

```python
import numpy as np
import pytest
from rebot_b601_mapping.command_shaper import CommandShaper, TargetViolation

LIMITS = ((-2.8, 2.8), (-3.14, 0.02), (-3.14, 0.0), (-1.87, 1.57), (-1.57, 1.57), (-3.14, 3.14))

def make_shaper(baseline=(0.0, -1.0, -1.0, 0.0, 0.0, 0.0)):
    return CommandShaper(
        joint_limits=LIMITS, baseline_rad=baseline,
        max_speed_rad_s=0.5,
        max_acceleration_rad_s2=5.0, max_jerk_rad_s3=20.0,
    )

def test_first_step_starts_exactly_at_follower_baseline():
    shaper = make_shaper()
    shaper.reset((0.0, -1.0, -1.0, 0.0, 0.0, 0.0))
    first = shaper.step((0.0, -1.0, -1.0, 0.0, 0.0, 0.0), 0.02)
    assert first.position_rad == pytest.approx((0.0, -1.0, -1.0, 0.0, 0.0, 0.0))

def test_sequence_respects_speed_acceleration_and_jerk():
    shaper = make_shaper()
    shaper.reset((0.0, -1.0, -1.0, 0.0, 0.0, 0.0))
    samples = [shaper.step((1.0, -1.0, -1.0, 0.0, 0.0, 0.0), 0.02) for _ in range(100)]
    assert max(abs(s.velocity_rad_s[0]) for s in samples) <= 0.5 + 1e-9
    assert max(abs(s.acceleration_rad_s2[0]) for s in samples) <= 5.0 + 1e-9
    jerks = np.diff([s.acceleration_rad_s2[0] for s in samples]) / 0.02
    assert np.max(np.abs(jerks)) <= 20.0 + 1e-8

def test_targets_at_web_joint_limits_are_allowed_but_beyond_are_rejected():
    shaper = make_shaper(baseline=(0.0, 0.014, -0.020027, 0.0, 0.0, 0.0))
    shaper.reset((0.0, 0.014, -0.020027, 0.0, 0.0, 0.0))
    shaper.step((0.0, 0.02, 0.0, 0.0, 0.0, 0.0), 0.02)
    with pytest.raises(TargetViolation, match="joint2"):
        shaper.step((0.0, 0.021, 0.0, 0.0, 0.0, 0.0), 0.02)
```

再覆盖非有限值、向量长度错误、`dt <= 0`、相对偏移 `>1.5 rad` 和真实关节限位越界。

- [ ] **Step 2: 运行测试并确认失败**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_command_shaper.py -q
```

Expected: FAIL，包含 `ModuleNotFoundError: rebot_b601_mapping.command_shaper`。

- [ ] **Step 3: 实现确定性的离散目标整形器**

实现前先验证六轴维度和有限值。每一轴直接使用与网页遥操作一致的真实范围
`[lower, upper]`，边界端点可用，范围外原始目标在整形和发送前拒绝。每周期按以下顺序更新：

```python
error = raw - position
desired_velocity = np.clip(error / dt, -max_speed, max_speed)
desired_acceleration = np.clip((desired_velocity - velocity) / dt, -max_acceleration, max_acceleration)
acceleration += np.clip(
    desired_acceleration - acceleration,
    -max_jerk * dt,
    max_jerk * dt,
)
velocity = np.clip(velocity + acceleration * dt, -max_speed, max_speed)
position = position + velocity * dt
```

接近目标时使用制动距离 `v²/(2*a_max)` 降低 `desired_velocity`，不得通过直接把速度或
加速度清零来违反有限差分约束。测试应驱动实现直到目标稳定，而不是依赖固定周期后恰好
到达。每个返回对象都使用不可变六元素元组。

- [ ] **Step 4: 运行目标测试并确认通过**

Run: 使用 Step 2 的相同命令。

Expected: PASS，包括所有有限差分限制和边界方向测试。

- [ ] **Step 5: 提交目标整形器**

```bash
git add Python_SDK/rebot_b601_mapping/command_shaper.py \
  Python_SDK/rebot_b601_mapping/tests/test_command_shaper.py
git commit -m "feat: 添加六轴实时目标整形"
```

### Task 3: 安全监视器与显式状态机

**Files:**
- Create: `Python_SDK/rebot_b601_mapping/safety_supervisor.py`
- Create: `Python_SDK/rebot_b601_mapping/tests/test_safety_supervisor.py`

**Interfaces:**
- Produces: `FollowState`：`DISCONNECTED/PRECHECK/ENABLED_HOLD/FOLLOWING/RECOVERABLE_HOLD/RETURNING_SAFE_HOME/VERIFYING_SAFE_HOME/DISABLING/OPERATOR_RECOVERY/CRITICAL_STOP`。
- Produces: `SafetyEvent`：`PRECHECK_OK/ENABLE_OK/FOLLOW_START/RECOVERABLE_FAULT/HOLD_CONFIRMED/SAFE_HOME_REACHED/SAFE_HOME_FAILED_HEALTHY/CRITICAL_FAULT/DISABLE_OK`。
- Produces: `SafetyAction`：`START_HOLD/START_RETURN/KEEP_ENABLED_HOLD/REQUEST_PROTECTIVE_DISABLE/CLOSE_HANDLES`。
- Produces: `SafetySupervisor(initial_state: FollowState = FollowState.DISCONNECTED)` 和 `SafetySupervisor.transition(event, reason="") -> Transition`。
- Produces: `RuntimeGuard.observe(observation: RuntimeObservation) -> GuardFault | None`。

- [ ] **Step 1: 写恢复路径和严重路径的失败测试**

```python
from rebot_b601_mapping.safety_supervisor import (
    FollowState, SafetyAction, SafetyEvent, SafetySupervisor,
    RuntimeGuard, RuntimeObservation, FaultClass,
)

def test_recoverable_fault_never_disables_before_verified_safe_home():
    supervisor = SafetySupervisor(FollowState.FOLLOWING)
    first = supervisor.transition(SafetyEvent.RECOVERABLE_FAULT, "leader stale")
    assert first.state is FollowState.RECOVERABLE_HOLD
    assert first.actions == (SafetyAction.START_HOLD,)
    second = supervisor.transition(SafetyEvent.HOLD_CONFIRMED)
    assert second.actions == (SafetyAction.START_RETURN,)
    assert SafetyAction.REQUEST_PROTECTIVE_DISABLE not in first.actions + second.actions

def test_failed_return_while_healthy_requires_enabled_operator_recovery():
    supervisor = SafetySupervisor(FollowState.RETURNING_SAFE_HOME)
    transition = supervisor.transition(SafetyEvent.SAFE_HOME_FAILED_HEALTHY, "timeout")
    assert transition.state is FollowState.OPERATOR_RECOVERY
    assert transition.actions == (SafetyAction.KEEP_ENABLED_HOLD,)

def test_only_critical_fault_requests_disable_away_from_home():
    supervisor = SafetySupervisor(FollowState.FOLLOWING)
    transition = supervisor.transition(SafetyEvent.CRITICAL_FAULT, "feedback stale")
    assert transition.state is FollowState.CRITICAL_STOP
    assert transition.actions == (SafetyAction.START_HOLD, SafetyAction.REQUEST_PROTECTIVE_DISABLE)
```

为 `RuntimeGuard` 增加三类精确测试：跟踪误差只在连续 `0.30 s` 后成为可恢复故障；连续
3 次 deadline miss 成为可恢复故障；从臂反馈年龄 `>0.25 s`、状态码非 1、命令写失败
或端口身份变化立即成为严重故障。引导臂年龄 `>0.5 s` 为可恢复故障。

- [ ] **Step 2: 运行测试并确认失败**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_safety_supervisor.py -q
```

Expected: FAIL，包含缺少 `safety_supervisor` 模块。

- [ ] **Step 3: 实现表驱动状态转换和有状态运行监视器**

用不可变类型固定输出：

```python
@dataclass(frozen=True)
class Transition:
    state: FollowState
    actions: tuple[SafetyAction, ...]
    reason: str

@dataclass(frozen=True)
class RuntimeObservation:
    now_s: float
    leader_age_s: float
    follower_age_s: float
    tracking_error_rad: tuple[float, ...]
    status_codes: tuple[int, ...]
    deadline_missed: bool
    command_write_ok: bool
    port_identity_ok: bool

@dataclass(frozen=True)
class GuardFault:
    fault_class: FaultClass
    reason: str
```

非法状态转换必须抛 `RuntimeError`，而不是静默忽略。`RuntimeGuard` 保存首次连续越差时间
和连续 deadline miss 计数；任何一次恢复正常都清零对应连续计数。将状态转换表写成模块
常量，使测试可穷举检查：任何可恢复路径在 `SAFE_HOME_REACHED` 前都不得包含
`REQUEST_PROTECTIVE_DISABLE`。

- [ ] **Step 4: 运行目标测试并确认通过**

Run: 使用 Step 2 的相同命令。

Expected: PASS。

- [ ] **Step 5: 提交安全状态机**

```bash
git add Python_SDK/rebot_b601_mapping/safety_supervisor.py \
  Python_SDK/rebot_b601_mapping/tests/test_safety_supervisor.py
git commit -m "feat: 添加实时跟随安全状态机"
```

### Task 4: 可审计的 motorbridge 六轴控制器

**Files:**
- Create: `Python_SDK/rebot_b601_mapping/follower_controller.py`
- Create: `Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py`

**Interfaces:**
- Consumes: `ARM_MOTOR_SPECS`、`GRIPPER_SPEC`、`POS_VEL_GAINS_BY_NAME`。
- Produces: `FollowerArmState(timestamp_s, positions_rad, velocities_rad_s, torques_nm, status_codes, gripper_position_rad, gripper_velocity_rad_s, gripper_torque_nm, gripper_status_code)`；机械臂向量长度固定为 6，夹爪字段单独保存。
- Produces: `FollowerController(port: str, *, config: LiveFollowConfig, joint_limits, controller_factory=_make_controller, clock=time.monotonic, sleep=time.sleep, identity_factory=PortIdentity.capture, identity_checker=assert_same_port)`；`joint_limits` 直接来自映射配置中的网页六轴边界。
- Produces: `FollowerController.open()`、`read_state(expected_arm_status=None, expected_gripper_status=0)`、`verify_pos_vel_configuration(timeout_ms=250)`、`enable_hold(speed_rad_s: float) -> EnableHoldResult`、`cycle(target_rad, speed_rad_s) -> FollowerArmState`、`hold_current(speed_rad_s: float, *, fallback_target_rad=None) -> HoldResult`、`disable_verified()`、`close()`；使能和保持结果都区分反馈状态与实际发送命令。
- Produces: `FollowerCommunicationError` 和 `FollowerLifecycleError`；所有 SDK 原始异常通过 `raise FollowerCommunicationError(message) from exc` 或 `raise FollowerLifecycleError(message) from exc` 保留。

- [ ] **Step 1: 写伪电机生命周期失败测试**

在测试中建立记录所有方法调用的 `FakeController/FakeMotor`。至少写以下断言：

```python
import pytest
from rebot_b601_mapping.hardware_specs import POS_VEL_GAINS_BY_NAME

expected = POS_VEL_GAINS_BY_NAME

# make_follower() 创建 FakeController，并通过 controller_factory、伪时钟和
# 伪设备身份将它注入 FollowerController；可选参数控制六轴初始位置或指定轴使能失败。
def make_follower(*, positions=(0.0, -1.0, -1.0, 0.0, 0.0, 0.0), enable_failure_at=None):
    controller = FakeController(positions=positions, enable_failure_at=enable_failure_at)
    follower = FollowerController(
        "/dev/fake-follower",
        config=load_live_follow_config(LIVE_CONFIG),
        joint_limits=JOINT_LIMITS,
        controller_factory=lambda port, baudrate: controller,
        clock=FakeClock(),
        sleep=lambda seconds: None,
        identity_factory=lambda path: PortIdentity(path, path, 166),
        identity_checker=lambda identity: None,
    )
    return controller, follower

def test_verify_pos_vel_configuration_reads_without_writes_or_mode_changes():
    controller, follower = make_follower()
    follower.open()
    follower.verify_pos_vel_configuration()
    for motor in controller.motors[:6]:
        assert motor.register_writes == []
        assert motor.ensure_modes == []
    assert controller.motors[6].lifecycle_and_command_calls == []

def test_enable_first_target_is_exact_current_feedback_and_gripper_stays_disabled():
    controller, follower = make_follower(positions=(0.1, -1.0, -1.1, 0.2, 0.0, 0.3))
    follower.open()
    follower.verify_pos_vel_configuration()
    state = follower.enable_hold(0.5)
    assert [motor.pos_vel_calls[0][0] for motor in controller.motors[:6]] == pytest.approx(state.positions_rad)
    assert controller.motors[6].enable_calls == 0

def test_partial_enable_failure_rolls_back_and_verifies_all_six_disabled():
    controller, follower = make_follower(enable_failure_at="joint4")
    follower.open()
    follower.verify_pos_vel_configuration()
    with pytest.raises(FollowerLifecycleError, match="joint4"):
        follower.enable_hold(0.5)
    assert [motor.state.status_code for motor in controller.motors[:6]] == [0] * 6
```

再测试：任一寄存器读取失败、模式错误或增益无效时没有电机被使能；合法持久增益无需与
YAML 参考值精确相等；任一六轴 `send_pos_vel` 失败立即显式抛错；
全六轴反馈事务超过 `0.25 s` 或返回非有限值显式抛错；`disable_verified()` 逐轴失能并要求
六轴状态 0；`close()` 只关闭句柄、不调用 `disable_all()`、`shutdown()` 或夹爪失能。

- [ ] **Step 2: 运行测试并确认失败**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py -q
```

Expected: FAIL，包含缺少 `follower_controller` 模块。

- [ ] **Step 3: 实现六轴控制器且保持单一总线所有者**

`open()` 注册七个电机以读取夹爪状态，但控制方法只遍历前六个。
`verify_pos_vel_configuration()` 在六轴失能时只读核对 RID 10 与 RID 25～28；不写寄存器，
不调用 `ensure_mode()`。RID 10 必须是 POS_VEL，RID 25～28 必须有限且非负，但无需与
主项目 YAML 参考值精确相等；读取失败、模式错误或增益无效时在使能前失败。
`enable_hold(speed_rad_s)` 在使能前读取 `qF0`。然后逐轴执行 `enable()`，每一轴一旦确认
`status_code=1` 就立即对该轴发送首个 `send_pos_vel(qF0[i], speed_rad_s)`，不能等六轴全部
使能后才首次发送保持目标。完成后验证六轴状态均为 1、夹爪状态仍为 0，并返回最新反馈。
不要调用 `Controller.enable_all()`。

反馈事务使用同一个控制器锁，逐电机 `request_feedback()` 并轮询到七个新状态或绝对截止
时间。`FollowerArmState.timestamp_s` 是整组反馈完成时的单调时间。`cycle()` 先验证六轴
向量和速度范围，依次发送六个 `send_pos_vel()`，随后完成一组反馈；任何写入失败都包含
关节名并抛 `FollowerCommunicationError`。

`hold_current(speed_rad_s)` 先取得新鲜反馈，再对该反馈调用 `cycle()`。若反馈本身不可用，则由上层
改用最后成功命令；本方法不得伪造当前位置。`close()` 只关闭七个 motor 句柄和 controller
句柄，并聚合关闭错误。

- [ ] **Step 4: 运行目标测试和原只读适配器测试**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py \
  Python_SDK/rebot_b601_mapping/tests/test_follower_reader.py -q
```

Expected: PASS，且只读适配器测试仍禁止所有生命周期调用。

- [ ] **Step 5: 提交从臂控制器**

```bash
git add Python_SDK/rebot_b601_mapping/follower_controller.py \
  Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py
git commit -m "feat: 添加从臂六轴SDK控制器"
```

### Task 5: 实时跟随编排、回位和 JSONL 证据

**Files:**
- Create: `Python_SDK/rebot_b601_mapping/live_follow.py`
- Create: `Python_SDK/rebot_b601_mapping/tests/test_live_follow.py`

**Interfaces:**
- Consumes: `LeaderReader`、`FollowerController`、`CommandShaper`、`RuntimeGuard`、`SafetySupervisor`。
- Produces: `StopRequest`，其中 `request_normal()` 设置正常停止、`request_emergency()` 设置紧急停止，并提供只读属性 `normal_requested`、`emergency_requested` 和 `requested`。
- Produces: `LiveRunSummary(final_state, cycles, safe_home_verified, disable_verified, disable_result_known, log_path)`。
- Produces: `poll_recovery_decision(timeout_s: float) -> str | None`，使用 `select.select()` 非阻塞读取 `retry` 或 `emergency_stop`。
- Produces: `run_live_follow(*, leader_port, follower_port, mapping_path, live_config_path, log_path, confirmed, speed_rad_s=None, max_cycles=None, leader_factory=LeaderReader, follower_factory=FollowerController, port_checker=assert_ports_unoccupied, clock=time.monotonic, sleep=time.sleep, stop_request=None, recovery_decider=poll_recovery_decision) -> LiveRunSummary`。
- Factory contract: `leader_factory(port) -> LeaderReaderLike`；`follower_factory(port, config=live_config) -> FollowerControllerLike`。

- [ ] **Step 1: 写编排失败测试，先固定安全调用顺序**

使用带事件列表的伪引导臂和伪从臂。测试必须包含：

- `make_fakes(**scenario)` 返回共享 `events: list[str]`、`FakeLeader` 和 `FakeFollower`；
  `FakeFollower` 实现 Task 4 的公开方法并记录每个目标。
- `run_fake(tmp_path, *, leader, follower, confirmed=True)` 用伪端口检查器、确定性伪时钟、
  无阻塞 sleep 和临时配置调用 `run_live_follow()`。
- `ordered(events, *names)` 仅在全部名称存在且索引严格递增时返回 True。

```python
import pytest

def test_startup_holds_qf0_then_captures_fresh_leader_baseline(tmp_path):
    events, leader, follower = make_fakes(stop_after_cycles=2)
    summary = run_fake(tmp_path, leader=leader, follower=follower)
    assert events.index("follower:cycle-qf0") < events.index("leader:capture-qL0")
    assert follower.sent_targets[0] == pytest.approx(follower.start_positions)
    assert summary.cycles == 2

def test_without_confirmation_only_reads_preflight_and_never_enables(tmp_path):
    events, leader, follower = make_fakes(stop_after_cycles=0)
    summary = run_fake(tmp_path, leader=leader, follower=follower, confirmed=False)
    assert "follower:read-disabled" in events
    assert "follower:prepare-pos-vel" not in events
    assert "follower:enable" not in events
    assert summary.final_state.name == "DISCONNECTED"

def test_leader_timeout_holds_returns_home_verifies_then_disables(tmp_path):
    events, leader, follower = make_fakes(leader_timeout=True)
    summary = run_fake(tmp_path, leader=leader, follower=follower)
    assert ordered(events, "hold", "return", "home-stable", "disable", "close")
    assert summary.safe_home_verified is True
    assert summary.disable_verified is True

def test_return_failure_while_healthy_never_disables_or_closes(tmp_path):
    events, leader, follower = make_fakes(return_timeout=True, recovery_decisions=["emergency_stop"])
    summary = run_fake(tmp_path, leader=leader, follower=follower)
    assert "operator-recovery-enabled-hold" in events
    assert events.index("operator-recovery-enabled-hold") < events.index("protective-disable")
    assert "normal-disable" not in events

def test_unreachable_follower_reports_unknown_disable_result(tmp_path):
    events, leader, follower = make_fakes(feedback_stale=True, disable_unreachable=True)
    summary = run_fake(tmp_path, leader=leader, follower=follower)
    assert summary.final_state.name == "CRITICAL_STOP"
    assert summary.disable_result_known is False
```

再覆盖：映射符号对六轴实际目标生效；夹爪未出现在命令向量；`max_cycles` 仅用于测试并走
正常停止；跟踪误差持续阈值；连续三次 deadline miss；第一次正常停止和紧急停止的不同
路径；每条 JSONL 可被 `json.loads()` 读取且含规格要求的全部字段。

- [ ] **Step 2: 运行测试并确认失败**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_live_follow.py -q
```

Expected: FAIL，包含缺少 `live_follow` 模块。

- [ ] **Step 3: 实现最新样本线程和绝对截止时间循环**

`LatestLeaderSampler` 在线程内独占 `LeaderReader`，只保存最新不可变 `LeaderSample`、最后
错误和更新时间；停止时只关闭引导臂串口。最终 `qL0` 必须在从臂 `enable_hold()` 已稳定后
通过连续 5 个有限样本中位数采集。

控制循环使用：

```python
period = 1.0 / config.control_rate_hz
previous_tick = clock()
deadline = previous_tick + period
while not stop_request.requested:
    now = clock()
    dt = now - previous_tick
    raw = map_virtual_follower(latest_leader, baseline, mapping).positions_rad
    shaped = follow_shaper.step(raw, dt)
    follower_state = follower.cycle(shaped.position_rad, speed)
    fault = guard.observe(RuntimeObservation(
        now_s=now,
        leader_age_s=now - latest_leader.timestamp_s,
        follower_age_s=now - follower_state.timestamp_s,
        tracking_error_rad=tuple(
            abs(actual - commanded)
            for actual, commanded in zip(
                follower_state.positions_rad,
                shaped.position_rad,
                strict=True,
            )
        ),
        status_codes=follower_state.status_codes,
        deadline_missed=clock() > deadline,
        command_write_ok=True,
        port_identity_ok=True,
    ))
    deadline += period
    sleep(max(0.0, deadline - clock()))
```

所有循环事件通过一个 `write_jsonl(record)` 辅助函数立即写入且 `flush()`，但终端摘要最多
5 Hz。可恢复事件必须执行 `hold_current()`；若它失败，使用最后成功的 `q_cmd` 再发一次
保持。确认反馈和状态健康后，从保持位置生成同步五次多项式轨迹向网页 `safe_home` 运行；
该固定回位终点与普通跟随目标使用同一组网页真实关节限位，直到误差
`<0.02`、速度 `<0.05` 连续 `1.0 s` 或到达 `30.0 s` 超时。回位起点或逐周期反馈一旦
越过该闭区间，禁止判定到位和自动失能，转入保持使能的人工恢复流程。
保持阶段如果反馈已经越界，不把越界反馈原样回写，而是重复最后一次已发送且经同一边界
验证的命令；使能接口也必须区分实际发送的启动命令与发送后的反馈，避免使能后漂移反馈
覆盖该安全命令。最终硬件写入口必须在使能保持和每周期命令前执行该检查；正常跟随每周期
反馈越界时，同样记录原始原因并进入最后安全命令保持。

回位超时且状态健康时进入人工恢复循环。该循环每个 50 Hz 周期继续发送保持目标、刷新
反馈并调用 `recovery_decider(period)`；没有输入时返回 `None`，输入只接受 `retry` 或
`emergency_stop`。因此等待人工选择不会阻塞保持和健康监控。不要在 `finally` 中失能；
`finally` 只在已经验证正常失能，或严重路径已经记录失能结果后关闭句柄。

- [ ] **Step 4: 运行目标测试并确认通过**

Run: 使用 Step 2 的相同命令。

Expected: PASS，事件顺序断言证明可恢复异常不会直接失能。

- [ ] **Step 5: 提交实时编排器**

```bash
git add Python_SDK/rebot_b601_mapping/live_follow.py \
  Python_SDK/rebot_b601_mapping/tests/test_live_follow.py
git commit -m "feat: 添加六轴实时跟随编排"
```

### Task 6: CLI、信号处理和操作文档

**Files:**
- Modify: `Python_SDK/rebot_b601_mapping/cli.py`
- Modify: `Python_SDK/rebot_b601_mapping/README.md`
- Create: `Python_SDK/rebot_b601_mapping/tests/test_cli_follow.py`

**Interfaces:**
- Consumes: `run_live_follow()` 和 `StopRequest`。
- Produces CLI: `python -m rebot_b601_mapping.cli follow ...`。
- Produces: `_make_sigint_handler(stop_request: StopRequest)`；第一次调用 `request_normal()`，第二次调用 `request_emergency()`。
- Preserves CLI: `snapshot`、`calibrate-directions`、`rviz-preview` 的参数、退出码和只读措辞。

- [ ] **Step 1: 写 CLI 失败测试**

```python
from pathlib import Path

from rebot_b601_mapping.live_follow import LiveRunSummary, StopRequest
from rebot_b601_mapping.safety_supervisor import FollowState

MAPPING = Path(__file__).parents[1] / "mapping.example.json"
LIVE = Path(__file__).parents[1] / "live_follow.example.json"


def fake_summary(log_path):
    return LiveRunSummary(
        final_state=FollowState.DISCONNECTED,
        cycles=0,
        safe_home_verified=False,
        disable_verified=False,
        disable_result_known=True,
        log_path=Path(log_path),
    )


def test_follow_cli_passes_explicit_motion_confirmation(monkeypatch, tmp_path):
    from rebot_b601_mapping import cli
    received = {}
    monkeypatch.setattr(
        cli,
        "run_live_follow",
        lambda **kwargs: received.update(kwargs) or fake_summary(kwargs["log_path"]),
    )
    code = cli.main([
        "follow", "--leader-port", "/dev/ttyUSB0", "--follower-port", "/dev/ttyACM0",
        "--mapping-config", str(MAPPING), "--live-config", str(LIVE),
        "--log", str(tmp_path / "follow.jsonl"), "--speed-rad-s", "0.5",
        "--confirm-live-motion",
    ])
    assert code == 0
    assert received["confirmed"] is True
    assert received["speed_rad_s"] == 0.5

def test_follow_without_confirmation_runs_static_preflight_only(monkeypatch, tmp_path):
    from rebot_b601_mapping import cli
    received = {}
    monkeypatch.setattr(
        cli,
        "run_live_follow",
        lambda **kwargs: received.update(kwargs) or fake_summary(kwargs["log_path"]),
    )
    code = cli.main([
        "follow", "--mapping-config", str(MAPPING),
        "--live-config", str(LIVE),
        "--log", str(tmp_path / "preflight.jsonl"),
    ])
    assert code == 0
    assert received["confirmed"] is False

def test_second_sigint_requests_emergency_stop():
    from rebot_b601_mapping import cli
    request = StopRequest()
    handler = cli._make_sigint_handler(request)
    handler(2, None)
    assert request.normal_requested is True
    assert request.emergency_requested is False
    handler(2, None)
    assert request.normal_requested is True
    assert request.emergency_requested is True
```

信号测试直接调用回调，不使用真实 `os.kill()`。

- [ ] **Step 2: 运行 CLI 测试并确认失败**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_cli_follow.py -q
```

Expected: FAIL，因为解析器尚无 `follow` 子命令。

- [ ] **Step 3: 实现命令、退出码和简体中文文档**

新增参数：

```text
follow
  --leader-port /dev/ttyUSB0
  --follower-port /dev/ttyACM0
  --mapping-config Python_SDK/rebot_b601_mapping/mapping.example.json
  --live-config Python_SDK/rebot_b601_mapping/live_follow.example.json
  --log /tmp/stararm-rebot-live-follow.jsonl
  --speed-rad-s 0.5
  --confirm-live-motion
```

无确认标志时执行只读静态门禁并打印“未使能、未发送运动命令”。有确认标志时安装 SIGINT
处理器：第一次请求正常保持/回位，第二次请求紧急停机。退出码固定为：正常成功 `0`、
静态门禁/配置失败 `1`、人工恢复后紧急停止或严重故障 `2`、键盘紧急停止 `130`。

README 写出：端口占用检查、另一个任务正常停止 ROS 控制器的要求、静态预检命令、真机
跟随命令、映射公式、参数表、夹爪不受控、第一次/第二次 Ctrl+C 语义、回位失败保持使能
以及日志检查方法。不得把 RViz 验收写成真机方向标定。

- [ ] **Step 4: 运行 CLI、原命令和文档相关测试**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_cli_follow.py \
  Python_SDK/rebot_b601_mapping/tests/test_cli_snapshot.py \
  Python_SDK/rebot_b601_mapping/tests/test_cli_calibrate.py \
  Python_SDK/rebot_b601_mapping/tests/test_rviz_preview.py -q
```

Expected: PASS，原三个子命令没有回归。

- [ ] **Step 5: 提交 CLI 和文档**

```bash
git add Python_SDK/rebot_b601_mapping/cli.py \
  Python_SDK/rebot_b601_mapping/README.md \
  Python_SDK/rebot_b601_mapping/tests/test_cli_follow.py
git commit -m "feat: 接入实时跟随命令"
```

### Task 7: 全量软件验证和代码边界审计

**Files:**
- Verify only: `Python_SDK/rebot_b601_mapping/`

**Interfaces:**
- Consumes: Tasks 1～6 的全部代码。
- Produces: 不接触硬件的完整测试、编译和静态审计证据。

- [ ] **Step 1: 运行全量单元测试**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests -q
```

Expected: 全部 PASS；测试输出不得出现访问 `/dev/ttyUSB0` 或 `/dev/ttyACM0`。

- [ ] **Step 2: 编译全部 Python 模块**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m compileall \
  Python_SDK/rebot_b601_mapping -q
```

Expected: exit code 0。

- [ ] **Step 3: 审计危险清理和夹爪命令**

```bash
rg -n "finally:.*disable|enable_all|disable_all|shutdown\(|set_zero|gripper.*(enable|disable|send_)" \
  Python_SDK/rebot_b601_mapping
```

Expected: 只允许测试中的禁止调用断言、只读说明或明确严重异常路径；生产代码不得出现
通用 `finally: disable`、`enable_all()`、`disable_all()`、置零或夹爪控制调用。

- [ ] **Step 4: 检查差异和工作区**

```bash
git diff --check
git status --short --branch
```

Expected: `git diff --check` 无输出；工作区没有未提交的实现文件。如果审计要求修复，先加
回归测试、完成最小修复、重新运行 Steps 1～4，并单独提交 `fix: 收紧实时跟随安全边界`。

### Task 8: 静态真机门禁与全六轴实时验收

**Files:**
- Runtime evidence only: `/tmp/stararm-rebot-live-follow.jsonl`
- Do not modify: `/home/a/project/rebot_Arm`

**Interfaces:**
- Consumes: 已通过 Task 7 的 `follow` 命令和用户已经给出的真机运动授权。
- Produces: 静态门禁结果、J1～J6 实时跟随指标、正常回安全位和失能验证结果。

- [ ] **Step 1: 重新核对设备和串口占用者**

```bash
ls -l /dev/ttyUSB0 /dev/ttyACM0
fuser -v /dev/ttyUSB0 /dev/ttyACM0
```

Expected: 两个设备存在。若 `/dev/ttyACM0` 仍由另一个任务的 ROS 控制器占用，先读取进程
命令行确认身份，再向该进程发送一次 SIGINT 并等待其正常退出；不得使用 `kill -9`，也不得
停止无关进程。退出后再次运行 `fuser`，两个串口必须为空闲。

- [ ] **Step 2: 只读执行静态门禁**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m rebot_b601_mapping.cli follow \
  --leader-port /dev/ttyUSB0 \
  --follower-port /dev/ttyACM0 \
  --mapping-config Python_SDK/rebot_b601_mapping/mapping.example.json \
  --live-config Python_SDK/rebot_b601_mapping/live_follow.example.json \
  --log /tmp/stararm-rebot-live-follow-preflight.jsonl \
  --speed-rad-s 0.5
```

Expected: 输出映射、两侧反馈、从臂六轴与夹爪 `status_code=0`、实际阈值，并明确“未使能、
未发送运动命令”。任一条件失败都停止，不进入真机跟随。

- [ ] **Step 3: 启动 J1～J6 全轴实时跟随**

确认工作空间无障碍物、从臂有机械支撑且操作员可触及急停后运行：

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

Expected: 从臂先在当前 `qF0` 保持，无启动跳变；随后直接进入 J1～J6 全轴实时跟随，夹爪
不接收控制。操作员用引导臂完成一系列动作并目视确认方向和响应。

- [ ] **Step 4: 正常停止并验证安全位**

动作完成后只发送一次 Ctrl+C。程序必须依次报告：保持当前位置、受控返回
`[-1.549363136291504,0.01659393310546875,-0.02002716064453125,-0.00858306884765625,0.10395240783691406,0.00133514404296875]`、六轴误差 `<0.02 rad` 且速度 `<0.05 rad/s` 稳定
`1.0 s`、六轴失能并验证 `status_code=0`、句柄关闭。

如果回位失败但状态健康，程序必须保持使能等待人工恢复；此时不得关闭终端或假定失能。
只有操作员明确选择重试或紧急停机后才能继续。

- [ ] **Step 5: 解析日志并报告实测结果**

```bash
PYTHONPATH=Python_SDK .venv/bin/python - <<'PY'
import json
from pathlib import Path
rows = [json.loads(line) for line in Path('/tmp/stararm-rebot-live-follow.jsonl').read_text(encoding='utf-8').splitlines()]
cycles = [row for row in rows if row.get('event') == 'cycle']
periods = sorted(float(row['period_s']) for row in cycles)
def percentile(values, fraction):
    return values[min(len(values) - 1, round((len(values) - 1) * fraction))]
print({
    'cycles': len(cycles),
    'p95_period_s': percentile(periods, 0.95),
    'p99_period_s': percentile(periods, 0.99),
    'max_tracking_error_rad': max(max(row['tracking_error_rad']) for row in cycles),
    'max_leader_lag_rad': max(max(abs(v) for v in row['leader_lag_rad']) for row in cycles),
    'final_event': rows[-1]['event'],
    'safe_home_verified': rows[-1].get('safe_home_verified'),
    'disable_verified': rows[-1].get('disable_verified'),
})
PY
```

Expected: JSONL 全部可解析；报告实际循环样本数、P95/P99 周期、最大跟踪误差、最大主从
滞后、停止原因、安全位和失能验证。不得主动制造通信中断、电机故障或坠落风险来测试
严重异常；这些路径以 Task 3～5 的伪 SDK 测试作为本阶段证据。
