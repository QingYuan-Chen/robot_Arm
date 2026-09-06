# Follow Read-Only Controller Precheck Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove all startup register writes from live follow, verify the persisted POS_VEL mode and validate existing gains read-only, and start relative following from the two arms' current poses.

**Architecture:** `FollowerController` owns a new read-only configuration validator that reads Damiao RID 10 and RIDs 25–28 and never calls a write or `ensure_mode`. `run_live_follow` invokes that validator only after confirmed live motion and before `enable_hold`; the existing current-position hold and post-stop web parking return remain unchanged.

**Tech Stack:** Python 3.12, pytest, motorbridge 0.4.6 Python binding, JSONL runtime logs

**Spec:** `docs/superpowers/specs/2026-09-04-follow-read-only-controller-precheck-design.md`

## Global Constraints

- Modify only `/home/a/project/Star-Arm-102-sdk-test`.
- Do not modify `/home/a/project/rebot_Arm` or motorbridge.
- Do not operate either serial port during implementation or automated verification.
- Preserve web J1–J6 limits and the existing guarded web parking return.
- Startup must issue zero register writes and zero joint commands before explicit enable.
- Automated tests do not constitute hardware acceptance.

---

### Task 1: Read-only POS_VEL configuration validation

**Files:**
- Modify: `Python_SDK/rebot_b601_mapping/follower_controller.py:1-235`
- Test: `Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py:35-305`

**Interfaces:**
- Consumes: `ARM_MOTOR_SPECS`, motorbridge `Mode.POS_VEL`, `Motor.get_register_u32(rid, timeout_ms)`, and `Motor.get_register_f32(rid, timeout_ms)`.
- Produces: `FollowerController.verify_pos_vel_configuration(timeout_ms: int = 250) -> None`.

- [ ] **Step 1: Extend the fake motor with read-only register state and call recording**

```python
self.register_u32 = {10: 2}
gains = POS_VEL_GAINS_BY_NAME[name] if name in POS_VEL_GAINS_BY_NAME else None
self.register_f32 = {} if gains is None else {
    25: gains.vel_kp,
    26: gains.vel_ki,
    27: gains.pos_kp,
    28: gains.pos_ki,
}
self.register_reads: list[tuple[str, int, int]] = []

def get_register_u32(self, rid: int, timeout_ms: int = 1000) -> int:
    self.register_reads.append(("u32", rid, timeout_ms))
    return self.register_u32[rid]

def get_register_f32(self, rid: int, timeout_ms: int = 1000) -> float:
    self.register_reads.append(("f32", rid, timeout_ms))
    return self.register_f32[rid]
```

- [ ] **Step 2: Replace the old prepare tests with failing read-only contract tests**

```python
def test_verify_pos_vel_configuration_reads_mode_and_gains_without_writes() -> None:
    controller, follower = make_follower()
    follower.open()

    follower.verify_pos_vel_configuration()

    for motor in controller.motors[:6]:
        assert motor.register_reads == [
            ("u32", 10, 250),
            ("f32", 25, 250),
            ("f32", 26, 250),
            ("f32", 27, 250),
            ("f32", 28, 250),
        ]
        assert motor.register_writes == []
        assert motor.ensure_modes == []
```

Also add one test that changes joint3 RID 10 and expects `FollowerLifecycleError` containing `joint3` and `RID 10`, one that verifies J2 RID 25 value `0.0130000003` is accepted, parameterized tests rejecting negative/NaN/Inf gains, and one that makes a register read raise and verifies all seven motors have zero enable calls and zero position commands.

- [ ] **Step 3: Run the focused tests and verify they fail**

Run:

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py -q
```

Expected: FAIL because `verify_pos_vel_configuration` does not exist and the old tests still expect writes.

- [ ] **Step 4: Implement the minimal read-only validator and remove `prepare_pos_vel`**

```python
POS_VEL_GAIN_REGISTERS = (25, 26, 27, 28)

def verify_pos_vel_configuration(self, timeout_ms: int = 250) -> None:
    from motorbridge import Mode

    timeout = int(timeout_ms)
    if timeout <= 0:
        raise ValueError("timeout_ms 必须为正整数")
    for spec, motor in zip(ARM_MOTOR_SPECS, self._motors[:6], strict=True):
        try:
            actual_mode = int(motor.get_register_u32(10, timeout))
            expected_mode = int(Mode.POS_VEL)
            if actual_mode != expected_mode:
                raise FollowerLifecycleError(
                    f"{spec.name} RID 10 模式不符：期望 {expected_mode}，实际 {actual_mode}"
                )
            for rid in POS_VEL_GAIN_REGISTERS:
                actual = float(motor.get_register_f32(rid, timeout))
                if not math.isfinite(actual) or actual < 0.0:
                    raise FollowerLifecycleError(
                        f"{spec.name} RID {rid} 参数无效：实际 {actual:.9g}，要求有限且非负"
                    )
        except FollowerLifecycleError:
            raise
        except Exception as exc:
            raise FollowerLifecycleError(
                f"{spec.name} POS_VEL 配置只读校验失败：{exc}"
            ) from exc
```

Do not retain any call to `write_register_f32`, `write_register_u32`, or `ensure_mode` in the live-follow controller.

- [ ] **Step 5: Run the focused tests and verify they pass**

Run the command from Step 3. Expected: all follower-controller tests PASS.

- [ ] **Step 6: Commit the controller and unit-test change**

```bash
git add Python_SDK/rebot_b601_mapping/follower_controller.py \
  Python_SDK/rebot_b601_mapping/tests/test_follower_controller.py
git commit -m "fix: 跟随启动只读校验从臂配置"
```

---

### Task 2: Wire read-only validation into live-follow startup

**Files:**
- Modify: `Python_SDK/rebot_b601_mapping/live_follow.py:448-480`
- Test: `Python_SDK/rebot_b601_mapping/tests/test_live_follow.py:105-340`

**Interfaces:**
- Consumes: `FollowerController.verify_pos_vel_configuration(timeout_ms=250) -> None` from Task 1.
- Produces: confirmed startup ordering `preflight -> verify configuration -> enable current-position hold -> capture leader baseline`.

- [ ] **Step 1: Change the fake follower and add failing orchestration tests**

```python
def verify_pos_vel_configuration(self) -> None:
    self.events.append("follower:verify-pos-vel-read-only")
```

Add a failure option that raises `FollowerLifecycleError("joint3 RID 10 模式不符")`, then assert the failure path contains neither `follower:enable` nor any sent target. Update the normal startup test to assert:

```python
assert events.index("follower:read-disabled") < events.index(
    "follower:verify-pos-vel-read-only"
) < events.index("follower:enable")
assert events.index("follower:cycle-qf0") < events.index("leader:capture-qL0")
```

Keep the unconfirmed preflight test asserting that configuration verification and enable are both absent.

- [ ] **Step 2: Run the focused orchestration tests and verify they fail**

Run:

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests/test_live_follow.py -q
```

Expected: FAIL because production still calls `prepare_pos_vel()`.

- [ ] **Step 3: Replace the startup call**

```python
follower.verify_pos_vel_configuration()
enable_result = follower.enable_hold(speed)
```

Do not add any ready-pose movement. Preserve `enable_hold()` using the latest validated current feedback as its first target and preserve leader baseline capture after the hold succeeds.

- [ ] **Step 4: Run live-follow tests and verify they pass**

Run the command from Step 2. Expected: all live-follow tests PASS.

- [ ] **Step 5: Commit the orchestration change**

```bash
git add Python_SDK/rebot_b601_mapping/live_follow.py \
  Python_SDK/rebot_b601_mapping/tests/test_live_follow.py
git commit -m "fix: 移除跟随启动寄存器写入"
```

---

### Task 3: Documentation and complete offline verification

**Files:**
- Modify: `Python_SDK/rebot_b601_mapping/README.md`
- Modify: `docs/superpowers/specs/2026-09-03-star-arm-102-ld-rebot-b601-live-follow-design.md`
- Modify: `docs/superpowers/plans/2026-09-03-star-arm-102-ld-rebot-b601-live-follow-implementation.md`

**Interfaces:**
- Consumes: final behavior from Tasks 1–2.
- Produces: operator documentation stating that live follow is register-write-free at startup and requires pre-provisioned POS_VEL configuration.

- [ ] **Step 1: Update documentation without changing the launch command**

Replace statements that say live follow writes RIDs 25–28 or switches mode automatically. State explicitly that startup reads RID 10 and RIDs 25–28, refuses mismatches before enable, starts from current poses, and reserves the web parking pose for guarded shutdown return.

- [ ] **Step 2: Check for stale behavior claims and forbidden calls**

Run:

```bash
rg -n "prepare_pos_vel|write_register_f32|ensure_mode" \
  Python_SDK/rebot_b601_mapping/follower_controller.py \
  Python_SDK/rebot_b601_mapping/live_follow.py \
  Python_SDK/rebot_b601_mapping/tests
```

Expected: no production live-follow startup call or test expectation for register writes remains.

- [ ] **Step 3: Run the complete test suite**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests -q
```

Expected: all tests PASS.

- [ ] **Step 4: Run compile and data validation**

```bash
PYTHONPATH=Python_SDK .venv/bin/python -m compileall \
  Python_SDK/rebot_b601_mapping -q
.venv/bin/python -m json.tool \
  Python_SDK/rebot_b601_mapping/live_follow.example.json >/dev/null
git diff --check
```

Expected: all commands exit 0.

- [ ] **Step 5: Commit documentation and report the hardware gate**

```bash
git add Python_SDK/rebot_b601_mapping/README.md \
  docs/superpowers/specs/2026-09-03-star-arm-102-ld-rebot-b601-live-follow-design.md \
  docs/superpowers/plans/2026-09-03-star-arm-102-ld-rebot-b601-live-follow-implementation.md \
  docs/superpowers/plans/2026-09-04-follow-read-only-controller-precheck.md
git commit -m "docs: 对齐跟随只读启动流程"
```

Report test counts exactly. State that no serial port was opened and that the original live command must not be run until motorbridge is installed with the verified frame-classification fix and the user grants fresh real-motion authorization.
