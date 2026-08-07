# Current / Upstream MuJoCo A/B 对比结果

日期：2026-08-07  
报告 JSON：[2026-08-07-baseline-comparison.json](2026-08-07-baseline-comparison.json)  
上游快照：`main@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`  
测试时长：1 秒 headless physics。

## 1. Model contract

| 指标 | Current baseline | Upstream robot.xml | Upstream scene.xml |
|---|---:|---:|---:|
| `nq` | 8 | 8 | 15 |
| `nv` | 8 | 8 | 14 |
| `nu` | 7 | 8 | 8 |
| `njnt` | 8 | 8 | 9 |
| `nbody` | 11 | 11 | 13 |
| `ngeom` | 20 | 20 | 27 |
| `nsensor` | 0 | 26 | 26 |

Current 使用 6 个 arm position actuator + 1 个 `gripper` actuator；上游使用 6 个
独立 torque actuator + 左右手指各 1 个 force actuator。上游 scene 额外包含 free cube，
因此不能与 robot-only XML 直接按 `nq/nv` 互换。

## 2. Health/headless

两套模型均通过有限值检查：

- Current：`ok=true`、`8 joints`、`7 actuators`、1 秒 smoke `finite=true`、3 个 contacts；
- Upstream：`ok=true`、`8 joints`、`8 actuators`、`physics_step_finite=true`、headless
  `achieved_duration=1.0000000000000007`。

结论：两套 backend 都能稳定启动 headless physics；upstream 的 health JSON 更完整，
但不能据此判断控制效果更好。

## 3. Joint trajectory response

两套均用各关节独立目标、运行 1 秒的 step-response 对比：

| 指标 | Current | Upstream | 观察 |
|---|---:|---:|---|
| max final abs error | `0.7860 rad` | `0.6443 rad` | Upstream 较小，约降低 18.0% |
| max abs error | `1.5683 rad` | `1.5697 rad` | 峰值误差基本相同 |
| max abs velocity | `4.0664 rad/s` | `1.9567 rad/s` | Upstream 约低 51.9% |
| max actuator force | `27.0` | `19.1282` | actuator/control contract 不同，不能只看绝对值判优劣 |

该结果说明 upstream motor/control 参数在本次 1 秒响应中更平滑、最终误差较小；但
它使用不同 actuator 类型、dynamics 和 calibration，尚不能作为替换当前控制器的结论。

## 4. Gripper/contact smoke

| 指标 | Current grasp benchmark | Upstream scene smoke |
|---|---:|---:|
| finite | `true` | `true` |
| initial cube height | `0.0500 m` | `0.0400 m` |
| final cube height | `0.019892 m` | `0.019945 m` |
| max contacts | 6 | 4 |
| final contacts | 6 | 4 |
| gripper result | `contact_without_lift`, `grasp_success=false` | no lift command; gripper width `0.040466 m` |

两套都能产生接触并保持有限状态，但都没有完成 lift success；当前接触数更高，不能
直接解释为抓取质量更好，因为碰撞几何、scene keyframe 和 actuator contract 不同。

## 5. Upstream test status

在完整本地快照及所需 support inputs 下运行：

```text
219 passed, 1 skipped, 1 failed
```

唯一失败仍是 `tests/test_mujoco_sim_core.py::test_saved_integration_state_replays_deterministically`
第 379 行的嵌套 `mapping -> tuple` `pytest.approx` 断言。数值最大差约 `7.05e-18`，
仿真时间差为 0；报告将其分类为 `test_defect`，不是 physics runtime failure。

## 6. Decision

本地快照方案已产生可复核的实际对比效果：upstream 在本次参数下表现出更低的最终误差
和峰值速度，但 actuator/sensor/gripper contract 不兼容当前默认 backend。保留两套实现，
后续应在同一控制契约和校准参数下再做公平比较；不直接替换当前 `rebotarm_simulation`。
