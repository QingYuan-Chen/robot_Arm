# Current / Upstream MuJoCo A/B 对比结果

日期：2026-08-07  
报告 JSON：[2026-08-07-baseline-comparison.json](2026-08-07-baseline-comparison.json)  
同一命令契约 JSON：[2026-08-07-baseline-comparison-command-contract.json](2026-08-07-baseline-comparison-command-contract.json)
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

### 3.1 同一命令契约复测

第二轮不再逐关节发送不同后端各自的 native command，而是向两套 backend 发送同一个
6-DOF target vector 和 `gripper_width=0.04 m`：

```text
joint1..joint6 = [1.4, -0.785, -0.785, 0.710, 0.785, 1.570] rad
```

| 指标 | Current adapter | Upstream `RebotArmMujoco` | 观察 |
|---|---:|---:|---|
| max final abs error | `0.7930 rad` | `0.3611 rad` | Upstream 约低 54.5% |
| max abs error | `1.5691 rad` | `1.5697 rad` | 起步峰值基本相同 |
| max abs velocity | `5.2087 rad/s` | `1.9563 rad/s` | Upstream 约低 62.4% |
| max arm actuator force | `27.0` | `20.0223` | 仍受 actuator 类型/限幅影响 |
| final gripper width | `0.040403 m` | `0.039620 m` | 命令接近，但闭环实现不同 |

最终关节误差显示差异并非所有轴一致：

| Joint | Current abs error | Upstream abs error |
|---|---:|---:|
| joint1 | `0.0018 rad` | `0.3611 rad` |
| joint2 | `0.0131 rad` | `0.1137 rad` |
| joint3 | `0.0418 rad` | `0.1012 rad` |
| joint4 | `0.3464 rad` | `0.0332 rad` |
| joint5 | `0.7930 rad` | `0.0924 rad` |
| joint6 | `0.0020 rad` | `0.1104 rad` |

这次结果只证明“同一目标命令”下的现有实现差异，不证明上游模型可直接替换当前模型：
上游仍使用独立 torque/force actuator 和自有 motor controller，Current 仍使用 position
actuator 适配器。下一步若要比较控制器本身，必须在隔离 overlay 中统一 actuator 类型、
limits、gains 和 reset/keyframe，再复测并单独保留该 overlay 证据。

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
