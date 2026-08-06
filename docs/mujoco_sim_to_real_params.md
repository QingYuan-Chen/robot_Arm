# MuJoCo 到真机迁移参数表

本文用于记录仿真参数和真实硬件参数的对应关系。当前表格先作为迁移模板，
真实数值需要根据电机、减速器、控制板电流限制、夹爪机构和实测负载补齐。

## 边界原则

- MuJoCo 只用于离线物理验证、接触调试、抓取策略迭代。
- 真机执行仍必须经过 `rebotarmcontroller`。
- MuJoCo 的 `kp`、`kv`、`forcerange` 不是电机 SDK 参数，不能直接下发到真机。
- 所有真机安全阈值应以硬件规格和实测为准，仿真参数只能作为验证参考。

## 当前仿真参数

| 关节 | MuJoCo ctrlrange | MuJoCo forcerange | kp | kv | 迁移状态 |
| --- | --- | --- | --- | --- | --- |
| joint1 | -2.8 2.8 | -27 27 | 270 | 24 | 待实机确认 |
| joint2 | -3.14 0 | -27 27 | 270 | 24 | 待实机确认 |
| joint3 | -3.14 0 | -27 27 | 270 | 24 | 待实机确认 |
| joint4 | -1.87 1.57 | -7 7 | 70 | 10 | 当前大阶跃误差较大 |
| joint5 | -1.57 1.57 | -7 7 | 70 | 10 | 待实机确认 |
| joint6 | -3.14 3.14 | -7 7 | 70 | 10 | 当前大阶跃误差较大 |
| gripper | 0 0.045 | -20 20 | 600 | 60 | 待夹爪实测 |

## 需要补齐的真机字段

| 字段 | 来源 | 用途 |
| --- | --- | --- |
| 最大关节角 | 机械限位 / URDF 校准 | position limit |
| 最大速度 | 电机规格 / 实测 | MoveIt velocity limit |
| 最大加速度 | 实测 / 安全策略 | MoveIt acceleration limit |
| 最大电流/力矩 | 电机与驱动器规格 | 真机安全限制 |
| 减速比 | 机械规格 | 仿真惯量和力矩换算 |
| 夹爪最大开口 | 实测 | gripper width mapping |
| 夹爪闭合力 | 实测 | grasp quality 判断 |
| 额定负载 | 任务规格 | step-response benchmark 目标 |

## 下一步

1. 使用 `step-response-suite` 生成当前仿真基线。
2. 补齐真实电机和夹爪规格。
3. 决定仿真参数路线：真实硬件保守标定，或演示优先标定。
4. 按路线调整 `MOTOR_PROFILES`，并重新运行 action / step-response / grasp benchmark。
