# rebotarmcontroller

真实机械臂与夹爪的硬件控制包，是仓库中唯一允许访问电机 SDK、串口/总线和真实执行器的包。它发布已校验反馈、提供 ROS 服务/动作，并执行最后一道低层安全检查。

## 目录结构

```text
rebotarmcontroller/
├── rebotarmcontroller/
│   ├── rebotarm_controller.py       # reBotArmController ROS 节点装配
│   ├── hardware_manager.py           # 电机 SDK、通道和唯一硬件控制环
│   ├── ros_services.py               # enable/disable/safe_home/stop/zero/夹爪等服务
│   ├── ros_actions.py                # MoveToPose、FollowJointTrajectory、夹爪动作
│   ├── ros_publishers.py              # joint_states、JointMotorState、ArmStatus
│   ├── motor_passthrough.py           # 受门控的低层电机命令透传
│   ├── conversions.py                 # 位姿、RPY、旋转矩阵互转
│   ├── p0_acceptance_core.py          # Gate B/C 的纯判据和指标
│   ├── examples/
│   │   ├── gravity_compensation.py   # 重力补偿/手动拖拽演示
│   │   ├── gripper_control.py         # 夹爪开合交互演示
│   │   ├── move_to.py                 # 关节空间点到点演示
│   │   ├── move_to_pose.py            # 末端位姿演示
│   │   └── p0_gate_bc_acceptance.py   # 显式使能-保持-失能验收工具
│   └── __init__.py
├── setup.py / package.xml / resource/*
```

## 模块职责

- `HardwareManager` 独占厂商 SDK、串口/总线和硬件控制循环；其它模块只能通过它访问真实电机。
- `rebotarm_controller.py` 组合参数、硬件管理器、服务、动作和发布器，形成唯一真实 controller 节点。
- `ros_publishers.py` 只发布已校验反馈；`ArmStatus` 用于通告 connected/enabled/control loop/error 状态。
- `ros_services.py` 是服务级安全边界；`ros_actions.py` 处理位姿、轨迹和夹爪的长时执行/取消。
- `motor_passthrough.py` 是绕过规划的底层入口，只能在硬件层允许时使用，不做轨迹插值、碰撞检查或限位策略。
- `p0_acceptance_core.py` 与 `examples/p0_gate_bc_acceptance.py` 用于现场安全门判断，不把软件测试冒充硬件验收。

## 对外入口

```text
reBotArmController      -> 真机控制主节点
GravityCompensation     -> 重力补偿演示
GripperControl          -> 夹爪交互演示
MoveTo                  -> 关节点位演示
MoveToPose              -> 末端位姿演示
p0_gate_bc_acceptance   -> P0 Gate B/C 验收工具
```

真实 controller 通常由 `rebotarm_bringup/launch/hardware_controller.launch.py` 组合，其他 launch 文件只 include 这个硬件片段，避免重复创建控制器。

## 真机启动原则

1. 上电默认失能；先确认新鲜反馈、通信和现场安全状态。
2. 必须通过显式 `/rebotarm/enable` 才允许运动。
3. `FollowJointTrajectory`、停止、safe home、夹爪和底层命令都必须经过本包。
4. 健康 enabled 状态下，任务级失败不能直接把机械臂带离已验证 baseline 后失能；应停止并保持，或受保护回 baseline、确认到位后再失能。

