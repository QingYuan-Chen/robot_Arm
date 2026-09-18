# rebotarm_msgs

reBotArm 的 ROS 2 接口契约包。它只生成 `.msg`、`.srv` 和 `.action` 的语言绑定，不包含节点、硬件访问、规划算法或启动文件。接口改变会同时影响 controller、motion、teach、teleop、dashboard、vision 和 simulation。

## 目录结构

```text
rebotarm_msgs/
├── msg/
│   ├── ArmStatus.msg                 # 整臂健康、使能、控制环和错误码
│   ├── JointMotorState.msg            # 单电机/夹爪反馈
│   ├── JointMotorCmd.msg              # 低层电机命令
│   ├── Detection2D.msg                # 单个 2D 检测
│   ├── Detection2DArray.msg           # 一帧检测集合
│   ├── GraspCandidate.msg             # 单个抓取候选
│   ├── GraspCandidateArray.msg        # 候选集合与最优索引
│   ├── GraspPlan.msg                  # pregrasp + grasp 计划
│   ├── ExecutionState.msg             # 执行阶段状态
│   └── TaskStatus.msg                 # 任务级状态
├── srv/
│   ├── ExecutePose.srv                # 规划并可选执行末端位姿
│   ├── MoveToPoseIK.srv               # 位姿到关节目标/IK 请求
│   ├── PlanGraspForLabel.srv          # 按标签规划抓取
│   ├── SetGripper.srv                 # 设置夹爪开口
│   ├── GraspGripper.srv               # 受限闭合抓取
│   ├── SetMode.srv                    # 设置控制模式
│   ├── SetZero.srv                    # 电机/夹爪置零
│   └── SetTeachRecordPath.srv         # 设置示教记录路径
├── action/
│   ├── MoveToPose.action               # 长时末端位姿运动
│   ├── MoveRelative.action             # 相对位移动作
│   └── ExecuteGrasp.action             # 分阶段抓取动作
├── CMakeLists.txt                     # rosidl_generate_interfaces
└── package.xml                        # rosidl 构建与运行依赖
```

## 接口分层

| 类型 | 主要消费者 | 作用 |
| --- | --- | --- |
| `ArmStatus`、`JointMotorState` | dashboard、teach、验收工具 | 读取连接、使能、反馈新鲜度和故障状态 |
| `Detection2D*`、`GraspCandidate*`、`GraspPlan` | vision、仿真/可视化 | 传递检测、候选和经过筛选的抓取计划 |
| `ExecutionState`、`TaskStatus` | dashboard、voice、vision | 汇报执行阶段和任务占用 |
| `JointMotorCmd` | controller 低层透传 | 明确表示绕过规划的底层命令，必须满足控制器安全门 |
| `ExecutePose`、`MoveToPoseIK`、抓取/夹爪服务 | motion、vision、controller | 规划、IK 和夹爪服务边界 |
| 三个 action | motion、controller、voice/simulation | 表达需要反馈、取消和结果的长时任务 |

## 构建关系

`CMakeLists.txt` 用 `rosidl_generate_interfaces()` 生成 Python/C++ 绑定，依赖字段直接引用的 `builtin_interfaces`、`geometry_msgs`、`std_msgs` 和 `trajectory_msgs`。修改接口时要同步检查所有服务器、客户端、launch 和测试；构建通过不代表真实硬件执行链已验证。

