# 耦合维护迁移说明

## 资源路径

- 唯一 URDF：`rebotarm_moveit_config/config/rebotarm.urdf`。
- 网格：`rebotarm_moveit_config/meshes/`。原12个文件字节不变；20个URDF引用全部可解析。
- 组合参数：`rebotarm_bringup/config/teleop_control.yaml` 和 `interactive_control.yaml`。
- 视觉准备位运动参数：`rebotarm_motion/config/visual_ready.yaml`。

标准 launch 已更新。外部脚本若直接引用旧 package share 路径，需要改为上述所有者；不保留第二份资源副本。
旧 interactive_control Python/console 包装入口和 vision 的 visual_ready 入口继续兼容。

## 运行职责

Dashboard 通过 `TeachReplayWorkflow` 调用 teach 业务，保留 Web 权限和命令路由。
工作流仍与 Dashboard 同进程，但状态和业务实现归 teach；本次没有引入新的跨进程协议。
MoveIt 服务就绪检查改用规划器公开接口，不读取私有 `_client`。

controller 不再创建示教文件或提供录制服务。`moveit_hardware` 和硬件模式
`teleop_keyboard` 启动独立 `TeachRecorderNode`；仿真模式 `teleop_system` 启动同一实现。
主 Web app 经 moveit_hardware 获得唯一录制实例。录制服务名不变，默认启动时不录制，
不自动开启重力补偿。只启动 driver_only 时，需要另外启动 teach_record 才有录制服务；
已有 app/teleop 录制节点时不要重复启动 teach_record。

录制只写入新批次，要求反馈在0.15秒以内且真实电机状态与位置对应同一批次。
`sample_rate_hz=150` 是消费频率上限；硬件反馈50Hz时不会伪造150Hz的独立采样。
ROS `/joint_states` 发布频率不变，但同一接收批次的重复发布使用同一header时间戳。
没有修改硬件控制频率、控制增益、夹爪零位或失能策略。

仿真固件参考值冻结在自己的 `motor_control_calibration.yaml`，数值保持迁移前一致；
后续修改真实驱动配置不会隐式重调仿真。仿真标定更新应显式审查。

## 验证与使用

反馈与解耦成果已整合到主目录；使用重新构建的非symlink安装空间
`/home/a/project/rebot_Arm/install`。主目录统一使用标准 `build/`、`install/`、
`log/`，不再保留带整合日期的目录。旧构建和旧安装备份已移入回收站；
独立工作树不受本次整理影响。下一次人工启动前，在新终端加载：

```bash
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
source /home/a/project/rebot_Arm/install/setup.bash
```

工作树遥操作已由用户测试确认；主目录整合后本轮只做软件验收，未启动真机controller、访问串口、使能、置零或发送动作。
MuJoCo安装资源加载、Dashboard HTTP读取及禁用命令拦截、录制去重均有独立软件验证。
硬件录制吞吐和运行链切换仍需后续明确授权的实体复测。

主目录整合验证：13包独立构建成功，分层20项及全量822项通过、8项跳过，
Python编译通过；10个关键模块来自主目录安装且与源码一致，20个URDF网格引用
和11个启动入口参数解析通过。MotorBridge运行契约为 `0.4.6+rebotarm.2`、
`feedback_sequence=true`。构建有simulation assets的setuptools包发现警告，
未导致失败。整合检查日志保留在主目录 `log/checks`。
