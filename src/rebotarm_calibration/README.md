# rebotarm_calibration

标定与 TF 校验工具包。它只负责把相机、末端和参考物之间的几何关系解算成可检查的结果；不访问电机 SDK，不规划轨迹，也不直接执行机械臂动作。

## 目录结构

```text
rebotarm_calibration/
├── rebotarm_calibration/
│   ├── aruco_pose.py              # ArUco 角点检测与相机到标记位姿
│   ├── handeye_solver.py          # eye-in-hand 手眼外参求解与评估
│   ├── handeye_residual.py        # 多样本手眼残差、离群和一致性分析
│   ├── handeye_residual_cli.py    # 手眼残差 JSON 命令行入口
│   ├── tcp_calibration.py         # TCP 偏置、旋转和 pivot 模型的纯计算
│   ├── tcp_calibration_node.py    # 交互采样并输出 TCP 标定结果
│   └── __init__.py
├── setup.py                       # ament_python 与 console_scripts
├── package.xml                    # ROS 2 依赖
└── resource/rebotarm_calibration  # ament 包索引
```

## 脚本职责

- `aruco_pose.py`：从 BGR 图像、相机内参和标记尺寸估计 `camera_to_marker` 齐次变换。
- `handeye_solver.py`：根据多组 `base->end`、`camera->marker` 数据求 `end->camera`，并给出旋转/平移残差。
- `handeye_residual.py`：离线分析已有手眼结果的样本一致性；`analyze_handeye_residual` 返回可序列化报告。
- `handeye_residual_cli.py`：读取 JSON 数据集，调用残差分析并打印报告。
- `tcp_calibration.py`：提供末端姿态到 TCP 偏置的估计、平均、门限分析和 YAML 格式化。
- `tcp_calibration_node.py`：ROS 2 交互节点，操作者逐姿态采样，节点负责采集、解算和输出，不负责移动机械臂。

## 对外入口

```bash
ros2 run rebotarm_calibration rebotarm_handeye_residual \
  --input dataset.json --output residual_report.json
ros2 run rebotarm_calibration rebotarm_tcp_calibration
```

两个名字来自 `setup.py`，是包的公开接口。详细参数以各入口的 `--help` 和当前源码为准。

## 数据流与边界

```text
相机图像/CameraInfo + ArUco
        │
        ├── handeye_solver / handeye_residual ──> 手眼外参候选与残差报告
        └── tcp_calibration_node ───────────────> TCP 偏置报告
                                                     │
                                      配置或 TF 被 vision/motion 消费
```

标定结果必须经过人工检查后再写入配置。软件求解通过不等于真实末端方向、接触或抓取验收通过。
