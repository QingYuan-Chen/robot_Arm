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
ros2 run rebotarm_calibration rebotarm_handeye_capture
ros2 run rebotarm_calibration rebotarm_handeye_calibration --input session.json --output report.json
```

这些名字来自 `setup.py`，是包的公开接口。详细参数以各入口的 `--help` 和当前源码为准。

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

网页标定的分阶段技术路线见
[`docs/calibration_web_plan.md`](../../docs/calibration_web_plan.md)。网页只负责向导、
状态和人工确认；会话、采集质量门与数学求解继续由本包和 ROS 适配器负责。

## 采集质量与验收边界

TCP 节点默认要求至少 5 个样本。每次采集总超时 `capture_timeout_sec=15` 秒，
TF 必须在 `maximum_age_sec=0.5` 秒内、时间戳推进，并在
`stability_window_sec=0.4` 秒内保持位置变化不超过 1 mm、姿态变化不超过 0.5 度。
等待人工输入时不采样，按回车后才进入受限采集窗口。

ArUco 模式要求 Image、CameraInfo 与 `aruco.camera_frame` 完全匹配；该参数必须指定
图像对应的光学坐标系。内参分辨率必须匹配，时间差默认最多 0.1 秒，
畸变模型仅支持 plumb_bob/rational_polynomial。TF 按图像时间查询；
不接受无时间戳或缓存旧图。默认质量门为 RMSE <= 1 px、面积 >= 400 px²、距离 <= 2 m。
这些阈值可通过同名 ROS 参数配置；原始时间戳、逐帧质量和拒绝原因写入报告。

手眼残差报告新增 `observability`：相对旋转约束必须满秩且条件数 <= 100，
只绕同一轴运动会被拒绝。求解器同样执行该门，并支持字典与 4×4 矩阵输入。
低残差只证明采样一致性；部署前仍需独立多轴留出姿态验证和物理测量，不能替代硬件验收。

退化 TCP 报告的未知条件数以 JSON null 表示；失败时不输出可复制的 TCP YAML。
节点只读相机和 TF，不执行运动、使能或部署参数。

网页操作、恢复、参数和安全顺序见 [使用说明](../../docs/calibration_web_usage.md)。
网页正式状态所有者为 `session_store.py`，采集入口为 `handeye_capture_node.py`；
`handeye_workflow.py`/`tcp_workflow.py`负责训练与独立验证，`provenance.py`记录来源。
