# rebotarm_dashboard

本机 Web 面板与状态 API 包。它聚合 ROS 状态、提供页面/HTTP/SSE 接口，并把浏览器意图转交给 `rebotarm_teleop`、`rebotarm_teach` 和控制器服务；不实现运动规划、示教算法或电机 SDK 访问。

## 目录结构

```text
rebotarm_dashboard/
├── rebotarm_dashboard/
│   ├── teleop_status_panel_node.py # 唯一常驻 ROS 节点：状态聚合 + HTTP 服务
│   ├── status_panel_http.py        # GET、SSE、POST 路由和本机 HTTP server
│   ├── status_panel_page.py        # 从包资源读取 index.html
│   ├── status_panel_assets/
│   │   ├── index.html              # 页面、JavaScript 和样式资源
│   │   └── __init__.py
│   ├── status_panel_state.py       # 快照、状态存储和 SSE 编码
│   ├── status_panel_api.py         # POST 白名单与 handler 分发
│   ├── arm_command_api.py          # enable/disable/safe_home 命令门控
│   ├── arm_control_client.py       # 控制器 Trigger 服务客户端
│   ├── web_command_gateway.py      # dry_run/execute 执行模式安全门
│   ├── web_robot_assets.py         # URDF、网格路径和夹爪显示换算
│   ├── parameter_helpers.py        # 参数数组与传感器 QoS 整形
│   └── __init__.py
├── setup.py                         # 安装 HTML 资源并注册节点
├── package.xml
└── resource/rebotarm_dashboard
```

## 脚本职责

- `TeleopStatusPanelNode` 订阅关节/电机/整臂/遥操作/示教状态，把快照交给 HTTP 层，并创建 ROS 服务与动作客户端。
- `status_panel_http.py` 提供 `/`、`/api/status`、`/api/config`、示教记录查询、`/events` SSE、`/robot/urdf` 和 `/robot/meshes/<name>`。
- `status_panel_api.py` 集中列出可写 POST 路径，例如键盘、点到点、示教录制/回放、停止、回安全原点和使能/失能。
- `status_panel_page.py` 用 `importlib.resources` 读取安装后的 `status_panel_assets/index.html`，不依赖源码树相对路径。
- `arm_command_api.py` 与 `web_command_gateway.py` 在调用 ROS 之前检查命令白名单、回放锁定和执行模式。
- `status_panel_state.py` 只做状态聚合/序列化；`web_robot_assets.py` 只做模型资源、安全路径和夹爪显示数据处理。
- `arm_control_client.py`、`parameter_helpers.py` 是客户端和参数辅助层，避免把 ROS 调用散落在 HTTP 路由中。

## 对外入口和启动

```bash
ros2 run rebotarm_dashboard TeleopStatusPanelNode
```

生产组合通常由 `rebotarm_bringup` 启动：

```text
浏览器 -> rebotarm_dashboard -> rebotarm_teleop / rebotarm_teach / controller ROS 接口
```

Dashboard HTTP 只绑定本机 UI/API，不是视觉传输链路；当前视觉路径使用 ROS RGB-D、CameraInfo 和 detection 话题。

## 边界

- 不在这里生成 `JointTrajectory`、做 MoveIt 规划或实现示教回放质量策略。
- 不从这里直接 import 电机 SDK。
- Web 预览和真实执行由请求中的执行模式及下游安全门区分；默认 dry-run/预览不能当作真机执行证据。

## 标定页面

同端口 `/calibration` 提供手眼/TCP向导，HTTP请求通过CalibrationCommand调用
独立标定节点。默认不自动启动相机/标定/控制器；不会自动应用外参。
操作与证据见 [网页标定说明](../../docs/calibration_web_usage.md)。
