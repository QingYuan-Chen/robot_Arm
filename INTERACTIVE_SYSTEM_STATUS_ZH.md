# 机械臂交互控制系统当前状态说明

## 1. 当前目标

当前系统的核心目标是：

```text
RViz 交互 marker
-> pose_target
-> preview
-> execute
```

也就是用户在 RViz 中拖动机械臂末端目标后，系统先生成目标位姿，再做预览求解，最后再通过执行接口进入仿真或真机执行链。

## 2. 主要 ROS2 包职责

### `rebotarm_bringup`

负责：

- launch 启动入口
- URDF / 机械臂描述
- 正式 RViz 配置

### `rebotarm_interactive_control`

负责：

- interactive marker 交互
- preview 链路
- execution 链路
- debug / smoke 测试工具

### `rebotarmcontroller`

负责：

- 真机控制器的 ROS2 包装层
- joint state 发布
- trajectory / 控制接口
- 对接底层 Python SDK / motorbridge

### `rebotarm_msgs`

负责：

- 自定义 service / action 接口

## 3. 当前三大核心节点

### `MarkerServerNode`

职责：

- 创建正式的 `ee_target` interactive marker
- 接收 RViz feedback
- 发布：

```text
/rebotarm/interactive_control/pose_target
```

它只负责：

- 交互输入入口

它不负责：

- IK
- preview 求解
- execute
- estop

---

### `PreviewNode`

职责：

- 订阅：

```text
/rebotarm/interactive_control/pose_target
```

- 进行 preview 求解
- 发布：

```text
/rebotarm/interactive_control/preview
/rebotarm/interactive_control/status
```

它当前内部仍然复用了：

- `PreviewManager`
- `PosePreviewSolver`

因此它已经在节点层面独立出来了，但内部实现后续还可以继续收紧。

---

### `ExecutionNode`

职责：

- 订阅最近一次 preview 结果
- 提供服务：

```text
/rebotarm/interactive_control/execute_preview
/rebotarm/interactive_control/estop
/rebotarm/interactive_control/reset_estop
/rebotarm/interactive_control/set_mode
```

- 处理 simulation / real 分流
- 将真机执行请求送入 trajectory action 路径

它负责的是：

- 执行控制入口

## 4. 当前正式主链

现在系统正式主链已经整理为：

```text
RViz
-> MarkerServerNode
-> /pose_target
-> PreviewNode
-> /preview + /status
-> ExecutionNode
-> /execute_preview
-> reBotArmController
```

这就是当前工程里最重要的一条主链。

## 5. 当前正式启动方式

### 正式入口

后续默认使用：

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py
```

这个入口会启动：

- `reBotArmController`
- `robot_state_publisher`
- `MarkerServerNode`
- `PreviewNode`
- `ExecutionNode`
- `rviz2`

这是当前正式系统入口。

## 6. 当前调试入口

如果只是单独排查 marker / preview / execution，可使用这些调试入口：

- `interactive_debug.launch.py`
- `marker_server_debug.launch.py`
- `preview_debug.launch.py`
- `interactive_stage5_debug.launch.py`

这些是调试入口，不是正式主入口。

## 7. Legacy / 兼容入口

以下入口目前仍然保留，但已经不推荐作为默认工作流：

- `interactive_basic.launch.py`
- `InteractiveTargetNode`

它们现在主要用于：

- 兼容旧链路
- 防止迁移过程中一下子失去历史入口

后续推荐的正式架构是：

```text
MarkerServerNode -> PreviewNode -> ExecutionNode
```

## 8. 当前已经验证过的能力

### 交互能力

- RViz marker 显示正常
- marker 拖动正常
- `/ee_target/feedback` 有输出

### 预览链路

- `pose_target -> preview` 链路已打通
- `/preview` 有输出
- `/status` 有输出

### 执行链路

- `execute_preview` 服务可调用
- `execution accepted` 已验证

### 架构能力

- 正式链路与调试链路已经分开
- marker / preview / execution 已拆为独立节点
- 正式入口已经建立

## 9. 当前仍属“临时实现”的部分

### `PreviewNode` 内部仍然复用旧 preview 管理逻辑

目前它仍然复用了：

- `PreviewManager`
- `PosePreviewSolver`

这意味着：

- 节点边界已经清晰
- 但内部逻辑还不是最终最优结构

### `ExecutionNode` 目前仍是“最小落地版”

当前已经能工作，但后续还可以进一步改进：

- 更清晰的状态存储
- 更正式的 preview 到 execute 数据交接
- 更明确的 simulation / real 边界

### `InteractiveTargetNode` 仍然存在

它现在已经不应该继续扩功能，后续应继续清理或最终移除。

## 10. 当前最容易踩坑的地方

### 1. launch 混用

不要同时乱起：

- `interactive_basic.launch.py`
- `interactive_system.launch.py`
- 各种 debug launch

否则容易出现：

- namespace 冲突
- marker update 序号异常
- 当前不知道到底是谁在发 topic

### 2. build / install 缓存残留

你已经遇到过：

- `PackageNotFoundError`
- entry point 不更新

如果：

- 改了 `setup.py`
- 新增了 console script
- launch 找不到新入口

优先这样清理：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 3. DDS 共享内存残留

你之前遇到过：

- `RTPS_TRANSPORT_SHM Error`

这通常不是代码逻辑本身的问题，更像是：

- ROS2 / Fast DDS 共享内存残留
- 上一次进程未正常退出

必要时可以：

- 关干净 ROS2 进程
- 或直接重启虚拟机

## 11. 当前工程定位

一句话概括：

```text
这个项目已经不再是一个临时拼起来的原型，而是一个完成了第一轮架构重整、具备继续接 MoveIt2 的 ROS2 机械臂交互控制骨架。
```

这说明你现在最难的“把乱工程收成能持续开发的工程骨架”这一步，已经基本完成了。

## 12. 下一阶段建议

下一阶段建议按这个顺序推进：

### 第一优先级：继续稳定化收尾

建议继续做：

- 进一步清理 `InteractiveTargetNode`
- 收紧 `PreviewNode` / `ExecutionNode` 的内部实现
- 统一状态流

### 第二优先级：开始接 MoveIt2

等稳定化收尾差不多后，再开始：

- MoveIt2 接入 preview 层
- IK 求解
- 可达性判断
- 约束与轨迹规划

也就是说：

```text
现在最合适的下一阶段是：稳定化收尾 + MoveIt2 接入
```
## 视觉链路阶段记录（2026-05-21）

当前视觉部分进入“Windows 采集与推理、Ubuntu ROS2 消费结果”的阶段。

已确认：

- Gemini2 设备本身正常，Windows 设备列表能看到 Orbbec Gemini 2 RGB Camera、IR Camera、Depth Camera 和 Data Channel。
- VMware 直通 Gemini2 给 Ubuntu 时，容易出现彩色图灰/黑、帧率异常低的问题；暂不把这个作为主路线。
- Windows 侧 `tools/windows_mjpeg_server.py` 已支持 DSHOW 固定后端、黑帧检测自动重开、YOLO 检测 JSON、MJPEG 原图和标注图。
- 当前验证到 Gemini2 RGB 的 Windows 摄像头索引是 `3`；索引 `0` 不是目标 Gemini2。
- Ubuntu 侧 `rebotarm_vision` 已支持 `network_mjpeg` 和 `enable_network_detection`，可从 Windows HTTP 接口发布 `/camera/color/image_raw` 与 `/grasp/detections`。
- 已在 Ubuntu 上构建通过：`colcon build --symlink-install --packages-select rebotarm_vision`。
- 已验证 `/grasp/detections` 能发布 Windows YOLO 结果，频率约 15Hz。

注意事项：

- 如果使用全局 `D:\esp\python.exe`，当前 torch 是 CPU 版，应使用 `--yolo-device cpu`。
- 如果使用用户已有的 CUDA YOLO 虚拟环境，再使用 `--yolo-device 0`。
- 旧服务如果没有带 `--model-path`，`/detections.json` 会为空；如果画面全黑，停止旧服务后用 `--backend dshow --camera-index 3` 重新启动。

下一步：

1. 用用户已有 YOLO 虚拟环境启动 Windows 服务，确认 `detections.json` 稳定。
2. 在 Ubuntu 继续验证 `/grasp/detections` 和 `/camera/color/image_raw`。
3. 后续再接深度图或 2D 检测框到 3D 抓取候选点的转换。

## 2D 抓取候选阶段记录（2026-05-21）

当前新增 `rebotarm_grasp_candidate_node`，视觉链路推进为：

```text
Windows Gemini2 彩色图
-> Windows YOLO detections.json
-> Ubuntu /grasp/detections
-> Ubuntu /grasp/candidates
```

已完成：

- 新增 `rebotarm_vision.converters.grasp_candidates`，把 `Detection2DArray` 转成 `GraspCandidateArray`。
- 新增 `rebotarm_vision.grasp_candidate_node`，订阅 `/grasp/detections`，发布 `/grasp/candidates`。
- `vision.launch.py` 已同时启动视觉节点和抓取候选节点。
- `camera.yaml` 已加入候选节点参数：
  - `grasp.input_topic: /grasp/detections`
  - `grasp.output_topic: /grasp/candidates`
  - `grasp.min_confidence: 0.5`
  - `grasp.allowed_classes: [""]`
- 网络彩色图没有深度时，不再把正常 color-only 帧误报为 partial frame。

校验结果：

- `python3 -m pytest tests/test_network_detection_json.py tests/test_network_mjpeg_driver.py tests/test_grasp_candidate_converter.py -q`：4 个测试通过。
- `colcon build --symlink-install --packages-select rebotarm_vision`：构建通过。
- `/grasp/candidates` 冒烟验证通过，能输出候选点，频率约 `14.8-15Hz`。

下一步：

1. 给 `grasp.allowed_classes` 配置真实要抓的类别，例如 `bottle`、`cup`、`banana`，避免把椅子、显示器这类背景目标作为候选。
2. 接入深度图或标定转换，把 2D 像素候选点变成相机坐标系下的 3D 抓取点。
3. 再把 3D 抓取点接到 MoveIt 预览和执行链路。

## 3D 抓取候选预留阶段记录（2026-05-22）

当前新增 `rebotarm_grasp_3d_node`，链路预留为：

```text
/grasp/candidates
+ /camera/depth/image_raw
-> /grasp/candidates_3d
```

已完成：

- 新增 `rebotarm_vision.converters.grasp_3d`，根据像素点、深度图和相机内参计算相机坐标。
- 新增 `rebotarm_vision.grasp_3d_node`，订阅 2D 候选点和深度图，发布 3D 候选点。
- `vision.launch.py` 已同时启动视觉节点、2D 候选节点、3D 候选节点。
- `camera.yaml` 已加入 `grasp_3d.*` 参数。
- 当前没有深度图时，3D 节点只等待深度，不影响 `/grasp/candidates` 继续发布。

校验结果：

- 先写测试并确认失败：缺少 `rebotarm_vision.converters.grasp_3d`。
- 实现后 `python3 -m pytest tests/test_network_detection_json.py tests/test_network_mjpeg_driver.py tests/test_grasp_candidate_converter.py tests/test_grasp_3d_converter.py -q`：6 个测试通过。
- `colcon build --symlink-install --packages-select rebotarm_vision`：构建通过。
- ROS 节点级校验：人工发布 `u=740, v=410, depth=1000mm`，输出 `x=0.2, y=0.1, z=1.0`。
- 真实 `vision.launch.py` 冒烟：`/grasp/candidates` 仍约 `15Hz`，3D 节点正常启动。

下一步：

1. 获取 Gemini2 在 `1280x720` 彩色/深度对齐模式下的真实内参。
2. 让 Windows 或 Ubuntu 提供稳定 `/camera/depth/image_raw`。
3. 验证 `/grasp/candidates_3d` 的坐标方向和机械臂基坐标系 TF 转换。

## Windows 深度转发阶段记录（2026-05-22）

当前选择第一条路线：Gemini2 继续留在 Windows 主机，Windows 读取彩色图、深度图并运行 YOLO，Ubuntu 只通过 HTTP 消费结果。

新增接口：

```text
/depth.png
/camera_info.json
```

已完成：

- `tools/windows_mjpeg_server.py` 新增 `--capture-source orbbec`。
- Orbbec 模式使用 `pyorbbecsdk` 读取 Gemini2 color + depth，并开启硬件对齐 `HW_MODE`。
- `/depth.png` 输出 16 位 PNG 深度图，单位按当前链路约定为毫米。
- `/camera_info.json` 输出 `fx/fy/cx/cy`、深度尺寸、深度有效像素数等信息。
- Ubuntu `NetworkMjpegDriver` 新增 `depth_url`，能读取并解码 16 位 PNG。
- `camera.yaml` 已配置 `camera.network_depth_url: http://192.168.145.1:8080/depth.png`。
- 为兼容旧 OpenCV 服务，`camera.enable_depth` 暂时保持 `false`；只要 `depth.png` 可用，ROS 仍会发布 `/camera/depth/image_raw`。

校验结果：

- 已先写 `test_network_depth_driver.py` 并确认失败：旧驱动不支持 `depth_url`。
- 实现后相关测试 `7 passed`。
- `colcon build --symlink-install --packages-select rebotarm_vision` 通过。
- 当前 8080 仍是旧 OpenCV 服务，因此 `/depth.png` 和 `/camera_info.json` 返回 404，符合预期。
- 旧服务无深度时真实 launch 冒烟通过：`/grasp/candidates` 仍约 `15Hz`，没有 Traceback、ERROR、partial frame 警告。

待实机验证：

- 在用户的 Windows Orbbec/YOLO 虚拟环境中安装或确认 `pyorbbecsdk`。
- 用 `--capture-source orbbec` 启动 Windows 服务。
- 验证 `/depth.png`、`/camera/depth/image_raw` 和 `/grasp/candidates_3d`。

## Windows Orbbec 深度实测结果（2026-05-22）

本次对当前 Windows 环境做了实际测试。

设备状态：

- Windows 能识别 Gemini2 设备。
- 设备管理层面存在：
  - `Orbbec Gemini 2 RGB Camera`
  - `Orbbec Gemini 2 IR Camera`
  - `Orbbec Gemini 2 Depth Camera`
  - `Orbbec Gemini Data Channel`
- 设备状态均为 `OK`。

当前服务状态：

- 8080 端口仍是旧 OpenCV 服务：
  - 启动命令包含 `--backend dshow --camera-index 3`
  - `/health` 返回 `ok; camera=backend=dshow opened=True`
  - `/depth.png` 返回 `404`，说明旧服务没有深度接口。

Python 环境检查：

- `D:\anaconda3\envs\yolov11\python.exe`
  - `ultralytics=True`
  - `torch=2.3.0+cu118`
  - `cuda=True`
  - `pyorbbecsdk=False`
- `D:\anaconda3\envs\yolov8\python.exe`
  - `ultralytics=True`
  - `torch=2.4.0+cu124`
  - `cuda=True`
  - `pyorbbecsdk=False`
- `D:\esp\python.exe`
  - `ultralytics=True`
  - `torch=2.12.0+cpu`
  - `cuda=False`
  - `pyorbbecsdk=False`

独立端口 Orbbec 模式测试：

```powershell
D:\anaconda3\envs\yolov11\python.exe tools\windows_mjpeg_server.py `
  --capture-source orbbec `
  --host 127.0.0.1 `
  --port 8081 `
  --width 1280 `
  --height 720 `
  --fps 30 `
  --depth-width 1280 `
  --depth-height 720 `
  --depth-fps 30 `
  --model-path tools\yolo26s-seg.pt `
  --yolo-device 0 `
  --detection-fps 15
```

测试结果：

- 服务进程能启动 HTTP 接口。
- `/camera_info.json` 可以返回默认结构。
- `/depth.png` 返回 `503`。
- `/health` 显示：

```text
pyorbbecsdk unavailable: ModuleNotFoundError: No module named 'pyorbbecsdk'; camera=orbbec_import_failed
```

结论：

- 当前深度链路的主要问题不是 Gemini2 设备，也不是 ROS2 网络深度消费代码。
- 当前阻塞点是 Windows 可用 YOLO 环境里没有安装 `pyorbbecsdk`。
- 推荐优先在 `D:\anaconda3\envs\yolov11` 中安装/配置 `pyorbbecsdk`，因为该环境已有 GPU 版 torch 和 ultralytics。
## Windows pyorbbecsdk 源码编译安装记录（2026-05-22）

本次继续使用独立环境：

```text
D:\anaconda3\envs\orbbec_yolo
Python 3.11.15
```

已完成：

- 安装 Visual Studio Build Tools 2022。
- 确认 MSVC 可用：`vcvars64.bat`、`cl.exe`。
- 从 Gitee 获取 `pyorbbecsdk` 源码到 `sdk\pyorbbecsdk`。
- 用 CMake + MSVC 编译通过，生成 `pyorbbecsdk.cp311-win_amd64.pyd`。
- 执行 `cmake --install`，安装 Orbbec SDK 运行库和扩展。
- 在 `orbbec_yolo` 中执行 `pip install -e sdk\pyorbbecsdk`。
- 验证 `import pyorbbecsdk` 成功。
- 验证 `Context()` 创建成功。

当前问题：

- `Pipeline()` 会触发 Windows 原生异常：`0xc0000374`。
- `Context().query_devices()` 当前返回 `0`。
- 同一时间 Windows PnP 查询已经看不到 Orbbec Gemini2 设备，OpenCV 也只看到 index `0`，看不到原来的 Gemini2 index `3`。

当前判断：

- `pyorbbecsdk` 已经在 Python 3.11 环境中源码编译并安装成功。
- 现在不能继续实测 `/depth.png` 的主要原因是 Gemini2 当前没有连接在 Windows 主机侧，或 USB/VMware 状态需要重新切回 Windows。
- 旧 8080 OpenCV 服务因为相机不可用已被停止，避免占端口造成误判。

下一步操作：

1. 在 VMware 菜单里把 Gemini2 从 Ubuntu 断开，重新连接到 Windows 主机。
2. 在 Windows 设备管理器/Orbbec Viewer 中确认 Gemini2 RGB/Depth/Data Channel 都出现。
3. 再运行：

```powershell
D:\anaconda3\envs\orbbec_yolo\python.exe -X faulthandler -c "from pyorbbecsdk import Context; c=Context(); print(c.query_devices().get_count())"
```

4. 如果设备数大于 0，再启动 Orbbec 深度服务测试 `/depth.png`。

## Windows Orbbec 深度链路实测通过（2026-05-22）

用户重新连接 Gemini2 摄像头线后继续测试。

Windows 设备状态：

- Windows 再次识别到 Gemini2：
  - `Orbbec Gemini 2 RGB Camera`
  - `Orbbec Gemini 2 IR Camera`
  - `Orbbec Gemini 2 Depth Camera`
  - `Orbbec Gemini Data Channel`
- OpenCV DSHOW 扫描恢复：
  - index `0` 可打开
  - index `3` 可打开，Gemini2 RGB 画面正常

pyorbbecsdk 状态：

- `Context().query_devices()` 返回 `1`
- 设备枚举：

```text
Orbbec Gemini 2
Serial Number: AY6V16300BP
Connection Type: USB3.0
```

- `Pipeline()` 创建成功。

Windows Orbbec 服务：

当前启动在测试端口 `8081`：

```powershell
D:\anaconda3\envs\orbbec_yolo\python.exe tools\windows_mjpeg_server.py `
  --capture-source orbbec `
  --host 0.0.0.0 `
  --port 8081 `
  --width 1280 `
  --height 720 `
  --fps 30 `
  --depth-width 1280 `
  --depth-height 720 `
  --depth-fps 30 `
  --model-path= `
  --detection-fps 15
```

说明：本次为了先验证深度，使用 `--model-path=` 关闭 YOLO。

Windows 本机接口测试：

- `/health`：

```text
ok; detector=yolo disabled; camera=orbbec opened=True align=HW_MODE; depth=ok
```

- `/snapshot.jpg`：HTTP 200，约 130 KB，图像标准差正常。
- `/depth.png`：HTTP 200，约 330 KB，解码为 `uint16 (720, 1280)`。
- 深度有效像素约 `52 万`。
- `/camera_info.json` 返回真实内参：

```text
fx=692.562744140625
fy=692.2272338867188
cx=641.2417602539062
cy=361.8166198730469
```

Ubuntu 网络访问测试：

- Ubuntu 能访问 `http://192.168.145.1:8081/depth.png`
- 解码结果：`uint16 (720, 1280)`，有效深度正常。

ROS2 测试：

- `camera.yaml` 已切换到 `8081`，并写入上述 Gemini2 内参。
- `/camera/depth/image_raw` 能发布 `mono16 1280x720`。
- 修复了 `rebotarm_grasp_3d_node` 的深度订阅 QoS：改为 `qos_profile_sensor_data`，解决 sensor data 发布与默认可靠订阅不兼容的问题。
- 人工发布 2D candidate 后，`/grasp/candidates_3d` 输出成功：

```text
frame=camera_depth_frame
source=3d_depth_projection
valid=True
x=-0.0039
y=-0.0057
z=2.174
jaw_width=0.3139
object_length=0.3141
```

当前注意事项：

- 当前 `orbbec_yolo` 环境还没有安装/验证 `ultralytics` 和 GPU torch，所以 Windows Orbbec 服务本次只验证了彩色图和深度图，没有跑 YOLO。
- `/camera/depth/image_raw` 频率实测有波动，短时约 `0.8-4Hz`，后续需要优化网络拉取/节点循环或改为独立深度线程。
- 下一步可以把 YOLO 依赖装进 `orbbec_yolo`，实现同一个 Windows 服务同时输出 `/detections.json` 和 `/depth.png`。

## Windows Orbbec + YOLO + 深度完整链路实测通过（2026-05-22）

当前 `orbbec_yolo` 环境已经完成 GPU YOLO 验证，并且 Windows 服务可同时读取 Gemini2 彩色图、深度图和 YOLO 检测结果。

当前服务进程：

```text
D:\anaconda3\envs\orbbec_yolo\python.exe tools\windows_mjpeg_server.py
--capture-source orbbec
--port 8081
--width 1280 --height 720 --fps 30
--depth-width 1280 --depth-height 720 --depth-fps 30
--model-path tools\yolo26s-seg.pt
--yolo-device 0
--detection-fps 15
```

Windows 接口实测：

- `/health` 返回 `ok; camera=orbbec opened=True align=HW_MODE; depth=ok`。
- `/detections.json` 有实时检测结果，本次画面中检测到 `laptop/chair/tv/keyboard/mouse/backpack` 等目标。
- `/depth.png` 返回 HTTP 200，约 `356KB`，为 16 位深度 PNG。

Ubuntu ROS2 完整联调：

- `/grasp/detections`：实测约 `15Hz`。
- `/camera/depth/image_raw`：发布 `mono16 1280x720`，深度数据来自 Windows `/depth.png`。
- `/grasp/candidates_3d`：实测约 `15Hz`，能输出 `valid: true` 的 3D 候选点。
- 日志中未发现 `Traceback`、`process has died`、QoS 不兼容或深度异常。

阶段判断：

视觉链路已经进入“3D 抓取候选可用”的阶段。现在主要瓶颈不是环境安装，而是：

1. 目标筛选：从通用 YOLO 类别里选出真正要抓的物体。
2. 坐标标定：把 `camera_depth_frame` 转到机械臂基座坐标系。
3. 抓取策略：把 3D 候选点转换成夹爪姿态、预抓取点和退出点。
4. MoveIt 闭环：调用规划和执行节点完成一次可控抓取。

## 目标筛选与无效 3D 候选过滤完成（2026-05-22）

当前已经把抓取候选从“所有 YOLO 目标”收紧为“白名单小物体目标”：

```yaml
grasp.min_confidence: 0.35
grasp.allowed_classes:
  - bottle
```

并新增 3D 输出过滤：

```yaml
grasp_3d.publish_invalid_candidates: false
```

实测结果：

- 当前 `/grasp/candidates` 只允许 `bottle`。
- `chair/tv/laptop/keyboard/mouse/cup/tool` 等类别已经不会进入抓取候选。
- `/grasp/candidates_3d` 只输出 `valid: true` 的瓶子 3D 坐标；如果画面中没有 `bottle`，候选为空是正常结果。
- 相关单元测试 `8 passed`。

当前下一步：

1. 当前先围绕 `bottle` 做单目标抓取验证。
2. 做相机到机械臂基座的外参标定。
3. 生成夹爪姿态和预抓取/退出点，再接 MoveIt 执行。

## 手眼外参记忆已接入视觉启动链路（2026-05-23）

当前已新增：

```text
src/rebotarm_vision/config/handeye.yaml
src/rebotarm_vision/rebotarm_vision/handeye_config.py
```

`handeye.yaml` 用来存储 Gemini2 相对夹爪末端的 eye-in-hand 外参：

```text
end_link -> camera_depth_frame
```

`vision.launch.py` 已经接入自动发布逻辑：每次启动视觉链路时，会读取 `handeye.yaml` 并启动 `tf2_ros static_transform_publisher`。这意味着手眼标定结果可以保存，不需要每次启动都重新标定。

当前外参已经从旧版 `softare/rebot_grasp/config/calibration/orbbec_gemini2/hand_eye.npz` 迁移：

```text
translation = [0.005668446, 0.034519527, 0.056490805]
quaternion = [0.567049781, -0.565885703, 0.435347452, 0.410731680]
```

实测验证：

- `test_handeye_config.py` 通过。
- 视觉包构建通过。
- `ros2 run tf2_ros tf2_echo end_link camera_depth_frame` 能读到上述 TF。

下一步是用静止物体做迁移结果验证：机械臂换几个姿态观察同一个目标，检查目标转换到 `base_link` 后是否基本不变。若误差过大，再重新运行 ArUco 手眼标定。

## base_link 稳定性观察工具已加入（2026-05-23）

当前新增：

```text
src/rebotarm_vision/rebotarm_vision/transform_points.py
src/rebotarm_vision/rebotarm_vision/grasp_base_watch_node.py
tests/test_transform_points.py
```

并注册命令：

```bash
ros2 run rebotarm_vision rebotarm_grasp_base_watch
```

用途：

- 订阅 `/grasp/candidates_3d`
- 查询 TF：`base_link <- camera_depth_frame`
- 输出同一目标在 `base_link` 下的位置
- 计算最近窗口内 xyz 波动 `spread50`

验证状态：

- `test_transform_points.py` 通过。
- `rebotarm_vision` 构建通过。
- 命令可启动，短时 `timeout` 退出无 traceback。

下一步实际操作：保持一个目标不动，移动机械臂到不同姿态，观察 `base=(...)` 和 `spread50=(...)mm` 是否稳定。
## 2026-05-23 当前阶段：实机拖动验证手眼标定

当前项目已经从“Windows 采集 + YOLO + Ubuntu ROS2 接收 3D 候选点”推进到“用实机姿态变化验证手眼标定是否可用”的阶段。

目前验证目标：

- Windows 侧 Gemini2 + YOLO 服务继续输出 `bottle` 检测和深度图。
- Ubuntu 侧 `rebotarm_vision` 发布 `/grasp/candidates_3d`。
- `rebotarm_grasp_base_watch` 将 `camera_depth_frame` 下的候选点转换到 `base_link`。
- 实机进入重力补偿后，人工拖动机械臂到几个不同姿态。
- 观察同一个静止 `bottle` 的 `base_link=(x,y,z)` 是否基本不变。

本次发现并修复的问题：

```text
/rebotarm/gravity_compensation/start
success=False
message="'RobotArm' object has no attribute 'fresh'"
```

结论：这是 `rebotarmcontroller` 与当前底层机械臂 SDK 的方法名兼容问题，不是 YOLO、深度图或 TF 本身的问题。已将反馈刷新逻辑改成兼容当前 SDK 的实现，并在 Ubuntu 端编译通过 `rebotarmcontroller`。

下一步测试顺序：

1. 重启实机 launch，确保加载新编译的 controller。
2. 调用 `/rebotarm/gravity_compensation/start`。
3. 如果服务返回 `success=True`，先轻轻扶住机械臂测试拖动手感。
4. 再打开 `rebotarm_grasp_base_watch`，观察静止瓶子的 `base_link` 坐标。
5. 每移动一个机械臂姿态，保持目标和相机视野稳定 2-3 秒，记录 `base_link` 的变化。

判定标准：

- 静止目标在同一姿态下跳动很小，说明视觉和深度输入稳定。
- 拖动到不同姿态后 `base_link` 坐标仍接近，说明手眼外参方向大概率正确。
- 如果不同姿态下 `base_link` 明显漂移，优先检查 `end_link -> camera_depth_frame` 手眼外参方向和单位。

## 2026-05-24 手眼稳定性验收结论

实测在机械臂移动过程中，`rebotarm_grasp_base_watch` 的 `spread50` 会短时间变大，例如：

```text
spread50=(17.0,111.3,2.1)mm
```

这是可以接受的。原因是移动过程中最近 50 帧窗口会同时包含移动前、移动中、移动后的视觉帧和 TF，图像、深度、关节状态不是严格同一时刻。

更重要的判断点是机械臂停住后的稳定值。当前实测停稳后可以回到：

```text
spread50=(0.9,1.1,0.0)mm
```

该结果说明静止状态下视觉深度和 `base_link` 转换链路稳定。

当前验收标准定为：

- 移动过程中 `spread50` 变大：可以接受。
- 停止 2-3 秒后 `spread50` 回到几毫米以内：可以接受。
- 同一个静止瓶子在不同机械臂姿态下，稳定后的 `base_link=(x,y,z)` 差别约 `5-15mm` 以内：当前手眼外参可先用于抓取联调。
- 如果停稳后仍然有几十毫米到上百毫米漂移，再重新检查手眼外参或 TF 方向。

因此当前阶段可以从“手眼验证”推进到“基于 bottle 3D 候选点的抓取联调”。

## 2026-05-24 预抓取执行不动的定位结论

现象：

```text
/rebotarm/interactive_control/execute_preview
success=True
message='execution accepted'
```

但实机电机没有移动。

定位结论：`ExecutionNode` 当前默认读取 `interactive_control.yaml` 中的：

```yaml
mode: simulation
```

因此服务返回 `execution accepted` 只代表仿真预览被接受，不代表真实轨迹已经发送给底层电机。该现象不是 IK 不可达，也不是 bottle 坐标错误。

当前实机执行前必须手动切换为：

```bash
ros2 service call /rebotarm/interactive_control/set_mode rebotarm_msgs/srv/SetMode "{mode: real}"
```

再调用：

```bash
ros2 service call /rebotarm/interactive_control/execute_preview std_srvs/srv/Trigger "{}"
```

下一步优化项：可以考虑在 `interactive_system.launch.py` 中增加显式 `execution_mode` 参数，避免 `use_hardware:=true` 时仍默认处于 `simulation`。

## 2026-05-24 旧版源码抓取思路对照

旧版 `softare/rebot_grasp` 的抓取主线不是简单把 `end_link` 移动到物体 3D 中心，而是：

```text
YOLO / OBB / mask
-> 根据短边估计夹爪开合方向
-> 根据深度和相机内参反投影得到相机坐标下的抓取点
-> 构造视觉抓取坐标系 [grip_axis, open_axis, approach_axis]
-> 转成 reBotArm 需要的 TCP 姿态
-> 当前末端 FK * 手眼外参 = camera -> base
-> 得到 grasp pose 和 pregrasp pose
-> open gripper
-> move_to(pregrasp)
-> move_to(grasp)
-> grasp()
-> 回到 ready_pose
```

关键源码位置：

- `softare/rebot_grasp/scripts/main.py`：主流程，`G` 键冻结当前帧并执行抓取。
- `softare/rebot_grasp/utils/ordinary_grasp.py`：从检测框、mask/OBB、深度图估计抓取点、夹爪方向和夹爪宽度。
- `softare/rebot_grasp/utils/transforms.py`：把视觉抓取坐标系转换成 reBotArm TCP 姿态。
- `softare/rebot_grasp/drivers/robot/rebot_arm.py`：封装 `move_to()`、`open_gripper()`、`grasp()`、`safe_home()`。

因此当前 ROS2 阶段还缺的不是视觉坐标，而是“抓取 TCP 姿态生成”和“grasp_tcp/end_link 偏移建模”。当前 `/grasp/candidates_3d` 只提供物体候选中心点和粗略尺寸，不等价于完整抓取位姿。

## 2026-05-24 抓取实现路线确认

当前抓取实现路线确认：以旧版 `softare/rebot_grasp` 源码为基准迁移到 ROS2，不采用“直接把 `end_link` 移到物体中心点上方”的临时方案作为正式抓取逻辑。

正式路线应保持旧版思路：

```text
检测目标
-> 估计抓取中心、夹爪开合方向、接近方向
-> 生成 grasp TCP 姿态
-> 通过手眼外参转换到 base_link
-> 生成 pregrasp / grasp 两个位姿
-> 打开夹爪
-> 到 pregrasp
-> 到 grasp
-> 力控闭合夹爪
-> 抬起或回 ready_pose
```

后续 ROS2 实现不应只发布物体中心点，而应新增或扩展抓取规划节点，输出可执行的 `pregrasp_pose`、`grasp_pose`、`jaw_width` 和目标类别/置信度。MoveIt 执行阶段应控制 `grasp_tcp` 或等价补偿后的 `end_link`，而不是直接控制 `end_link` 到物体中心。

## 2026-05-24 ROS2 抓取规划第一版完成

已新增 ROS2 抓取规划第一版：

```text
rebotarm_grasp_plan_node
```

新增消息：

```text
rebotarm_msgs/msg/GraspPlan
```

当前链路推进为：

```text
/grasp/candidates_3d
-> rebotarm_grasp_plan_node
-> /grasp/plan
-> /grasp/pregrasp_pose
-> /grasp/grasp_pose
```

第一版实现内容：

- 从 `/grasp/candidates_3d` 选择最佳有效候选。
- 查询 TF，将候选点转换到 `base_link`。
- 按旧版源码思路生成 `grasp_pose` 和 `pregrasp_pose`。
- `pregrasp_pose` 沿 TCP X 轴反方向退 `0.08m`。
- 支持 `end_link -> grasp_tcp` 平移偏移配置，后续测量真实夹爪 TCP 后可直接填入。
- `vision.launch.py` 已接入该节点，视觉链路启动时会自动启动抓取规划节点。

当前限制：

- 第一版还没有完整迁移旧版 `ordinary_grasp.py` 的 OBB/mask 方向估计。
- 当前默认使用 top-down 接近方向：

```text
tcp_approach_axis = [0, 0, -1]
tcp_open_axis = [0, 1, 0]
```

验证结果：

- `colcon build --symlink-install --packages-select rebotarm_msgs rebotarm_vision` 通过。
- `python3 -m pytest tests/test_grasp_candidate_converter.py tests/test_grasp_3d_converter.py tests/test_transform_points.py tests/test_grasp_plan_converter.py -q` 通过，共 9 个测试。
- 无硬件 ROS 冒烟通过，人工 bottle 候选生成规划日志：

```text
plan class=bottle conf=0.80 pregrasp=(+0.224,-0.241,+0.326) grasp=(+0.224,-0.241,+0.246) jaw=0.060m
```

下一步：

- 用真实 `/grasp/plan` 输出测试 MoveIt 预览。
- 测量并填写真实 `grasp_plan.tcp_offset_xyz`。
- 再迁移旧版 OBB/mask 抓取方向估计，让夹爪开合轴跟随物体短边方向。

## 2026-05-24 抓取预览一键转发工具完成

已新增工具节点：

```text
rebotarm_send_grasp_preview
```

用途：把 `/grasp/plan` 中的 `pregrasp_pose` 自动发布到 `/rebotarm/interactive_control/pose_target`，用于快速测试：

```text
视觉识别
-> 3D 候选
-> 抓取规划
-> MoveIt preview
```

默认只发送一次并退出，避免持续覆盖人工调试输入。

使用命令：

```bash
ros2 run rebotarm_vision rebotarm_send_grasp_preview
```

如果需要发送 `grasp_pose` 而不是 `pregrasp_pose`：

```bash
ros2 run rebotarm_vision rebotarm_send_grasp_preview --ros-args -p pose_mode:=grasp
```

当前仅建议用于 MoveIt 预览测试；真实抓取执行仍需先确认 `grasp_tcp` 偏移和夹爪方向。

## 2026-05-24 MoveIt 仿真预览 joint_states 问题修复

现象：使用 `use_moveit_preview:=true` 且 `use_hardware:=false` 启动交互系统时，`move_group` 等待 `/rebotarm/joint_states` 超时并崩溃。

根因：硬件关闭时没有 `reBotArmController` 发布 `/rebotarm/joint_states`；而旧 launch 中仿真 `joint_state_publisher` 只在 `use_moveit_preview:=false` 时启动，导致 MoveIt 预览模式下没有当前关节状态。

修复：`interactive_system.launch.py` 中的 `joint_state_publisher` 现在按 `use_hardware:=false` 启动，并 remap：

```text
/joint_states -> /rebotarm/joint_states
```

验证结果：

- `rebotarm_bringup` 编译通过。
- 仿真预览启动时 `/rebotarm/joint_states` 能收到消息。
- `move_group` 不再出现 `Unable to configure planning scene monitor`，并能进入 `You can start planning now!`。

## 2026-05-24 抓取规划默认姿态调整

验证结果显示，同一 `pregrasp` 位置下：

- `orientation=(0,0,0,1)`：MoveIt IK 可达。
- `orientation=(0,0.7071,0,0.7071)`：MoveIt IK 返回 `error_code=-31`，不可达。

因此当前抓取规划节点已从 top-down 默认姿态切换为保守可达姿态：

```text
pregrasp 位置仍然是 grasp 上方 0.08m
pregrasp 姿态默认使用 identity quaternion
```

实现上已将“退避方向”和“末端姿态方向”解耦：

```yaml
grasp_plan.pregrasp_offset_axis: [0.0, 0.0, -1.0]
grasp_plan.tcp_approach_axis: [1.0, 0.0, 0.0]
grasp_plan.tcp_open_axis: [0.0, 1.0, 0.0]
```

验证：

- `rebotarm_vision` 编译通过。
- `tests/test_grasp_plan_converter.py` 与 `tests/test_grasp_preview_sender.py` 共 8 个测试通过。
- 直接校验输出：

```text
pregrasp 0.226 -0.238 0.329
quat 0.0 0.0 0.0 1.0
```
