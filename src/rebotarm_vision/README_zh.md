# reBotArm 视觉链路使用说明

本文记录当前阶段的视觉链路方案、启动方式、验证方法和常见问题。

## 1. 当前结论

VMware Ubuntu 直接直通 Gemini2 时，彩色流会出现纯灰或 MJPG 坏帧。当前采用折中方案：

```text
Gemini2 摄像头
-> Windows 主机采集彩色图
-> Windows 主机运行 YOLO
-> HTTP 提供 snapshot / MJPEG / detections.json
-> Ubuntu 虚拟机中的 rebotarm_vision_node 接收
-> 发布 ROS2 topic
```

Ubuntu 侧不再直接访问 Gemini2，也不再默认在 Ubuntu 内运行 YOLO。

## 2. 当前默认配置

配置文件：

```text
~/robotarm_ros2/src/rebotarm_vision/config/camera.yaml
```

关键参数：

```yaml
camera.type: network_mjpeg
camera.color_width: 1280
camera.color_height: 720
camera.color_fps: 30
camera.network_snapshot_url: http://192.168.145.1:8081/snapshot.jpg
camera.network_stream_url: http://192.168.145.1:8081/video.mjpg
camera.network_detections_url: http://192.168.145.1:8081/detections.json
ros.loop_rate_hz: 15.0
ros.enable_detection: false
ros.enable_network_detection: true
```

含义：

- Windows 采集 Gemini2 彩色图，目标为 1280x720 / 30 FPS
- Windows 运行 YOLO，并输出 JSON 检测结果
- Ubuntu ROS2 节点以 15 Hz 拉取图像和检测结果
- Ubuntu 内置 YOLO 已关闭，避免虚拟机 CPU 推理过慢

## 3. Windows 端启动

先确保 Gemini2 连接在 Windows 主机，不要连接到 VMware 虚拟机。

在 Windows 中确认：

- Orbbec Viewer 能看到 Gemini2 彩色画面
- 设备管理器中有 `Orbbec Gemini 2 RGB Camera`
- 当前已确认 Gemini2 RGB 的 OpenCV 索引为 `3`

在你的 Windows YOLO 虚拟环境中启动：

```powershell
cd D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main

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
  --model-path tools\yolo26s-seg.pt `
  --yolo-device 0 `
  --detection-fps 15 `
  --allowed-classes bottle
```

如果当前环境暂时不能使用 CUDA，可把设备改成：

```powershell
--yolo-device cpu
```

注意：

- `--model-path` 必须是 Windows 能访问的路径
- 不要写 Ubuntu 路径，例如 `/home/u24/...`
- `--capture-source orbbec` 会同时提供彩色图、深度图和相机内参
- `--allowed-classes bottle` 会只发布瓶子检测结果，避免预览抓错目标

## 4. Windows 端验证

浏览器打开：

```text
http://127.0.0.1:8081/health
http://127.0.0.1:8081/snapshot.jpg
http://127.0.0.1:8081/video.mjpg
http://127.0.0.1:8081/annotated.mjpg
http://127.0.0.1:8081/detections.json
http://127.0.0.1:8081/depth.png
http://127.0.0.1:8081/camera_info.json
```

通过标准：

- `health` 显示 Orbbec 相机已打开，深度状态为 `ok`
- `snapshot.jpg` 是 Gemini2 原始彩色图
- `video.mjpg` 是 Gemini2 原始视频流
- `annotated.mjpg` 是 YOLO 标注后的画面
- `detections.json` 中 `detections` 数组能随画面变化更新
- `depth.png` 返回 16 位 PNG 深度图
- `camera_info.json` 返回 Gemini2 当前分辨率下的相机内参

如果 `health` 中出现：

```text
Invalid CUDA device
```

说明当前 Python 的 PyTorch 没有 CUDA。先使用：

```powershell
--yolo-device cpu
```

## 5. Ubuntu 端启动

在 Ubuntu 虚拟机中：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /home/u24/venvs/rebotarm_vision/bin/activate

ros2 launch rebotarm_vision vision.launch.py
```

## 6. Ubuntu 端验证

先验证能访问 Windows 图像：

```bash
python3 - <<'PY'
from urllib.request import urlopen
import cv2
import numpy as np

data = urlopen("http://192.168.145.1:8081/snapshot.jpg", timeout=5).read()
img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
print(img.shape, img.mean(), img.std())
PY
```

通过标准：

```text
(720, 1280, 3)
std 明显大于 0
```

检查 ROS 图像：

```bash
ros2 topic echo /camera/color/image_raw --once | grep -E "height|width|encoding|step"
```

通过标准：

```text
height: 720
width: 1280
encoding: bgr8
step: 3840
```

检查检测结果：

```bash
ros2 topic hz /grasp/detections
ros2 topic echo /grasp/detections --once
ros2 topic echo /camera/depth/image_raw --once | grep -E "height|width|encoding|step"
ros2 topic echo /grasp/plan --once
```

当前已验证 `/grasp/detections` 可达到约 15 Hz，检测框来自 Windows YOLO；`/camera/depth/image_raw` 来自 Windows Orbbec 深度转发；`/grasp/plan` 由 ordinary grasp 节点输出。

## 7. 当前 ROS2 输出

当前视觉节点发布：

```text
/camera/color/image_raw
/camera/color/annotated
/camera/depth/image_raw
/grasp/detections
/grasp/plan
/grasp/pregrasp_pose
/grasp/grasp_pose
```

说明：

- `/camera/color/image_raw` 来自 Windows 的 `snapshot.jpg`
- `/camera/depth/image_raw` 来自 Windows 的 `depth.png`
- `/grasp/detections` 来自 Windows 的 `detections.json`
- `/camera/color/annotated` 由 Ubuntu 根据 detections 重新绘制，主要用于 ROS 内部调试
- `/grasp/plan` 是旧版 ordinary grasp 算法封装后的抓取结果

## 8. 视觉抓取总启动和执行检查

如果要把视觉、MoveIt2 预览、RViz、抓取预览发送节点一起启动：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=simulation \
  start_visual_grasp_executor:=false
```

确认 RViz 中目标安全后，再切实机执行模式：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=real \
  start_visual_grasp_executor:=false
```

常用检查：

```bash
ros2 topic echo /grasp/plan --once
ros2 service list | grep -E "motion_execution|trajectory_stop|disable|gripper"
```

执行当前预抓取目标：

```bash
ros2 service call /rebotarm/motion_execution/execute_pose rebotarm_msgs/srv/ExecutePose "{pose: {header: {frame_id: base_link}, pose: {orientation: {w: 1.0}}}}"
```

完整靠近夹取流程使用独立服务。启动时关闭持续预览发送节点，并显式开启完整抓取执行节点：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=simulation \
  start_grasp_preview:=false \
  start_visual_grasp_executor:=true
```

测试：

```bash
ros2 topic echo /grasp/plan --once
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

实机时把 `execution_mode:=simulation` 改成 `execution_mode:=real`。完整抓取当前是半闭环联调版：移动阶段会等待新的 MoveIt preview ready，但阶段切换仍依赖执行接受结果和保守等待时间，不等价于强到位闭环。

停止和失能：

```bash
ros2 service call /rebotarm/interactive_control/stop std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

夹爪测试：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.09, max_effort: 0.3}"
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.025, max_effort: 0.3}"
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.015, max_effort: 0.3}"
```

`execution_mode:=simulation` 只测试视觉、TF、MoveIt IK 和预览链路，不会让实机运动；`execution_mode:=real` 只有调用 `execute_preview` 后才会运动。危险情况优先使用实体急停或断电。

### 8.1 V1.1 严格 20 次稳定性测试

当前视觉抓取已经可以初步稳定夹取后，下一步要验证的是：

```text
同一个 visual_ready 初始姿态
+ 人工重新摆放同一个瓶子
+ 每次抓取前人工确认安全
=> 连续 20 次抓取成功率和失败阶段
```

启动视觉抓取系统后，打开一个新终端运行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark \
  --attempts 20 \
  --service-timeout-sec 180 \
  --return-ready-before-each \
  --wait-enter
```

每一轮测试流程：

```text
1. 程序先调用 /rebotarm/visual_ready/move，让机械臂回到固定 visual_ready 姿态
2. 手动松开夹爪，并把瓶子摆到测试位置
3. 确认机械臂周围安全后按 Enter
4. 程序调用 /rebotarm/visual_grasp/execute 执行一次抓取
5. 自动记录本次 success/fail 和 failed_stage
6. 进入下一轮
```

最终统计示例：

```text
total=20 success=17 failed=3 success_rate=85.0%
failed_stage:
  close_gripper: 2
  move_to_pregrasp: 1
```

失败时看启动终端中的标准化日志：

```text
[visual_grasp][run=...][attempt=...][candidate=...][stage=plan]
[visual_grasp][run=...][attempt=...][candidate=...][stage=pregrasp_pose]
[visual_grasp][run=...][attempt=...][candidate=...][stage=grasp_pose]
[visual_grasp][run=...][attempt=...][candidate=...][stage=move_to_pregrasp] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=approach_grasp] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=close_gripper] start/ok/fail
[visual_grasp][run=...][attempt=...][candidate=...][stage=lift] start/ok/fail
```

失败摘要会额外包含：

```text
failed_stage
failure_message
pregrasp_pose
grasp_pose
jaw_width
last_gripper_reached_position
contact
closure_distance
```

判断规则：

- 如果失败集中在 `move_to_pregrasp`，优先排查 MoveIt 规划、起始姿态、目标可达性。
- 如果失败集中在 `approach_grasp`，优先排查 TCP 偏移、grasp 点和接近方向。
- 如果失败集中在 `close_gripper`，优先排查 `jaw_width`、夹爪接触判断和夹爪力度。
- 如果失败集中在 `lift`，优先排查是否夹稳、保持力和 lift 高度。

20 次成功率低于 80% 时，先不要继续增加复杂算法，先根据失败阶段定位 TCP、手眼 TF、深度稳定性或夹爪接触判断。

## 9. 常见问题

### 9.1 打开的是电脑自带摄像头

现象：

- 画面不是 Gemini2 视角
- 看到的是电脑前置摄像头画面

处理：

- 当前 Gemini2 RGB 的索引是 `3`
- Windows 启动命令使用 `--camera-index 3`

当前正式抓取路线推荐 `--capture-source orbbec`，通常不需要手动指定 `camera-index`。只有回退到 `--capture-source opencv` 时才需要检查摄像头索引。

### 9.2 Windows 看不到 Gemini2

现象：

- Orbbec Viewer 看不到 Gemini2
- Windows 设备管理器中没有 Orbbec Gemini 2 RGB Camera

处理：

- 在 VMware 中断开 Gemini2，不要让虚拟机占用 USB 设备
- 让 Gemini2 连接到 Windows 主机
- 重新打开 Orbbec Viewer 检查彩色图

### 9.3 画面变成全黑

当前脚本会检测 `frame.std() < 1.0` 的空/黑帧，并尝试重开相机。

如果仍持续全黑：

- 关闭 Orbbec Viewer
- 停止旧的 `python tools\windows_mjpeg_server.py`
- 重新启动 Windows 服务
- 确认 `health` 中显示 `backend=dshow opened=True`

### 9.4 Ubuntu 里访问 snapshot 失败

检查 Windows 服务是否启动：

```text
http://127.0.0.1:8081/health
```

检查 Ubuntu 能否访问：

```bash
curl http://192.168.145.1:8081/health
```

如果访问失败，检查：

- Windows 防火墙
- VMware NAT 网段是否仍为 `192.168.145.x`
- Windows 主机 VMnet8 地址是否仍为 `192.168.145.1`

### 9.5 detections 为空

可能原因：

- 模型路径错误
- 类别提示不匹配
- 画面里没有目标
- YOLO 虚拟环境没有正确激活
- `--yolo-device` 指向了不可用的 GPU

## 10. 当前进度

已完成：

- Windows Gemini2 彩色图采集
- Windows MJPEG / snapshot 服务
- Windows YOLO JSON 输出接口
- Windows annotated MJPEG 输出接口
- Windows 16 位深度图 `/depth.png` 和相机内参 `/camera_info.json` 转发
- Ubuntu ROS2 网络图像接收
- Ubuntu ROS2 网络检测结果接收
- Ubuntu ROS2 深度图 `/camera/depth/image_raw` 发布
- ordinary grasp ROS2 封装输出 `/grasp/plan`
- `rebotarm_send_grasp_preview` 把 `/grasp/plan` 转成 `/rebotarm/interactive_control/pose_target`
- MoveIt2 / RViz 预览链路
- `execution_mode:=simulation` 和 `execution_mode:=real` 分离
- `/rebotarm/interactive_control/stop`、`/rebotarm/trajectory_stop`、`/rebotarm/disable` 停止/失能测试入口
- `/rebotarm/gripper/set` 夹爪开合测试入口

当前仍需继续优化：

- 抓取姿态和瓶身接触位置还需要继续实物微调。
- `target_base_offset_xyz`、`base_z_offset_m`、`min_target_z_m` 仍属于现场安全偏移参数。
- 真正的 `move to pregrasp -> 慢速接近 grasp -> close gripper -> lift` 闭环还需要在预抓取可稳定命中后继续接入。

## 11. 抓取算法主线

当前正式路线已经切换为 ordinary grasp ROS 封装，不再保留早期简化候选链路。

```text
/grasp/detections + /camera/depth/image_raw
-> rebotarm_ordinary_grasp_node
-> /grasp/plan
-> /grasp/pregrasp_pose
-> /grasp/grasp_pose
```

早期的 2D/3D candidate 节点只用于阶段性验证，已经从正式源码入口中移除。

## 13. Windows 深度图转发

当前 Windows 服务脚本已经支持两种采集方式：

- `--capture-source opencv`：只通过 OpenCV 读取彩色摄像头，适合原来的 YOLO 折中方案，没有深度图。
- `--capture-source orbbec`：通过 `pyorbbecsdk` 读取 Gemini2 彩色图和深度图，推荐用于后续 3D 抓取。

Orbbec 模式启动示例：

```powershell
python tools\windows_mjpeg_server.py `
  --capture-source orbbec `
  --host 0.0.0.0 `
  --port 8081 `
  --width 1280 `
  --height 720 `
  --fps 30 `
  --depth-width 1280 `
  --depth-height 720 `
  --depth-fps 30 `
  --model-path tools\yolo26s-seg.pt `
  --yolo-device 0 `
  --detection-fps 15 `
  --allowed-classes bottle
```

新增 HTTP 接口：

```text
/depth.png
/camera_info.json
```

Ubuntu 侧 `network_mjpeg` 驱动会读取：

```text
camera.network_depth_url: http://192.168.145.1:8081/depth.png
```

验证命令：

```bash
curl -I http://192.168.145.1:8081/depth.png
curl http://192.168.145.1:8081/camera_info.json
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /grasp/plan --once
```

注意：Windows 运行环境需要能 `import pyorbbecsdk`。当前全局 Python 没有该库，因此这一步需要在你的 Orbbec/YOLO 虚拟环境里启动后再实机验证。

## 14. 深度链路实测结果

当前 Windows 深度服务已经实测可用，测试端口为 `8081`。

服务命令：

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

实测结果：

- `pyorbbecsdk` 可以枚举到 1 台 Gemini2。
- `/health` 返回 `camera=orbbec opened=True align=HW_MODE; depth=ok`。
- `/depth.png` 返回 16 位 PNG 深度图。
- `/camera_info.json` 返回真实内参。
- Ubuntu 能访问 `http://192.168.145.1:8081/depth.png`。
- ROS `/camera/depth/image_raw` 能发布 `mono16 1280x720`。
- `/grasp/plan` 当前由 ordinary grasp 节点输出。

当前还没有在 `orbbec_yolo` 环境里启用 YOLO，后续需要安装 `ultralytics/torch` 或把检测继续放在已有 YOLO 环境中。

## 15. 完整视觉链路当前状态（2026-05-22）

当前已经在 Windows `orbbec_yolo` 环境中验证同一服务同时输出：

```text
/snapshot.jpg
/video.mjpg
/annotated.mjpg
/detections.json
/depth.png
/camera_info.json
/health
```

当前服务命令：

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
  --model-path tools\yolo26s-seg.pt `
  --yolo-device 0 `
  --detection-fps 15
```

Ubuntu `camera.yaml` 当前指向：

```yaml
camera.network_snapshot_url: http://192.168.145.1:8081/snapshot.jpg
camera.network_stream_url: http://192.168.145.1:8081/video.mjpg
camera.network_depth_url: http://192.168.145.1:8081/depth.png
camera.network_detections_url: http://192.168.145.1:8081/detections.json
ros.loop_rate_hz: 15.0
ros.enable_network_detection: true
```

Gemini2 当前内参已经写入 `rebotarm_ordinary_grasp_node`：

```yaml
ordinary_grasp.fx: 692.562744140625
ordinary_grasp.fy: 692.2272338867188
ordinary_grasp.cx: 641.2417602539062
ordinary_grasp.cy: 361.8166198730469
```

本次 ROS2 联调结果：

- `/grasp/detections`：有实时 YOLO 检测，约 `15Hz`。
- `/camera/depth/image_raw`：有 Gemini2 深度图，`mono16 1280x720`。
- `/grasp/plan`：由旧 `ordinary_grasp.py` 路线生成。
- 日志未发现 `Traceback`、`process has died`、QoS 不兼容或深度异常。

注意：当前正式来源应是 `ordinary_grasp_mask_depth` 或 `ordinary_grasp_obb_depth`，不再使用早期简化 candidate 链路。

## 16. 抓取目标与 TCP 标定（2026-05-25）

当前抓取目标过滤在 Windows YOLO 服务和检测结果侧完成。ROS2 内部正式抓取主线是：

```text
ordinary grasp -> /grasp/plan -> MoveIt2
```

当前偏差收敛重点是 `end_link -> grasp_tcp`：

```yaml
rebotarm_grasp_tcp_frame:
  ros__parameters:
    tcp_offset_xyz: [0.0, 0.0, 0.0]

rebotarm_grasp_preview_sender:
  ros__parameters:
    tcp_offset_xyz: [0.0, 0.0, 0.0]
```

标定后两个位置的 `tcp_offset_xyz` 要保持一致。

## 17. Eye-in-hand 外参配置

Gemini2 安装在夹爪附近时，需要一条固定 TF：

```text
end_link -> camera_depth_frame
```

当前外参配置文件：

```text
config/handeye.yaml
```

`vision.launch.py` 启动时会读取该文件，并自动启动 `tf2_ros static_transform_publisher`。这表示标定结果可以长期保存；每次启动只是重新发布 TF，不需要重新标定。

当前 `handeye.yaml` 已经迁移旧版 `orbbec_gemini2/hand_eye.npz` 的 TSAI eye-in-hand 标定结果。旧版保存的是 `T_cam2gripper`，当前配置保存的是其逆变换 `end_link -> camera_depth_frame`：

```yaml
translation:
  x: 0.005668446
  y: 0.034519527
  z: 0.056490805
rotation:
  x: 0.567049781
  y: -0.565885703
  z: 0.435347452
  w: 0.410731680
```

验证命令：

```bash
ros2 run tf2_ros tf2_echo end_link camera_depth_frame
```

如果后续重新做手眼标定，只需要更新 `config/handeye.yaml` 的平移和四元数。相机或夹爪结构没有移动时，不需要重复标定。

## 18. `grasp_tcp` TF

当前新增调试/标定节点：

```bash
ros2 run rebotarm_vision rebotarm_grasp_tcp_frame
```

用途：发布 `end_link -> grasp_tcp`，在 RViz 里观察真实夹爪中心与 `end_link` 的偏移。

验证命令：

```bash
ros2 run tf2_ros tf2_echo end_link grasp_tcp
```

当 `grasp_tcp` 与真实夹爪中心重合后，再执行 `pregrasp -> grasp -> close gripper -> lift`。
