# reBotArm ROS2 交互预览使用说明

这份文档记录当前已经验证可用的 `RViz + MoveIt 预览 + Interactive Marker` 使用方式。

## 0. 视觉抓取一键联调流程（Windows Gemini2 + ROS2 ordinary grasp）

这一节是当前视觉抓取路线的复制粘贴版，适合给别人快速测试。当前路线是：

```text
Windows Gemini2 + YOLO/depth HTTP 服务（8081）
-> Ubuntu ROS2 rebotarm_vision
-> 旧版 ordinary grasp 算法封装节点
-> /grasp/plan
-> MoveIt2 预览
-> 可选实机执行
```

### 0.1 Windows 端启动 Gemini2 + YOLO + 深度服务

在 Windows PowerShell 中执行。Gemini2 要连接到 Windows 主机，不要被 VMware 虚拟机占用。

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

Windows 本机浏览器检查：

```text
http://127.0.0.1:8081/health
http://127.0.0.1:8081/snapshot.jpg
http://127.0.0.1:8081/annotated.mjpg
http://127.0.0.1:8081/detections.json
http://127.0.0.1:8081/depth.png
http://127.0.0.1:8081/camera_info.json
```

Ubuntu 侧检查能否访问 Windows 服务：

```bash
curl http://192.168.145.1:8081/health
curl http://192.168.145.1:8081/detections.json
curl -I http://192.168.145.1:8081/depth.png
curl http://192.168.145.1:8081/camera_info.json
```

### 0.2 Ubuntu 端加载 ROS2 环境

每个新的 Ubuntu 终端都先执行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果单独运行视觉节点或调试视觉 Python 代码，再激活视觉虚拟环境：

```bash
source /home/u24/venvs/rebotarm_vision/bin/activate
```

### 0.3 安全预览启动（默认推荐）

先用 `simulation`。它会计算视觉抓取点、TF、MoveIt IK 和 RViz 预览，但不会真的给机械臂发轨迹。

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=simulation
```

说明：

- `execution_mode:=simulation` 是安全默认值。
- `use_hardware:=true` 表示读取实机状态和控制器链路，但 simulation 模式下 `execute_preview` 不会执行实机轨迹。
- 不要写 `execution_mode:=true`，这是无效参数。

### 0.4 实机执行启动

只有在 RViz 中确认预抓取目标安全、不会撞物体或桌面后，再切到 real：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=real
```

注意：启动 `real` 本身不会让机械臂运动，只有调用 `execute_preview` 后才会执行当前预览目标。

### 0.5 检查话题、服务和抓取计划

```bash
ros2 node list
ros2 service list | grep -E "interactive_control|trajectory_stop|disable|gripper"
ros2 topic echo /grasp/plan --once
ros2 topic echo /rebotarm/interactive_control/pose_target --once
ros2 topic echo /rebotarm/interactive_control/preview --once
```

当前应能看到这些关键服务：

```text
/rebotarm/interactive_control/execute_preview
/rebotarm/interactive_control/set_mode
/rebotarm/interactive_control/stop
/rebotarm/trajectory_stop
/rebotarm/disable
/rebotarm/gripper/set
```

当前不再推荐使用软件锁存式 `estop/reset_estop`；危险情况优先用实体急停或断电。

### 0.6 执行当前预抓取目标

确认 RViz 预览安全后执行：

```bash
ros2 service call /rebotarm/interactive_control/execute_preview std_srvs/srv/Trigger "{}"
```

通过标准：

- simulation 模式：流程返回成功或预览执行成功，但机械臂不动。
- real 模式：机械臂向 `/rebotarm/interactive_control/pose_target` 对应的预抓取位姿运动。

### 0.7 完整靠近夹取流程

当前已经提供完整视觉抓取执行服务：

```text
/rebotarm/visual_grasp/execute
```

它执行的顺序是：

```text
move to pregrasp
-> slow/保守等待后 approach grasp
-> close gripper
-> lift
```

为了避免持续预览节点反复把 `pregrasp` 写回 `/rebotarm/interactive_control/pose_target`，测试完整抓取时建议这样启动：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=simulation \
  start_grasp_preview:=false \
  start_visual_grasp_executor:=true
```

确认 `/grasp/plan` 正常后，先在 simulation 模式测试：

```bash
ros2 topic echo /grasp/plan --once
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

确认 RViz 和 simulation 流程安全后，再切 real：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  use_local_rviz:=true \
  execution_mode:=real \
  start_grasp_preview:=false \
  start_visual_grasp_executor:=true
```

实机执行：

```bash
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

完整抓取停止：

```bash
ros2 service call /rebotarm/visual_grasp/stop std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

如果 `/rebotarm/interactive_control/stop` 在 MoveIt 规划或 `execute_preview` 回调期间响应慢，优先调用 `/rebotarm/trajectory_stop`。它直接到控制器层请求停止当前轨迹，比等待交互执行节点更直接。

完整抓取当前默认参数：

```text
pregrasp_base_z_offset_m: 0.05
grasp_base_z_offset_m: 0.0
lift_z_m: 0.08
close_position_m: 0.025
close_max_effort: 0.3
```

说明：第一版完整抓取会等待每个阶段的新 MoveIt preview ready 后再调用执行，但 `execute_preview` 返回仍只代表轨迹已接受，不代表机械臂已到位；阶段之间仍有保守等待时间，所以它是半闭环联调版，不是最终强闭环。第一次实机测试必须手放实体急停旁边，并把瓶子周围清空。

### 0.8 停止、底层停止和失能

正常停止当前交互执行：

```bash
ros2 service call /rebotarm/interactive_control/stop std_srvs/srv/Trigger "{}"
```

底层直接请求停止轨迹：

```bash
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

更强的软件失能：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

语义说明：

- `stop`：让上层交互执行停止，并请求底层停止继续追踪当前轨迹；不是暂停/继续，也不是精确保持在某一个插补点。
- `trajectory_stop`：底层轨迹停止接口，保留给上层调用或调试时直接调用。
- `disable`：失能电机，力度更强，但机械臂可能不再保持力。
- 真正危险时不要依赖 ROS 命令，直接按实体急停或切断机械臂电源。

### 0.9 夹爪开合测试

先张开夹爪：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.09, max_effort: 0.3}"
```

轻夹瓶身：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.025, max_effort: 0.3}"
```

更紧一点：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.015, max_effort: 0.3}"
```

谨慎测试完全闭合目标：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper "{position: 0.0, max_effort: 0.3}"
```

瓶子测试先用 `max_effort: 0.3`，不要一开始就用 `1.0` 或 `1.5`。

### 0.10 当前抓取偏移参数

`visual_grasp_system.launch.py` 当前默认把 ordinary grasp 的预抓取结果转换到 `base_link` 后，再做这些修正：

```text
tcp_offset_xyz: [-0.04, 0.0, 0.0]
target_base_offset_xyz: [0.0, 0.01, 0.0]
base_z_offset_m: 0.05
min_target_z_m: 0.16
```

含义：

- `tcp_offset_xyz`：`end_link` 到夹爪中心的 TCP 偏移。
- `target_base_offset_xyz`：在 `base_link` 世界坐标下对最终目标做固定平移；当前 `[0.0, 0.01, 0.0]` 用于补偿横向约 1 cm。
- `base_z_offset_m`：整体抬高目标，减少撞瓶/撞桌风险。
- `min_target_z_m`：限制最低目标高度。

## 1. 日常启动

在 Ubuntu 终端进入工作区后执行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=false \
  use_local_rviz:=true
```

参数说明：

- `use_moveit_preview:=true`：预览走 MoveIt IK
- `use_hardware:=false`：当前不接实机
- `use_local_rviz:=true`：启动外层 RViz

## 2. 启动成功标志

终端里重点看这些日志：

- `preview node ready: namespace=/rebotarm, backend=moveit`
- `execution node ready: namespace=/rebotarm`
- `marker server ready: namespace=/rebotarm`
- `Successfully loaded planner 'OMPL'`
- `You can start planning now!`

## 3. 正常节点状态

可用下面命令检查：

```bash
ros2 node list | sort | uniq -c
```

关键节点应至少包含：

- `/marker_server`
- `/preview_node`
- `/execution_node`
- `/rviz2`
- `/robot_state_publisher`
- `/static_transform_publisher`

说明：

- `move_group` 可能显示多个名字，这是 MoveIt 单进程内部多个 ROS node handle 的表现
- 只要 `marker_server / preview_node / execution_node / rviz2` 都是单实例，交互主链通常就是正常的

## 4. RViz 侧验收

启动后在 RViz 中确认：

- RViz 窗口正常弹出
- 机械臂模型能显示
- 能看到 `reBotArm EE Target` 交互 marker
- 鼠标拖动 marker 时有响应

## 5. Topic 检查

```bash
ros2 topic list | grep interactive
```

正常会看到：

- `/rebotarm/interactive_control/ee_target/feedback`
- `/rebotarm/interactive_control/ee_target/update`
- `/rebotarm/interactive_control/pose_target`
- `/rebotarm/interactive_control/preview`
- `/rebotarm/interactive_control/status`

## 6. 可达目标测试

先开一个终端监听预览：

```bash
ros2 topic echo /rebotarm/interactive_control/preview
```

再开另一个终端发送一个可达点：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 0.25, y: 0.00, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

通过标准：

- `/preview` 有更新
- `reachable: true`
- 消息类似 `moveit ik preview ready`

## 7. 不可达目标测试

发送一个明显超出工作空间的目标：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 1.20, y: 0.00, z: 1.20}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

通过标准：

- `/preview` 有更新
- `reachable: false`
- 消息类似 `target pose unreachable...`
- 不能假成功成 `reachable: true`

## 8. 常用状态查看

```bash
ros2 topic echo /rebotarm/interactive_control/status
ros2 topic echo /rebotarm/interactive_control/preview
```

## 9. 出问题时先清理旧进程

如果怀疑旧节点残留，先清理：

```bash
pkill -f interactive_system.launch.py
pkill -f demo.launch.py
pkill -f move_group
pkill -f rviz2
pkill -f MarkerServerNode
pkill -f PreviewNode
pkill -f ExecutionNode
pkill -f joint_state_publisher
pkill -f robot_state_publisher
pkill -f static_transform_publisher
```

然后重新：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

再执行启动命令。

## 10. 当前版本状态

当前已经验证通过的能力：

- RViz 正常启动
- Interactive Marker 正常显示
- MoveIt 预览正常
- 可达/不可达判断正常
- 交互主链为单实例运行

## 历史记录说明

下面从 `Gemini2 + Windows YOLO 折中方案使用记录（2026-05-21）` 开始的内容是早期阶段性记录，里面可能出现旧端口 `8080`、OpenCV `camera-index`、`/grasp/candidates -> /grasp/candidates_3d` 等已不作为正式入口的测试方式。

当前给别人复制粘贴测试时，请优先使用本文最上面的 `0. 视觉抓取一键联调流程（Windows Gemini2 + ROS2 ordinary grasp）`。正式路线现在是 `Windows 8081 -> /grasp/detections + /camera/depth/image_raw -> rebotarm_ordinary_grasp_node -> /grasp/plan -> MoveIt2/interactive_control`。

## Gemini2 + Windows YOLO 折中方案使用记录（2026-05-21）

当前建议先使用折中方案：Gemini2 留在 Windows 主机上采集彩色图并运行 YOLO，Ubuntu/ROS2 只通过网络读取图片和检测结果。这样可以绕过 VMware 直通 Gemini2 时彩色图变灰、帧率很低的问题。

Windows 侧启动示例：

```powershell
cd D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main
python tools\windows_mjpeg_server.py `
  --backend dshow `
  --camera-index 3 `
  --host 0.0.0.0 `
  --port 8080 `
  --width 1280 `
  --height 720 `
  --fps 30 `
  --model-path tools\yolo26s-seg.pt `
  --yolo-device cpu `
  --detection-fps 5
```

说明：

- 当前 Gemini2 RGB 在 Windows 里验证到的索引是 `3`，`0` 是电脑自带摄像头或其他摄像头。
- 如果使用你自己的 CUDA YOLO 虚拟环境，可以把 `--yolo-device cpu` 改成 `--yolo-device 0`；全局 `D:\esp\python.exe` 目前是 CPU 版 torch，不能使用 `--yolo-device 0`。
- `/snapshot.jpg` 是 ROS2 读取的单帧图像，`/detections.json` 是 Windows YOLO 输出的检测结果，`/health` 用于检查服务状态。
- Ubuntu 侧配置已指向 `http://192.168.145.1:8080/snapshot.jpg` 和 `http://192.168.145.1:8080/detections.json`。

Ubuntu 侧验证：

```bash
curl http://192.168.145.1:8080/health
curl http://192.168.145.1:8080/detections.json

cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_vision vision.launch.py
```

再开一个终端检查 ROS 输出：

```bash
ros2 topic hz /grasp/detections
ros2 topic echo /grasp/detections --once
```

本次已验证结果：Windows 能识别 Orbbec Gemini 2 RGB/IR/Depth/Data Channel；`camera-index 3` 用 DSHOW 能拿到正常 Gemini2 彩色画面；Ubuntu 能访问 Windows 服务；ROS2 `/grasp/detections` 可稳定发布，实测约 15Hz。

## 2D 抓取候选点使用记录（2026-05-21）

当前已经增加 `rebotarm_grasp_candidate_node`。它订阅：

```text
/grasp/detections
```

并发布：

```text
/grasp/candidates
```

第一版候选点仍然是 2D 像素坐标，不是机械臂 3D 坐标：

- `pose.position.x`：检测框中心像素 `u`
- `pose.position.y`：检测框中心像素 `v`
- `pose.position.z`：固定为 `0`
- `jaw_width`：检测框宽度，单位是像素
- `object_length`：检测框高度，单位是像素
- `source`：`2d_detection_pixel`
- `best_index`：当前置信度最高的候选目标索引

启动方式仍然使用视觉 launch：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_vision vision.launch.py
```

验证：

```bash
ros2 topic echo /grasp/candidates --once
ros2 topic hz /grasp/candidates
```

本次实测结果：`/grasp/candidates` 能正常输出 `chair/tv/laptop` 等检测目标生成的候选点，发布频率约 `14.8-15Hz`。

## 3D 抓取候选点预留链路（2026-05-22）

当前已经增加 `rebotarm_grasp_3d_node`。它订阅：

```text
/grasp/candidates
/camera/depth/image_raw
```

并发布：

```text
/grasp/candidates_3d
```

转换公式使用针孔相机模型：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth_mm * depth_scale_m
```

配置位置：

```text
~/robotarm_ros2/src/rebotarm_vision/config/camera.yaml
```

关键参数：

```yaml
rebotarm_grasp_3d_node:
  ros__parameters:
    grasp_3d.input_candidates_topic: /grasp/candidates
    grasp_3d.input_depth_topic: /camera/depth/image_raw
    grasp_3d.output_topic: /grasp/candidates_3d
    grasp_3d.output_frame_id: camera_depth_frame
    grasp_3d.fx: 500.0
    grasp_3d.fy: 500.0
    grasp_3d.cx: 640.0
    grasp_3d.cy: 360.0
    grasp_3d.depth_scale_m: 0.001
    grasp_3d.depth_window_px: 2
```

注意：上面的 `fx/fy/cx/cy` 目前是占位值，后续要替换成 Gemini2 对应分辨率下的真实内参。现在 Windows 折中方案还没有转发深度图，所以 `/grasp/candidates_3d` 只有在收到 `/camera/depth/image_raw` 后才会发布。

本次校验：

- 单元测试验证 `u=740, v=410, depth=1000mm` 可以转换为 `x=0.2, y=0.1, z=1.0`。
- ROS 冒烟验证 `rebotarm_grasp_3d_node` 能订阅假深度图和候选点并发布 `/grasp/candidates_3d`。
- 真实 `vision.launch.py` 启动后，2D `/grasp/candidates` 仍保持约 `15Hz`。

## Windows Orbbec 深度转发（2026-05-22）

当前 `tools/windows_mjpeg_server.py` 已新增 Orbbec SDK 模式。这个模式直接在 Windows 主机读取 Gemini2 彩色图和深度图，并继续运行 YOLO。

Windows 侧启动命令：

```powershell
cd D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main

python tools\windows_mjpeg_server.py `
  --capture-source orbbec `
  --host 0.0.0.0 `
  --port 8080 `
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

注意：

- 这个命令必须在已经安装 `pyorbbecsdk` 的 Windows Python/虚拟环境里运行。
- 如果暂时不用 GPU，可以把 `--yolo-device 0` 改成 `--yolo-device cpu`。
- `--capture-source opencv` 仍然保留，只适合彩色图和 YOLO；没有深度。

Windows 服务新增接口：

```text
http://127.0.0.1:8080/depth.png
http://127.0.0.1:8080/camera_info.json
```

Ubuntu 侧已经配置：

```yaml
camera.network_depth_url: http://192.168.145.1:8080/depth.png
```

验证：

```bash
curl -I http://192.168.145.1:8080/depth.png
curl http://192.168.145.1:8080/camera_info.json

ros2 topic hz /camera/depth/image_raw
ros2 topic echo /grasp/candidates_3d --once
```

本次已完成代码和测试校验：Ubuntu 能解码 16 位 PNG 深度图；旧 Windows OpenCV 服务没有深度时，2D `/grasp/candidates` 仍约 `15Hz` 且不报错。由于当前全局 Python 没有 `pyorbbecsdk`，Orbbec 真机深度接口需要在你的 Orbbec/YOLO 虚拟环境中启动后再做实机验证。

## Windows Orbbec 深度服务实测可用（2026-05-22）

当前可用的深度服务命令：

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
  --model-path= `
  --detection-fps 15
```

验证：

```powershell
curl http://127.0.0.1:8081/health
curl http://127.0.0.1:8081/camera_info.json
curl -o depth.png http://127.0.0.1:8081/depth.png
```

Ubuntu 验证：

```bash
curl -s -S http://192.168.145.1:8081/health
curl -o /tmp/depth_8081.png http://192.168.145.1:8081/depth.png
ros2 topic echo /camera/depth/image_raw --once
```

本次实测：

- Windows `/depth.png` 返回 HTTP 200。
- Ubuntu 能解码为 `uint16 720x1280` 深度图。
- ROS `/camera/depth/image_raw` 能发布 `mono16 1280x720`。
- 人工发布 2D candidate 后，`/grasp/candidates_3d` 能输出 3D 候选点。

注意：当前命令用 `--model-path=` 关闭了 YOLO，只验证彩色图和深度图。若要同一服务同时跑 YOLO，还需要在 `orbbec_yolo` 环境里安装并验证 `ultralytics/torch`。

## 15. Orbbec + YOLO + 深度完整链路实测通过（2026-05-22）

当前 `orbbec_yolo` 环境已经可以同时完成 Gemini2 彩色图、深度图和 YOLO 检测。Windows 服务运行在 `8081`：

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

Windows 本机验证：

```powershell
curl http://127.0.0.1:8081/health
curl http://127.0.0.1:8081/detections.json
curl -o depth.png http://127.0.0.1:8081/depth.png
```

通过标准：

- `/health` 返回 `camera=orbbec opened=True align=HW_MODE; depth=ok`。
- `/detections.json` 有检测结果，实测可识别 `chair/laptop/tv/keyboard/mouse/backpack` 等目标。
- `/depth.png` 返回 16 位 PNG 深度图。

Ubuntu 侧完整验证命令：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_vision vision.launch.py
```

另开终端检查：

```bash
ros2 topic echo /grasp/detections --once
ros2 topic echo /camera/depth/image_raw --once
ros2 topic echo /grasp/candidates_3d --once

ros2 topic hz /grasp/detections --window 5
ros2 topic hz /camera/depth/image_raw --window 5
ros2 topic hz /grasp/candidates_3d --window 5
```

本次实测结果：

- `/grasp/detections` 正常发布，频率约 `15Hz`。
- `/camera/depth/image_raw` 正常发布，格式为 `mono16 1280x720`。
- `/grasp/candidates_3d` 正常发布，频率约 `15Hz`。
- 3D 候选点中有 `valid: true` 的结果，`source=3d_depth_projection`，说明检测框已经成功结合深度图投影到相机坐标。
- 深度图 HTTP 拉取存在波动，短窗口实测约 `0.8-3.7Hz`；当前 3D 节点会使用最近一帧深度继续输出候选点。

当前阶段结论：视觉链路已经从“只能 2D 检测”推进到“Windows Gemini2 + YOLO + 深度，Ubuntu ROS2 生成 3D 抓取候选”。下一步重点不是再装环境，而是做目标筛选、坐标系标定和 MoveIt 抓取执行闭环。

## 16. 目标筛选与有效 3D 候选过滤（2026-05-22）

为了避免把椅子、电视、笔记本等不可抓目标送入后续执行链，当前已经在 `camera.yaml` 中开启类别白名单：

```yaml
rebotarm_grasp_candidate_node:
  ros__parameters:
    grasp.min_confidence: 0.35
    grasp.allowed_classes:
      - bottle
```

同时 3D 节点默认不发布无效深度候选：

```yaml
rebotarm_grasp_3d_node:
  ros__parameters:
    grasp_3d.publish_invalid_candidates: false
```

含义：

- `/grasp/candidates` 只保留白名单里的类别；当前只允许 `bottle`。
- `/grasp/candidates_3d` 只发布能拿到有效深度的 3D 候选点。
- 如果现场没有白名单目标，候选数组为空是正常结果。

本次实测：

- 当前白名单已收紧为只有 `bottle`，其他类别都会被过滤。
- 如果画面中没有被 YOLO 识别为 `bottle` 的目标，`/grasp/candidates` 和 `/grasp/candidates_3d` 为空是正常结果。
- `/grasp/candidates_3d` 没有再输出 `source=3d_depth_missing` 的无效候选。

如果要临时测试某个新目标，只需要把 YOLO 输出的 `class_name` 加入 `grasp.allowed_classes`，然后重新启动：

```bash
ros2 launch rebotarm_vision vision.launch.py
```

## 17. 手眼外参记忆与自动 TF 发布（2026-05-23）

Gemini2 安装在夹爪附近时，属于 eye-in-hand。相机和夹爪之间的固定关系存储在：

```text
~/robotarm_ros2/src/rebotarm_vision/config/handeye.yaml
```

当前文件内容已经从旧版 `softare/rebot_grasp/config/calibration/orbbec_gemini2/hand_eye.npz` 迁移。旧版结果是 `T_cam2gripper`，当前 ROS TF 需要 `end_link -> camera_depth_frame`，因此这里保存的是旧矩阵的逆：

```yaml
handeye:
  parent_frame: end_link
  child_frame: camera_depth_frame
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

含义：

- `parent_frame`：机械臂末端坐标系，当前使用 `end_link`。
- `child_frame`：Gemini2 深度相机坐标系，当前使用 `camera_depth_frame`。
- `translation/rotation`：手眼标定得到的 `end_link -> camera_depth_frame` 外参。

`vision.launch.py` 已经会在启动时自动读取 `handeye.yaml`，并发布静态 TF：

```text
end_link -> camera_depth_frame
```

所以正常使用时只需要启动：

```bash
ros2 launch rebotarm_vision vision.launch.py
```

验证 TF：

```bash
ros2 run tf2_ros tf2_echo end_link camera_depth_frame
```

本次实测输出：

```text
Translation: [0.006, 0.035, 0.056]
Rotation: [0.567, -0.566, 0.435, 0.411]
```

注意：这份结果来自旧版 TSAI eye-in-hand 标定，样本数为 32。因为当前确认旧版 TCP 与 ROS `end_link` 等价、旧版相机坐标与当前 `camera_depth_frame` 一致，所以可以直接迁移。只要相机固定不动，重启电脑或 ROS 都不需要重新标定。

## 18. 直接运行 base_link 稳定性观察工具（2026-05-23）

当前已经把稳定性观察脚本做成 ROS2 命令，不需要再手动粘贴 Python。

先启动机器人 TF 和视觉链路，然后运行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarm_vision rebotarm_grasp_base_watch
```

它会订阅：

```text
/grasp/candidates_3d
```

并把候选点从：

```text
camera_depth_frame
```

转换到：

```text
base_link
```

输出类似：

```text
class=mouse conf=0.43 camera=(+0.985,-0.502,+1.913) base=(+0.310,+0.080,+0.120) spread50=(18.0,22.0,12.0)mm
```

使用方法：

1. 桌面上放一个静止目标，当前请使用能被 YOLO 识别为 `bottle` 的瓶子。
2. 保持目标不动。
3. 让机械臂从几个不同姿态看同一个目标。
4. 看 `base=(...)` 是否基本稳定，以及 `spread50=(...)mm` 是否较小。

判断标准：

- `spread50` 小于 `20-50mm`：旧版手眼标定基本可用。
- `spread50` 大于 `80-100mm`：外参或 frame 对应关系可能有问题。
- 如果一直提示等待 TF，需要先启动能发布 `base_link -> end_link` 的机器人/MoveIt 链路。

可选参数：

```bash
ros2 run rebotarm_vision rebotarm_grasp_base_watch --ros-args \
  -p input_topic:=/grasp/candidates_3d \
  -p target_frame:=base_link \
  -p window_size:=50
```
## 2026-05-23 重力补偿模式调试记录

当前用于手眼标定稳定性验证的推荐流程是：先让实机进入重力补偿模式，人工拖动机械臂到多个姿态，再观察同一个静止 `bottle` 目标转换到 `base_link` 后的位置是否基本稳定。

本次修复了启动重力补偿时报错：

```text
'RobotArm' object has no attribute 'fresh'
```

原因是当前底层 `reBotArm_control_py` 版本没有 `RobotArm.fresh()` 方法。`rebotarmcontroller` 已改为兼容式刷新反馈：优先调用 `fresh()`，没有该方法时改用 `_request_and_poll()`，最后退回 `get_positions(request=True)`。

Ubuntu 端更新代码后需要重新编译并重启 launch：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rebotarmcontroller
source install/setup.bash
```

如果旧的 `interactive_system.launch.py` 还在运行，先在对应终端按 `Ctrl+C` 停掉，再重新启动：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=true \
  use_local_rviz:=false
```

另开一个 Ubuntu 终端启动重力补偿：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger "{}"
```

停止重力补偿：

```bash
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger "{}"
```

进入重力补偿后，机械臂应当可以被手动拖动，但不会完全“掉电自由落体”。如果明显下坠、抖动很大或某个关节发热异常，应立即停止：

```bash
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger "{}"
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

## 2026-05-24 手眼稳定性判断方法

重力补偿拖动测试时，不要看机械臂正在移动过程中的 `spread50` 峰值。移动时最近 50 帧窗口会混入不同时间的相机、深度和 TF 数据，`spread50` 短时间变大是正常现象。

正确判断方法：

```bash
ros2 run rebotarm_vision rebotarm_grasp_base_watch
```

拖动机械臂到一个姿态后停住 2-3 秒，只看停稳后的输出。例如：

```text
base_link=(+0.228,-0.241,+0.251) spread50=(0.9,1.1,0.0)mm
```

当前可接受标准：

- 停稳后 `spread50` 回到几毫米以内，说明单姿态下视觉和 TF 稳定。
- 同一个静止瓶子在 3-5 个不同机械臂姿态下，稳定后的 `base_link` 坐标差别约 `5-15mm` 以内，可以先进入抓取联调。
- 如果停稳后 `spread50` 仍然几十毫米以上，或者不同姿态的 `base_link` 坐标漂移超过 `30mm`，再回头检查手眼外参、TF 方向、深度对齐和时间同步。

## 2026-05-24 execute_preview 返回 accepted 但电机不动

如果调用：

```bash
ros2 service call /rebotarm/interactive_control/execute_preview std_srvs/srv/Trigger "{}"
```

返回：

```text
success=True, message='execution accepted'
```

但电机没有移动，优先检查交互执行模式。当前 `interactive_control.yaml` 默认是：

```yaml
mode: simulation
```

在 `simulation` 模式下，`execute_preview` 只表示“预览结果被接受”，不会向 `/rebotarm/follow_joint_trajectory` 发送真实轨迹，因此电机不会动。

实机执行前需要切换到 `real` 模式：

```bash
ros2 service call /rebotarm/interactive_control/set_mode rebotarm_msgs/srv/SetMode "{mode: real}"
```

再执行：

```bash
ros2 service call /rebotarm/interactive_control/execute_preview std_srvs/srv/Trigger "{}"
```

如果切到 `real` 后仍然不动，再检查：

```bash
ros2 action list -t | grep follow_joint_trajectory
ros2 topic echo /rebotarm/interactive_control/status --once
ros2 topic hz /rebotarm/joint_states
```

## 2026-05-24 正式抓取路线说明

当前正式抓取路线与旧版 `softare/rebot_grasp` 保持一致：不要把 `/grasp/candidates_3d` 中的物体中心点直接当作夹爪目标点。

`/grasp/candidates_3d` 当前只能说明：

```text
目标大致在 base_link 下哪里
```

它还不能直接说明：

```text
夹爪 TCP 应该到哪里
夹爪应该从哪个方向接近
夹爪开合轴应该怎么对准物体
```

正式抓取应使用旧源码思路：

```text
物体检测 + 深度
-> 估计抓取姿态
-> 生成 grasp pose 和 pregrasp pose
-> 控制 grasp_tcp，而不是直接控制 end_link
-> open -> pregrasp -> grasp -> close -> lift/ready
```

因此当前手动发送 `x,y,z+8cm` 只用于验证 MoveIt 可达性，不作为正式抓取命令。

## 2026-05-24 ROS2 抓取规划节点第一版

当前已新增 ROS2 抓取规划节点：

```text
rebotarm_grasp_plan_node
```

它订阅：

```text
/grasp/candidates_3d
```

发布：

```text
/grasp/plan
/grasp/pregrasp_pose
/grasp/grasp_pose
```

其中 `/grasp/plan` 类型为：

```text
rebotarm_msgs/msg/GraspPlan
```

第一版功能：

- 从 `/grasp/candidates_3d` 选择最佳有效候选。
- 通过 TF 转到 `base_link`。
- 按旧源码的思路生成 `grasp` 和 `pregrasp` 两个位姿。
- `pregrasp` 沿 TCP X 轴反方向退 `0.08m`。
- 支持配置 `end_link -> grasp_tcp` 偏移，后续测出真实夹爪 TCP 后填入。

当前第一版仍是保守版本：

- 已实现 `pregrasp/grasp` 双位姿。
- 已实现 TCP 偏移补偿接口。
- 还没有完整迁移旧版 OBB/mask 夹爪方向估计。
- 当前默认采用 top-down 接近方向：

```yaml
grasp_plan.tcp_approach_axis: [0.0, 0.0, -1.0]
grasp_plan.tcp_open_axis: [0.0, 1.0, 0.0]
```

启动视觉链路时会自动启动抓取规划节点：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_vision vision.launch.py
```

查看抓取规划输出：

```bash
ros2 topic echo /grasp/plan --once
ros2 topic echo /grasp/pregrasp_pose --once
ros2 topic echo /grasp/grasp_pose --once
```

如果要先用当前交互执行链路测试预抓取点，可以把 `/grasp/pregrasp_pose` 中的 pose 复制到：

```bash
ros2 topic pub --once /rebotarm/interactive_control/pose_target geometry_msgs/msg/Pose \
"{position: {x: 0.224, y: -0.241, z: 0.326}, orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}"
```

实际数值以 `/grasp/pregrasp_pose` 输出为准。执行前仍然需要：

```bash
ros2 service call /rebotarm/interactive_control/set_mode rebotarm_msgs/srv/SetMode "{mode: real}"
```

第一版测试结果：

- `colcon build --symlink-install --packages-select rebotarm_msgs rebotarm_vision` 通过。
- 抓取规划转换相关 pytest 通过。
- 无硬件 ROS 冒烟中，人工 bottle 候选可生成：

```text
pregrasp=(+0.224,-0.241,+0.326)
grasp=(+0.224,-0.241,+0.246)
jaw=0.060m
```

## 2026-05-24 一键发送抓取预览点

为避免每次手写 Python 转发 `/grasp/plan`，已新增测试工具：

```text
rebotarm_send_grasp_preview
```

默认行为：

```text
订阅 /grasp/plan
读取 pregrasp_pose
发布到 /rebotarm/interactive_control/pose_target
发布一次后自动退出
```

推荐仿真/预览测试流程：

终端 1 启动视觉和抓取规划：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_vision vision.launch.py
```

终端 2 启动 MoveIt 预览，先关闭硬件：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=false \
  use_local_rviz:=false
```

终端 3 一键发送 `pregrasp_pose` 到 MoveIt 预览：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rebotarm_vision rebotarm_send_grasp_preview
```

终端 4 查看预览结果：

```bash
ros2 topic echo /rebotarm/interactive_control/preview --once
```

如果要测试真正的 `grasp_pose`，可以显式切换参数，但当前不建议直接实机执行：

```bash
ros2 run rebotarm_vision rebotarm_send_grasp_preview --ros-args -p pose_mode:=grasp
```

当前验证结果：

- `colcon build --symlink-install --packages-select rebotarm_vision` 通过。
- `python3 -m pytest tests/test_grasp_preview_sender.py tests/test_grasp_plan_converter.py -q` 通过，共 7 个测试。
- `ros2 pkg executables rebotarm_vision` 可找到 `rebotarm_send_grasp_preview`。

## 2026-05-24 MoveIt 预览仿真 joint_states 修复

问题现象：

```text
ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=false \
  use_local_rviz:=false
```

启动约 10 秒后 `move_group` 报错：

```text
Didn't receive robot state (joint angles)
Unable to configure planning scene monitor
```

原因：`use_hardware:=false` 时没有真实 `reBotArmController` 发布 `/rebotarm/joint_states`，但 MoveIt 被配置为监听 `/rebotarm/joint_states`。之前仿真 `joint_state_publisher` 又只在 `use_moveit_preview:=false` 时启动，导致 MoveIt 预览模式下没有关节状态。

已修复：

- `interactive_system.launch.py` 中的 `joint_state_publisher` 改为只要 `use_hardware:=false` 就启动。
- 它发布的话题 remap 到 `/rebotarm/joint_states`，供 MoveIt 和 robot_state_publisher 使用。

现在可以重新执行仿真预览启动：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=false \
  use_local_rviz:=false
```

正常标志：

```text
interactive_joint_state_publisher: Got description, configuring robot
You can start planning now!
```

验证结果：

- `colcon build --symlink-install --packages-select rebotarm_bringup` 通过。
- `/rebotarm/joint_states` 能收到仿真关节状态。
- 短时启动验证中 `move_group` 不再因缺少 joint state 崩溃。

## 2026-05-24 抓取规划默认姿态改为保守可达姿态

实测发现：

```text
position=(0.226,-0.237,0.329), orientation=(0,0,0,1)
```

MoveIt IK 可以预览成功；但第一版 top-down 姿态：

```text
orientation=(0,0.7071,0,0.7071)
```

会返回：

```text
moveit preview failed: error_code=-31
```

因此当前抓取规划节点已调整为：

- `pregrasp` 位置仍保持抓取点上方 `0.08m`。
- 预抓取退避方向由 `grasp_plan.pregrasp_offset_axis` 单独控制。
- 默认末端姿态改为 MoveIt 已验证可达的保守姿态：

```text
orientation=(0,0,0,1)
```

当前配置：

```yaml
grasp_plan.pregrasp_offset_axis: [0.0, 0.0, -1.0]
grasp_plan.tcp_approach_axis: [1.0, 0.0, 0.0]
grasp_plan.tcp_open_axis: [0.0, 1.0, 0.0]
```

直接校验结果：

```text
pregrasp 0.226 -0.238 0.329
quat 0.0 0.0 0.0 1.0
```

后续如果要恢复旧源码的 top-down / OBB 姿态，需要先完成 `grasp_tcp` 偏移和可达姿态标定。
