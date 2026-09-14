# Ubuntu 视觉只读测试手册

本手册只启动 Gemini 2、YOLO、ROS RGB-D/CameraInfo/detection 和本地 GraspNet
候选发布，不启动机械臂控制器，不调用 enable，不发送轨迹。

## 1. 先准备环境

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

./tools/setup_ubuntu_vision.sh
./tools/setup_ubuntu_graspnet.sh
```

安装脚本输出 `CUDA available: True`、显卡名称、`GraspNet environment imports: OK`
和 `No broken requirements found.` 即表示 Python 环境检查通过。它不会下载
GraspNet 模型；必须先准备经过确认的模型资产：

```bash
export GRASPNET_MODEL_ROOT=/absolute/path/to/graspnet-model-root
export GRASPNET_CHECKPOINT_PATH=/absolute/path/to/checkpoint-rs.tar
```

## 2. 启动视觉只读链路

模型路径和推理设备使用仓库默认值，不需要再输入参数：

```bash
./tools/run_ubuntu_vision.sh
```

默认值为：

- YOLO：安装包内的 `yolo26s-seg.pt`
- YOLO 设备：`0`，即 CUDA 第一张 GPU
- YOLO 抓取白名单：`bottle`、`cup`；其他类别不会发布到 `/grasp/detections`
- GraspNet 设备：`cuda:0`
- RGB-D：`/camera/color/image_raw`、`/camera/depth/image_raw`
- YOLO 检测：`/grasp/detections`
- GraspNet 候选：`/grasp/graspnet_candidates`

如果安装包内没有模型，启动会明确报模型路径错误；不要通过静默切换到
未知模型继续验收。更换模型或机器时，使用完整 launch 参数覆盖即可，但不属于
日常测试命令。

## 3. 查看 YOLO 分割画面

另开终端：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rqt_image_view rqt_image_view /camera/color/annotated
```

同时检查话题是否持续发布：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/color/annotated
ros2 topic echo /grasp/detections --once
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
```

正常现象是 RGB、depth、annotated 频率持续更新，分割图上出现检测框/掩码，
检测消息的时间戳和 frame_id 有效。此步骤不代表 GraspNet 已成功。

## 4. 查看 GraspNet 候选和夹爪位姿

检查候选消息：

```bash
ros2 topic echo /grasp/graspnet_candidates --once
ros2 topic hz /grasp/graspnet_candidates
```

夹爪位姿优先使用 RViz Marker 显示。启动只读预览：

```bash
ros2 launch rebotarm_bringup visual_grasp_perception_preview.launch.py
```

在 RViz 中确认：

- Fixed Frame 使用 `camera_depth_frame` 或当前消息 frame；
- 添加 `MarkerArray`，话题选择视觉夹爪 Marker 话题；
- 能看到候选物体、pregrasp、grasp、TCP、接近箭头和夹爪开口方向。

这个预览不会启动真实机械臂，也不会执行候选。RViz Marker 比直接在 GraspNet
推理节点内启动 Open3D 窗口更适合 ROS 测试，因为它保持 ROS 时间戳、TF 和候选
消息一致，也不会让 GUI 阻塞推理线程。

当前版本没有把 Open3D GUI 作为默认 ROS 测试窗口。Open3D 适合离线保存一帧
点云后检查几何，但不适合直接放进实时 GraspNet 回调：GUI 事件循环可能阻塞
推理，且点云与候选位姿容易出现时间戳不一致。因此实时验收使用 RViz：它能同时
显示点云、TF、候选物体、pregrasp、grasp、TCP 和夹爪开口方向；需要离线点云
质量分析时再单独使用 Open3D 查看保存的数据。

## 5. 验收顺序

```text
相机 RGB/depth 持续发布
-> CameraInfo 有效
-> YOLO annotated 图像正常
-> Detection2DArray 有检测
-> GraspNet backend_available=True
-> graspnet_candidates 有非空候选
-> RViz 中夹爪位姿和 TF 方向合理
```

如果 YOLO 图像正常但候选为空，优先检查：

```bash
echo "$GRASPNET_MODEL_ROOT"
echo "$GRASPNET_CHECKPOINT_PATH"
ros2 topic echo /grasp/detections --once
```

候选为空时仍属于 fail-closed，不得因此直接开放真机执行。

## 6. 停止

使用 `Ctrl-C` 停止视觉终端和 RViz。整个手册流程不需要连接机械臂，
也不需要调用 `/rebotarm/enable`。
