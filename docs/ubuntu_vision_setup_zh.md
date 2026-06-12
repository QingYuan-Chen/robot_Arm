# Ubuntu 原生 Gemini2 与 YOLO 环境

本机方案不再依赖 Windows HTTP 服务，数据链路为：

```text
Gemini2 USB -> pyorbbecsdk -> rebotarm_vision_node
             -> Ultralytics YOLO/CUDA -> ROS 2 topics
```

原有 `camera.yaml` 和 Windows 服务仍保留为网络备用链路。Ubuntu 原生链路使用
`camera_ubuntu.yaml` 和 `vision_ubuntu.launch.py`。

## 1. 安装

在仓库根目录执行：

```bash
./tools/setup_ubuntu_vision.sh
```

脚本会创建 `.venv-vision`，安装 CUDA 12.8 版 PyTorch、Ultralytics 和
Orbbec Python SDK，并使用该 Python 重新构建 `rebotarm_vision`。

这里固定 `numpy==1.26.4` 和 `pyorbbecsdk2==2.0.18`。不要直接升级到
`pyorbbecsdk2==2.1.1`，它在 Python 3.12 上要求 NumPy 2，与 ROS 2 Jazzy
系统包中的 `cv_bridge` ABI 不兼容。

## 2. 相机权限

接入 Gemini2 后执行一次：

```bash
./tools/install_orbbec_udev_rules.sh
```

该步骤需要输入 sudo 密码。完成后拔插相机，并检查：

```bash
lsusb | grep -i -E "orbbec|2bc5"
```

## 3. 启动

```bash
./tools/run_ubuntu_vision.sh
```

默认使用：

- Gemini2 彩色流 `1280x720 @ 30 FPS`
- Gemini2 深度流 `1280x720 @ 30 FPS`
- 硬件深度对齐
- `yolo11n-seg.pt`
- NVIDIA GPU `device=0`
- 检测发布 `/grasp/detections`
- 标注图发布 `/camera/color/annotated`

临时改用 CPU：

```bash
./tools/run_ubuntu_vision.sh yolo_device:=cpu
```

## 4. 验证

```bash
.venv-vision/bin/python -c \
  "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /grasp/detections --once
```

如果启动时报找不到相机，优先检查 USB 枚举和 udev 规则。如果 CUDA 不可用，
检查 `nvidia-smi`，不要先安装系统级 CUDA Toolkit；PyTorch wheel 已包含运行时。
