# Ubuntu 原生 Gemini2 与 YOLO 环境

> 当前状态：本文描述的独立 Ubuntu 视觉入口已经存在；真实 Gemini 2 RGB-D 稳定性、SDK 内参、实际 stream profile、深度尺度和 RGB-D 对齐仍待 P2 硬件验收。完整 `visual_grasp_system.launch.py` 尚未完成 `vision_profile:=ubuntu_native` 集成。

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
- `YOLO26m-seg FP16 TensorRT engine`（源文件：`tools/yolo26m-seg-fp16-b1-640-linux.engine`）
- engine SHA-256：`9fe8e3b024a1804d890e4a760f9326a2475f370857f2249083f4c19fce920f48`
- NVIDIA GPU `device=0`
- 检测发布 `/grasp/detections`
- 标注图发布 `/camera/color/annotated`

TensorRT engine 只用于当前 Linux/CUDA/TensorRT 运行环境，不能切到 CPU。临时改用
CPU 时需要同时回退到随包保留的 `.pt` 模型：

```bash
./tools/run_ubuntu_vision.sh \
  yolo_model_path:="$PWD/install/rebotarm_vision/share/rebotarm_vision/models/yolo26s-seg.pt" \
  yolo_device:=cpu
```

## 4. 验证

```bash
.venv-vision/bin/python -c \
  "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /grasp/detections --once
```

当前验证还必须补充：

```bash
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
```

验收时记录而不是估算：

- 设备名称、序列号、固件版本和 USB 类型；
- 实际 RGB/depth profile 与格式；
- SDK intrinsics、distortion、extrinsics 和 depth scale；
- 深度中心点及多个已知距离样本的误差；
- 硬件对齐后 RGB 与 depth 是否对应同一像素坐标系。

在驱动尚未提供 SDK 标定信息前，`CameraInfo` 出现并不等于标定正确。原始深度也不能默认视为毫米，必须先确认设备 `depth_scale`。

如果启动时报找不到相机，优先检查 USB 枚举和 udev 规则。如果 CUDA 不可用，
检查 `nvidia-smi`，不要先安装系统级 CUDA Toolkit；PyTorch wheel 已包含运行时。

该入口只覆盖 Gemini 2 + YOLO，不代表 Ubuntu 本地 GraspNet、hand-eye 标定或完整抓取执行已经完成。

## 5. 查看 Gemini 2 点云

首次使用安装 ROS 2 深度图转点云组件：

```bash
sudo apt install ros-jazzy-depth-image-proc
```

保持 `./tools/run_ubuntu_vision.sh` 运行，在第二个终端启动转换节点：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/project/rebot_Arm/install/setup.bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/depth/image_raw \
  -r camera_info:=/camera/depth/camera_info \
  -r points:=/camera/depth/points
```

第三个终端运行 `rviz2`，将 `Fixed Frame` 设为 `camera_depth_frame`，添加
`PointCloud2` display 并选择 `/camera/depth/points`。视觉节点让 Image 与对应的
CameraInfo 使用相同 QoS，避免 `depth_image_proc` 因 reliability 不兼容而收不到
同步内参。该路径只读取相机，不启动机械臂控制器。

若要在同一个 Open3D 窗口查看 GraspNet 在采样/下采样前内部重建的完整 `XYZ+RGB`
场景点云和原始抓取位姿，先保持整合后的视觉抓取launch（其中包含
`rebotarm_graspnet_baseline_node`）运行，再执行：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/tools:$PWD/src/rebotarm_vision${PYTHONPATH:+:$PYTHONPATH}" \
  ./.venv-graspnet/bin/python tools/view_graspnet_scene_cloud.py
```

该查看器不执行 `sample_cloud()`，并把 `build_scene_cloud()` 产生的 points/colors
直接交给复用的 Open3D renderer，因此背景显示的是
`0.05-1.5 m` 采样前完整场景，而不是送入网络的 20,000 点确定性采样子集；抓取框来自
`/grasp/graspnet_candidates`，保持候选原始相机坐标位姿并按消息顺序显示前 10 个。
可用 `--top-n N` 调整抓取框数量，或用 `--max-points N` 限制仅用于显示的点数；
默认 `--max-points 0` 保留完整点云。此工具只做可视化，不启动 MoveIt、控制器或
机械臂执行。

夹爪优先使用可用的 `graspnetAPI` 官方几何；当前隔离环境缺少其非推理可视化重依赖
时，会自动使用 Open3D 原生实体平行夹爪（掌部 + 两根手指），而不是退化成难以看清
的细线框。启动日志会明确显示当前模式，例如：

```text
gripper_renderer=native_open3d_mesh graspnet_api_unavailable=ModuleNotFoundError: No module named 'trimesh'
```

这不影响 GraspNet 推理结果；原生实体的 position、rotation、jaw width、height 和
score 仍来自 `/grasp/graspnet_candidates`，消息未携带的 finger depth 继续使用
`0.04 m` 可视化缺省值。若运行中 renderer 从官方 geometry 降级到原生 mesh 或
最后一级 wireframe，日志会立即输出新的 mode 和失败原因，不会继续显示过期状态。

## 6. P4 Ubuntu 本地 GraspNet 环境

P4 使用与 `.venv-vision` 隔离的 `.venv-graspnet`，仅安装经过固定的
PyTorch/CUDA 和 GraspNet 推理依赖。安装和健康检查均为本地操作，不会连接机械臂、
打开串口、enable 电机或发送轨迹：

```bash
./tools/setup_ubuntu_graspnet.sh
PYTHONPATH= ./.venv-graspnet/bin/python tools/check_ubuntu_graspnet_env.py
PYTHONPATH= ./.venv-graspnet/bin/python -m pip check
```

当前仓库没有随附 GraspNet 模型代码或 checkpoint。安装脚本不会自动下载第三方
模型；只有在确认来源、版本和 license 后，才设置：

```bash
export GRASPNET_MODEL_ROOT=/path/to/reviewed/graspnet
export GRASPNET_CHECKPOINT_PATH=/path/to/reviewed/checkpoint.pth
```

Ubuntu生产路径不再单独启动localhost HTTP service。相机/YOLO topics 已启动后，可在
ROS 2环境中单独验证整合后的in-process node：

```bash
source install/setup.bash
ros2 run rebotarm_vision rebotarm_graspnet_baseline_node --ros-args \
  --params-file install/rebotarm_vision/share/rebotarm_vision/config/graspnet_ubuntu.yaml
```

该node直接将时间匹配的aligned RGB、米制depth、CameraInfo intrinsics、YOLO mask/bbox、
timestamp和`camera_depth_frame`交给同进程GraspNet runner，再把结果发布到
`/grasp/graspnet_candidates`。它保留完整场景collision cloud、目标mask/depth分离、
确定性20,000点采样、projection filter与jaw-width filter。RGB/depth skew超限、模型未配置
或推理异常时只发布空候选，不复用旧结果。

`tools/run_ubuntu_graspnet_service.sh`和`tools/ubuntu_graspnet_service.py`仅保留为历史回退/
contract测试工具，不在Ubuntu生产launch中启动；Windows/network candidates兼容模式不受影响。

响应原样保留 `timestamp_ns` 与 `frame_id`，并包含 `backend_configured`、`stale`
和 `candidates`。输入单位、图像尺寸、bbox、intrinsics 或 header 不合法时返回
HTTP 400；backend 未配置时返回 HTTP 503。

因此环境和 service liveness 通过不等于真实 GraspNet 推理已通过；真实候选输出仍需
在模型资产完成 provenance / license 审核后单独验收。
