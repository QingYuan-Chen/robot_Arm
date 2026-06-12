#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${repo_root}/.venv-vision"
python_bin="${venv_dir}/bin/python"

python3 -m venv --system-site-packages "${venv_dir}"
"${python_bin}" -m pip install --upgrade "pip<27" "setuptools<80" wheel
"${python_bin}" -m pip install \
  torch torchvision \
  --index-url https://download.pytorch.org/whl/cu128
"${python_bin}" -m pip install -r "${repo_root}/requirements-vision.txt"

set +u
source /opt/ros/jazzy/setup.bash
set -u
"${python_bin}" -m colcon \
  --log-base "${repo_root}/log" \
  build \
  --symlink-install \
  --base-paths "${repo_root}/src" \
  --build-base "${repo_root}/build" \
  --install-base "${repo_root}/install" \
  --packages-up-to rebotarm_vision

"${python_bin}" - <<'PY'
import cv2
import numpy
import pyorbbecsdk
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO

print(f"NumPy: {numpy.__version__}")
print(f"OpenCV: {cv2.__version__}")
print(f"PyTorch: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA device: {torch.cuda.get_device_name(0)}")
print("ROS cv_bridge, Orbbec SDK and Ultralytics imports: OK")
PY

echo
echo "Environment ready."
echo "Install the Orbbec udev rule once after connecting the camera:"
echo "  ${repo_root}/tools/install_orbbec_udev_rules.sh"
echo "Start Ubuntu-native vision with:"
echo "  ${repo_root}/tools/run_ubuntu_vision.sh"
