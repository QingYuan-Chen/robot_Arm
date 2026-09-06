#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REBOTARM_VISION_PYTHON="${REBOTARM_VISION_PYTHON:-${repo_root}/.venv-vision/bin/python}"

if ! command -v "${REBOTARM_VISION_PYTHON}" >/dev/null; then
  echo "Vision environment not found. Run tools/setup_ubuntu_vision.sh first." >&2
  exit 1
fi
if [[ ! -f "${repo_root}/install/setup.bash" ]]; then
  echo "ROS workspace is not built. Follow docs/local_setup_zh.md before starting vision." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${repo_root}/install/setup.bash"
set -u

exec ros2 launch rebotarm_vision vision_ubuntu.launch.py "$@"
