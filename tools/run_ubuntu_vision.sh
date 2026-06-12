#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${repo_root}/.venv-vision"

if [[ ! -x "${venv_dir}/bin/python" ]]; then
  echo "Vision environment not found. Run tools/setup_ubuntu_vision.sh first." >&2
  exit 1
fi
if [[ ! -f "${repo_root}/install/setup.bash" ]]; then
  echo "ROS workspace is not built. Run tools/setup_ubuntu_vision.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${venv_dir}/bin/activate"
source "${repo_root}/install/setup.bash"
set -u

exec ros2 launch rebotarm_vision vision_ubuntu.launch.py "$@"
