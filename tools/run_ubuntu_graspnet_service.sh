#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${repo_root}/.venv-graspnet/bin/python"

if [[ ! -x "${python_bin}" ]]; then
  echo "Missing .venv-graspnet; run ./tools/setup_ubuntu_graspnet.sh first." >&2
  exit 2
fi

export PYTHONPATH="${repo_root}/src/rebotarm_vision:${repo_root}/tools"
exec "${python_bin}" "${repo_root}/tools/ubuntu_graspnet_service.py" "$@"
