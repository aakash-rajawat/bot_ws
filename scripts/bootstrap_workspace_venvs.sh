#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${1:-/workspaces/bot_ws}"
VENV_DIR="${WS_ROOT}/xfeat_lightglue"
REQUIREMENTS_FILE="${WS_ROOT}/requirements_xfeat_lightglue.in"

for weight in   "${WS_ROOT}/third_party/accelerated_features/weights/xfeat.pt"   "${WS_ROOT}/third_party/accelerated_features/weights/xfeat-lighterglue.pt"
do
  if [[ ! -f "${weight}" ]]; then
    echo "Missing required model weight: ${weight}" >&2
    exit 1
  fi
done

rm -rf "${VENV_DIR}"
env -u PYTHONPATH python3 -m venv "${VENV_DIR}"
env -u PYTHONPATH "${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
env -u PYTHONPATH "${VENV_DIR}/bin/python" -m pip install -r "${REQUIREMENTS_FILE}"
