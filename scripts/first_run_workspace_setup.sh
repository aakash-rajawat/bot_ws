#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

cd "${WS_ROOT}"

ROSDEP_SOURCE_LIST="/etc/ros/rosdep/sources.list.d/00-bot-ws.list"
ROSDEP_SOURCE_LINE="yaml file://${WS_ROOT}/rosdep/base.yaml"

sudo mkdir -p "$(dirname "${ROSDEP_SOURCE_LIST}")"
echo "${ROSDEP_SOURCE_LINE}" | sudo tee "${ROSDEP_SOURCE_LIST}" >/dev/null

rosdep update --rosdistro "${ROS_DISTRO}" || true

bash "${WS_ROOT}/scripts/fetch_third_party.sh"
bash "${WS_ROOT}/scripts/bootstrap_workspace_venvs.sh" "${WS_ROOT}"
