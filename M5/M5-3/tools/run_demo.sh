#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
unset PYTHONHOME PYTHONPATH
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${M53_DEMO_DOMAIN:-174}"
if [[ "$ROS_DOMAIN_ID" != 173 && "$ROS_DOMAIN_ID" != 174 ]]; then
  echo "模拟器仅允许隔离 DDS 域 173 或 174" >&2
  exit 2
fi
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$PWD/work/demo_ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec .venv/bin/python -B tools/demo_terminal.py "$@"
