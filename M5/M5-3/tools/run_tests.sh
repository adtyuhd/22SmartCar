#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
unset PYTHONHOME PYTHONPATH
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$PWD/work/ros_logs"
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
mkdir -p work "$ROS_LOG_DIR"
if [[ "${1:-}" == "--ros" ]]; then
  export M53_RUN_ROS_TESTS=1
  shift
else
  unset M53_RUN_ROS_TESTS
fi
exec .venv/bin/python -B -m pytest -q -p no:cacheprovider --basetemp="$PWD/work/pytest" tests "$@"
