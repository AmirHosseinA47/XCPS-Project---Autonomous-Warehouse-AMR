#!/usr/bin/env bash
set -euo pipefail

WS="${HOME}/warehouse_ws"

if [[ ! -d "${WS}/src" ]]; then
  echo "ERROR: ${WS}/src does not exist."
  exit 1
fi

echo "[1/5] Removing Python cache files..."
find "${WS}/src" -type d -name "__pycache__" -prune -exec rm -rf {} +
find "${WS}/src" -type f -name "*.pyc" -delete

echo "[2/5] Removing old colcon output..."
rm -rf "${WS}/build" "${WS}/install" "${WS}/log"

echo "[3/5] Ensuring runtime log directory exists..."
mkdir -p "${WS}/mission_logs"

echo "[4/5] Rebuilding workspace..."
cd "${WS}"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo "[5/5] Done."
echo
echo "Run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/warehouse_ws/install/setup.bash"
echo "  ros2 launch warehouse_robot_description warehouse_system.launch.py"
