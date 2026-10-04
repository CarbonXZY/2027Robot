#!/usr/bin/env bash
#
# 一键启动整车控制：源工作空间 → ros2 launch
#
# 用法:
#   ./launch.sh
#
set -e

# 工作空间根目录 = 本脚本所在目录（从任意路径调用都成立）
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# 1. 源 ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "[launch.sh] 未找到 /opt/ros/humble/setup.bash" >&2
    exit 1
fi

# 2. 源工作空间
# shellcheck disable=SC1091
source install/setup.bash

# 3. 启动
echo "[launch.sh] ros2 launch robot robot.launch"
ros2 launch robot robot.launch
