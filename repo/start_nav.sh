#!/usr/bin/env bash
set -eo pipefail

PCD_FILE="${1:-/home/und/桌面/CorrectedFrontendMap_cut.pcd}"

cd "$(dirname "$0")"

# 清理旧进程
pkill -f "pcd_to_octomap" 2>/dev/null || true
pkill -f "jie_path" 2>/dev/null || true
pkill -f "octomap_to_occupied" 2>/dev/null || true
pkill -f "map_package" 2>/dev/null || true
pkill -f "pcd_map_import" 2>/dev/null || true
pkill -f "map_viewer" 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动导航 launch
ros2 launch jie_octomap import_pcd_map.launch.py &
LAUNCH_PID=$!
sleep 4

# 加载 PCD 地图
ros2 topic pub /pcd_file_cmd std_msgs/msg/String "data: '${PCD_FILE}'" -1
sleep 6

# 启动能显示路径的地图查看器
ros2 run jie_octomap map_viewer_gui &
VIEWER_PID=$!
sleep 2

echo ""
echo "导航系统已启动，PCD 地图加载完成。"
echo ""
echo "有两个窗口："
echo "  pcd_map_import_gui - 只显示 OctoMap 体素（无视这个窗口）"
echo "  map_viewer_gui     - 在这个窗口操作："
echo "    1. 点击 '起始点' → 在地图上选起点"
echo "    2. 点击 '目标点' → 在地图上选终点"
echo "    3. 路径（青色线条）自动显示"
echo ""

wait $LAUNCH_PID
