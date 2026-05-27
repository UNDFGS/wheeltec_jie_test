#!/usr/bin/env bash
# ============================================================
# 一键启动 Ackermann 轮式机器人 OctoMap 导航仿真
# ============================================================
set -eo pipefail

PCD_FILE="${1:-/home/und/桌面/merged_all_clean.pcd}"
WS_DIR="${HOME}/integrated_ws"

# 清理旧进程
kill $(ps aux | grep -E "pcd_to_octo|jie_path|octo_to|map_package|map_viewer|d1_control|kinematic_sim|static_transform|sim_nav" | grep -v grep | awk '{print $2}') 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

# 启动仿真 launch
ros2 launch octo_planner sim_nav.launch.py pcd_map_path:="${PCD_FILE}" &
LAUNCH_PID=$!
sleep 4

# 加载 PCD 地图
ros2 topic pub /pcd_file_cmd std_msgs/msg/String "data: '${PCD_FILE}'" -1
sleep 1

echo ""
echo "=============================================="
echo "  Ackermann 轮式机器人导航仿真已启动"
echo "  PCD: ${PCD_FILE}"
echo ""
echo "  TF 链: map → odom → base_link (Ackermann)"
echo ""
echo "  操作步骤:"
echo "    1. 在 map_viewer_gui 点击 '起始点'/'目标点' 设定路线"
echo "    2. 或在 RViz 中点击 '2D Goal Pose' 设定目标"
echo "    3. 路径规划完成后，点击 '开始导航' 或命令行:"
echo "       ros2 topic pub /start_navigation std_msgs/msg/Bool 'data: true' -1"
echo "    4. 机器人会在 RViz 中沿着路径移动"
echo "=============================================="
echo ""

wait $LAUNCH_PID
