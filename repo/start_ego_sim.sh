#!/usr/bin/env bash
# ============================================================
# 模式1: 仿真 + EGO-Planner 两级导航
# 全局 A* on OctoMap + 局部 EGO B样条避障
# ============================================================
set -eo pipefail

PCD_FILE="${1:-/home/und/桌面/merged_all_clean.pcd}"
WS_DIR="${HOME}/integrated_ws"

# 清理
pkill -f "pcd_to_octomap" 2>/dev/null || true
pkill -f "jie_path" 2>/dev/null || true
pkill -f "ackermann" 2>/dev/null || true
pkill -f "kinematic_sim" 2>/dev/null || true
pkill -f "ego_planner" 2>/dev/null || true
pkill -f "ego_bridge" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "robot_state" 2>/dev/null || true
pkill -f "static_trans" 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

ros2 launch octo_planner ego_nav.launch.py \
    launch_localization:=false \
    use_sim:=true \
    pcd_map_path:="${PCD_FILE}" &
LAUNCH_PID=$!
sleep 5

# 加载 PCD
ros2 topic pub /pcd_file_cmd std_msgs/msg/String "data: '${PCD_FILE}'" -1

echo ""
echo "=============================================="
echo "  EGO-Planner 两级导航仿真已启动"
echo "  全局: A* on OctoMap"
echo "  局部: EGO-Planner (B样条优化)"
echo "  控制: AckermannTracker (Stanley)"
echo ""
echo "  RViz 操作:"
echo "    2D Pose Estimate → 设定机器人位姿+起点"
echo "    2D Goal Pose     → 设定导航目标"
echo "=============================================="

wait $LAUNCH_PID
