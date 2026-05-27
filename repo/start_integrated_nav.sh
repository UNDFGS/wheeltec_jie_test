#!/usr/bin/env bash
# ============================================================
# 一键启动集成导航系统
# 定位: FAST-LIO + open3d_loc (ICP 重定位)
# 导航: OctoMap + A* 规划 + d1_controller 路径跟踪
#
# 当前模式: 测试模式（无 LiDAR 硬件时用静态 TF）
# 有 LiDAR 后: 先编译 FAST_LIO_LOCALIZATION_HUMANOID，再去掉 launch_localization:=false
# ============================================================
set -eo pipefail

PCD_FILE="${1:-/home/und/桌面/CorrectedFrontendMap_cut.pcd}"
WS_DIR="${HOME}/integrated_ws"

# 清理旧进程
pkill -f "pcd_to_octomap" 2>/dev/null || true
pkill -f "jie_path" 2>/dev/null || true
pkill -f "octomap_to_occupied" 2>/dev/null || true
pkill -f "map_package" 2>/dev/null || true
pkill -f "map_viewer" 2>/dev/null || true
pkill -f "d1_controller" 2>/dev/null || true
pkill -f "global_localization" 2>/dev/null || true
pkill -f "static_transform" 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

# 启动集成导航
# launch_localization:=false  表示用静态 TF 代替真实定位（测试用）
# launch_rviz:=false          不启动 RViz（可改为 true）
ros2 launch octo_planner integrated_nav.launch.py \
    launch_localization:=false \
    launch_rviz:=false &
LAUNCH_PID=$!
sleep 5

# 加载 PCD 地图
ros2 topic pub /pcd_file_cmd std_msgs/msg/String "data: '${PCD_FILE}'" -1
sleep 6

echo ""
echo "=========================================="
echo "  集成导航系统已启动"
echo "  地图: ${PCD_FILE}"
echo ""
echo "  TF 链: map → odom → base_link"
echo "  定位: 静态 TF（测试模式）"
echo ""
echo "  在 map_viewer_gui 窗口中操作:"
echo "    1. 点击 '起始点' → 在地图上选起点"
echo "    2. 点击 '目标点' → 在地图上选终点"
echo "    3. 路径（青色线条）自动显示"
echo ""
echo "  升级到真实定位（需 LiDAR 硬件）:"
echo "    1. 安装 livox-SDK2 + livox_ros_driver2"
echo "    2. cd ~/integrated_ws && colcon build"
echo "    3. 去掉 launch_localization:=false"
echo "=========================================="
echo ""

wait $LAUNCH_PID
