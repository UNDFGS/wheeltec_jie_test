#!/usr/bin/env bash
# ============================================================
# R550 Plus 实机部署 (FAST-LIO + LiDAR + EGO-Planner)
#
# 需先配置:
#   1. wheeltec_param.yaml: car_mode=senior_akm
#   2. LiDAR 串口: /dev/wheeltec_lidar
#   3. 底盘串口: /dev/wheeltec_controller
#   4. PCD 地图路径
# ============================================================
set -eo pipefail

PCD_FILE="${1:-/home/und/桌面/merged_all_clean.pcd}"
WS_DIR="${HOME}/integrated_ws"
WHEELTEC_WS="${HOME}/wheeltec"
FASTLIO_WS="${HOME}/fastlio2"
LIDAR_TOPIC="/point_cloud_raw"
CAR_MODE="senior_akm"   # R550 Plus = senior_akm 或 autoware_akm_ultra_single_antenna

# ---- 清理 ----
pkill -f "pcd_to_octomap" 2>/dev/null || true
pkill -f "jie_path" 2>/dev/null || true
pkill -f "ackermann" 2>/dev/null || true
pkill -f "ego_planner" 2>/dev/null || true
pkill -f "ego_bridge" 2>/dev/null || true
pkill -f "fast_lio" 2>/dev/null || true
pkill -f "global_localization" 2>/dev/null || true
pkill -f "wheeltec_robot" 2>/dev/null || true
pkill -f "hesai" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 1

# ---- 1. Wheeltec 底盘驱动 (发布 odom_combined→base_footprint, 订阅 /cmd_vel) ----
echo "=== 启动 R550 Plus 底盘驱动 ==="
source /opt/ros/humble/setup.bash
source "${WHEELTEC_WS}/install/setup.bash"
ros2 launch turn_on_wheeltec_robot base_serial.launch.py \
    car_mode:="${CAR_MODE}" &
sleep 2

# ---- 2. odom_combined→odom TF 桥接 ----
# wheeltec 驱动发布 odom_combined→base_footprint
# FAST-LIO2 发布 odom→base_imu → (bridge)→ base_footprint
echo "=== 建立 odom_combined→odom TF 桥接 ==="
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom odom_combined &
sleep 1
# URDF 由 ego_nav.launch.py 的 robot_state_publisher 提供
# (autoware_akm_ultra_single_antenna_robot.urdf, 用户确认)
# 不再单独启动 robot_mode_description.launch.py (避免双 robot_state_publisher 冲突)

# ---- 3. Hesai LiDAR 驱动 ----
# TODO: 替换为实际的 Hesai 16线 LiDAR 驱动启动命令
# 驱动需发布 /point_cloud_raw 话题
echo "=== 启动 Hesai LiDAR 驱动 (需确认驱动包名和launch文件) ==="
# source "${FASTLIO_WS}/install/setup.bash"
# ros2 launch hesai_ros_driver hesai_lidar.launch.py &
sleep 1

# ---- 4. 导航系统 (FAST-LIO + open3d_loc + EGO + A*) ----
echo "=== 启动导航系统 ==="
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
source "${FASTLIO_WS}/install/setup.bash"

ros2 launch octo_planner ego_nav.launch.py \
    launch_localization:=true \
    use_sim:=false \
    pcd_map_path:="${PCD_FILE}" \
    ego_bridge.use_sim_cloud:=False \
    ego_bridge.lidar_topic:="${LIDAR_TOPIC}" \
    ackermann_tracker.base_frame:=base_footprint \
    ego_bridge.base_frame:=base_footprint &
LAUNCH_PID=$!

sleep 5

echo ""
echo "=============================================="
echo "  R550 Plus 实机导航已启动"
echo ""
echo "  TF 链: map→odom→base_footprint→base_link"
echo "  定位: FAST-LIO2 + open3d_loc ICP"
echo "  全局: A* on OctoMap"
echo "  局部: EGO-Planner"
echo "  控制: AckermannTracker → /cmd_vel → 底盘"
echo ""
echo "  操作:"
echo "    1. RViz → 2D Pose Estimate 给初始位姿"
echo "    2. open3d_loc 自动 ICP 精确定位"
echo "    3. RViz → 2D Goal Pose 设目标"
echo "    4. 自动出发"
echo ""
echo "  注意: 首次启动需先 source 对应 workspace"
echo "=============================================="

wait $LAUNCH_PID
