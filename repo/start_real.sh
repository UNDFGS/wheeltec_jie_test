#!/usr/bin/env bash
# ============================================================
# R550 Plus 实机部署 (FAST-LIO + LiDAR + EGO-Planner)
#
# 所有组件在 ~/integrated_ws 单一 workspace 内:
#   - fast_lio:       FAST-LIO2 LiDAR-IMU 里程计
#   - open3d_loc:     ICP 全局重定位
#   - ego_planner:    EGO-Planner 局部避障
#   - octo_planner:   A* 全局规划 + Stanley 控制 + EGO 桥接
#   - jie_octomap:    OctoMap 八叉树管理
#
# 外部依赖 (未包含在 repo):
#   - wheeltec 底盘驱动 (~/wheeltec): 发布 /odom_combined, /imu/data_raw
#   - Hesai LiDAR 驱动:              发布 /point_cloud_raw
#
# 使用前确认:
#   1. wheeltec_param.yaml: car_mode=senior_akm
#   2. LiDAR 驱动已安装并能发布 /point_cloud_raw
#   3. 底盘串口: /dev/wheeltec_controller
#   4. PCD 地图路径
# ============================================================
set -eo pipefail

PCD_FILE="${1:-/home/und/桌面/merged_all_clean.pcd}"
WS_DIR="${HOME}/integrated_ws"
WHEELTEC_WS="${HOME}/wheeltec"
LIDAR_TOPIC="/point_cloud_raw"
CAR_MODE="senior_akm"

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

# ---- 1. Wheeltec 底盘驱动 ----
echo "=== 启动 R550 Plus 底盘驱动 ==="
source /opt/ros/humble/setup.bash
source "${WHEELTEC_WS}/install/setup.bash"
ros2 launch turn_on_wheeltec_robot base_serial.launch.py \
    car_mode:="${CAR_MODE}" &
sleep 2

# ---- 2. odom_combined→odom TF 桥接 ----
echo "=== 建立 odom_combined→odom TF 桥接 ==="
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom odom_combined &
sleep 1

# ---- 3. Hesai LiDAR 驱动 ----
# TODO: 根据实际安装的 Hesai 驱动包修改
echo "=== 启动 Hesai LiDAR 驱动 ==="
echo "  [TODO] 请根据实际驱动包名修改此脚本:"
echo "  例如: ros2 launch hesai_ros_driver hesai_lidar.launch.py"
# ros2 launch hesai_ros_driver hesai_lidar.launch.py &
sleep 1

# ---- 4. 导航系统 ----
echo "=== 启动导航系统 ==="
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

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
echo "  定位: FAST-LIO2 (LiDAR-IMU) + open3d_loc (ICP)"
echo "  全局: A* on OctoMap"
echo "  局部: EGO-Planner (B样条优化)"
echo "  控制: Stanley Ackermann → /cmd_vel → 底盘"
echo ""
echo "  操作步骤:"
echo "    1. RViz → 2D Pose Estimate 给初始位姿"
echo "    2. 等待 open3d_loc ICP 收敛 (查看 /baselink2map)"
echo "    3. RViz → 2D Goal Pose 设目标"
echo "=============================================="

wait $LAUNCH_PID
