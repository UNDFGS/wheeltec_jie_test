#!/usr/bin/env bash
# ================================================================
# 一键部署脚本: jie_3d_nav + FAST-LIO2 + open3d_loc + EGO-Planner
#
# 使用方法 (新机器上):
#   git clone https://github.com/UNDFGS/jie_3d_wheeltec.git
#   cd jie_3d_wheeltec
#   bash setup_workspace.sh
#
# 将创建:
#   ~/fastlio2/      FAST-LIO2 LiDAR-IMU 里程计
#   ~/integrated_ws/  导航 + 重定位 + EGO-Planner
# ================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

echo "========================================"
echo "  jie_3d_nav 一键部署"
echo "  ROS 2: ${ROS_DISTRO}"
echo "========================================"

# ---- 0. 安装系统依赖 ----
echo ""
echo "[0/5] 安装系统依赖..."
bash "${SCRIPT_DIR}/install_deps_humble.sh"

# ---- 1. FAST-LIO2 Workspace (~/fastlio2) ----
echo ""
echo "[1/5] 部署 FAST-LIO2..."

FASTLIO_WS="${HOME}/fastlio2"
FASTLIO_SRC="${FASTLIO_WS}/src"

mkdir -p "${FASTLIO_SRC}"

if [ ! -d "${FASTLIO_SRC}/fast_lio" ]; then
    git clone --depth 1 \
        https://github.com/hku-mars/FAST_LIO.git \
        "${FASTLIO_SRC}/fast_lio" 2>&1
    echo "[OK] FAST-LIO cloned"
else
    echo "[OK] FAST-LIO already exists"
fi

# 复制定制配置 (R550 Plus + Hesai 16线外参)
cp "${SCRIPT_DIR}/config/fastlio_wheeltec_params.yaml" \
   "${FASTLIO_SRC}/fast_lio/config/wheeltec_params.yaml"
echo "[OK] FAST-LIO config installed"

echo "编译 FAST-LIO2 workspace..."
cd "${FASTLIO_WS}"
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select fast_lio 2>&1
echo "[OK] FAST-LIO2 编译完成"

# ---- 2. 集成 Workspace (~/integrated_ws) ----
echo ""
echo "[2/5] 部署集成 workspace..."

INTEGRATED_WS="${HOME}/integrated_ws"
INTEGRATED_SRC="${INTEGRATED_WS}/src"

mkdir -p "${INTEGRATED_SRC}"

# 2a. 链接主仓库
if [ ! -L "${INTEGRATED_SRC}/jie_3d_nav" ] && [ ! -d "${INTEGRATED_SRC}/jie_3d_nav" ]; then
    ln -sf "${SCRIPT_DIR}" "${INTEGRATED_SRC}/jie_3d_nav"
    echo "[OK] jie_3d_nav linked"
else
    echo "[OK] jie_3d_nav already linked"
fi

# 2b. EGO-Planner (ros2_version 分支)
echo ""
echo "[3/5] 克隆 EGO-Planner..."
if [ ! -d "${INTEGRATED_SRC}/ego_planner" ]; then
    git clone --depth 1 --branch ros2_version --single-branch \
        https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git \
        "${INTEGRATED_SRC}/ego_planner" 2>&1
    echo "[OK] EGO-Planner cloned"
else
    echo "[OK] EGO-Planner already exists"
fi

# 2c. open3d_loc (FAST_LIO_LOCALIZATION_HUMANOID, humble 分支)
echo ""
echo "[4/5] 克隆 open3d_loc..."
if [ ! -d "${INTEGRATED_SRC}/FAST_LIO_LOCALIZATION_HUMANOID" ]; then
    git clone --depth 1 --branch humble --single-branch \
        https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git \
        "${INTEGRATED_SRC}/FAST_LIO_LOCALIZATION_HUMANOID" 2>&1
    echo "[OK] open3d_loc cloned"
else
    echo "[OK] open3d_loc already exists"
fi

# 配置 Open3D 路径
FASTLIO_DIR="${INTEGRATED_SRC}/FAST_LIO_LOCALIZATION_HUMANOID"
if [ -f "${FASTLIO_DIR}/open3d_loc/CMakeLists.txt" ]; then
    O3D_CMAKE_DIR=""
    for candidate in \
        /usr/lib/x86_64-linux-gnu/cmake/Open3D \
        /usr/local/lib/cmake/Open3D; do
        if [ -f "${candidate}/Open3DConfig.cmake" ]; then
            O3D_CMAKE_DIR="${candidate}"
            break
        fi
    done
    if [ -n "${O3D_CMAKE_DIR}" ]; then
        sed -i "s|set(Open3D_DIR .*)|set(Open3D_DIR \"${O3D_CMAKE_DIR}\")|" \
            "${FASTLIO_DIR}/open3d_loc/CMakeLists.txt"
        echo "[OK] Open3D configured: ${O3D_CMAKE_DIR}"
    else
        echo "[WARN] Open3D CMake config not found, may need manual setup"
    fi
fi

# 2d. wheeltec_robot_urdf (包含 autoware_akm URDF 的 ROS 包)
# 从 wheeltec 厂商包中提取，或直接使用 octo_planner 内置版
if [ ! -d "${INTEGRATED_SRC}/wheeltec_robot_urdf" ]; then
    echo "[INFO] wheeltec_robot_urdf package not found."
    echo "  octo_planner 已内置 autoware_akm_ultra_single_antenna_robot.urdf"
    echo "  若需完整 wheeltec URDF 包，请从 wheeltec 厂商获取并放入:"
    echo "  ${INTEGRATED_SRC}/wheeltec_robot_urdf"
fi

# ---- 3. 编译 ----
echo ""
echo "[5/5] 编译 integrated_ws..."
cd "${INTEGRATED_WS}"
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install \
    --packages-select jie_map_msgs jie_octomap octo_planner \
                     ego_planner open3d_loc 2>&1

echo ""
echo "========================================"
echo "  部署完成!"
echo ""
echo "  快速启动:"
echo ""
echo "  仿真 (A* + EGO):"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ~/integrated_ws/install/setup.bash"
echo "    bash ~/integrated_ws/src/jie_3d_nav/start_ego_sim.sh"
echo ""
echo "  仿真 (A* only):"
echo "    bash ~/integrated_ws/src/jie_3d_nav/start_akm_sim.sh"
echo ""
echo "  实机 (需 LiDAR + 底盘):"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ~/fastlio2/install/setup.bash"
echo "    source ~/integrated_ws/install/setup.bash"
echo "    bash ~/integrated_ws/src/jie_3d_nav/start_real.sh /path/to/map.pcd"
echo "========================================"
