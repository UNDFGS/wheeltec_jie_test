#!/usr/bin/env bash
# ================================================================
# 一键部署脚本: jie_3d_nav + FAST-LIO2 + open3d_loc + EGO-Planner
#
# 使用方法 (新机器上):
#   git clone https://github.com/UNDFGS/wheeltec_jie_test.git
#   cd wheeltec_jie_test/repo
#   bash setup_workspace.sh
#
# 仅需一个 workspace: ~/integrated_ws
#   - jie_3d_nav (主仓库, symlink)
#   - fast_lio (FAST-LIO2, repo 内置)
#   - ego_planner (EGO-Planner, git clone)
#   - FAST_LIO_LOCALIZATION_HUMANOID (open3d_loc, git clone)
# ================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

echo "========================================"
echo "  jie_3d_nav 一键部署"
echo "  ROS 2: ${ROS_DISTRO}"
echo "  Repo:  ${SCRIPT_DIR}"
echo "========================================"

# ---- 0. 安装系统依赖 ----
echo ""
echo "[0/5] 安装系统依赖..."
if [ -f "${SCRIPT_DIR}/install_deps_humble.sh" ]; then
    bash "${SCRIPT_DIR}/install_deps_humble.sh"
else
    echo "[WARN] install_deps_humble.sh not found, skipping apt deps"
fi

# ---- 1. 创建工作区 ----
echo ""
echo "[1/5] 创建 workspace..."

WS="${HOME}/integrated_ws"
SRC="${WS}/src"
mkdir -p "${SRC}"

# ---- 2. 链接/复制所有包到 workspace ----
echo ""
echo "[2/5] 部署源码包..."

# 2a. jie_3d_nav 主仓库 (symlink)
if [ ! -L "${SRC}/jie_3d_nav" ] && [ ! -d "${SRC}/jie_3d_nav" ]; then
    ln -sf "${SCRIPT_DIR}" "${SRC}/jie_3d_nav"
    echo "[OK] jie_3d_nav linked"
else
    echo "[OK] jie_3d_nav already exists"
fi

# 2b. FAST-LIO2 (repo 内置, 已含定制 wheeltec_params.yaml)
if [ ! -d "${SRC}/fast_lio" ]; then
    cp -r "${SCRIPT_DIR}/third_party/fast_lio" "${SRC}/fast_lio"
    echo "[OK] fast_lio (FAST-LIO2) installed from repo"
else
    echo "[OK] fast_lio already exists"
fi

# 2c. EGO-Planner (git clone)
echo ""
echo "[3/5] 克隆 EGO-Planner..."
if [ ! -d "${SRC}/ego_planner" ]; then
    git clone --depth 1 --branch ros2_version --single-branch \
        https://github.com/ZJU-FAST-Lab/ego-planner-swarm.git \
        "${SRC}/ego_planner" 2>&1
    echo "[OK] EGO-Planner cloned"
else
    echo "[OK] EGO-Planner already exists"
fi

# 2d. open3d_loc + bundled FAST_LIO (git clone)
echo ""
echo "[4/5] 克隆 open3d_loc (ICP 重定位)..."
if [ ! -d "${SRC}/FAST_LIO_LOCALIZATION_HUMANOID" ]; then
    git clone --depth 1 --branch humble --single-branch \
        https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git \
        "${SRC}/FAST_LIO_LOCALIZATION_HUMANOID" 2>&1
    echo "[OK] open3d_loc cloned"
else
    echo "[OK] open3d_loc already exists"
fi

# 配置 Open3D (open3d_loc 编译依赖)
O3D_DIR="${SRC}/FAST_LIO_LOCALIZATION_HUMANOID"
if [ -f "${O3D_DIR}/open3d_loc/CMakeLists.txt" ]; then
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
            "${O3D_DIR}/open3d_loc/CMakeLists.txt"
        echo "[OK] Open3D: ${O3D_CMAKE_DIR}"
    else
        echo "[WARN] Open3D CMake config not found"
        echo "  sudo apt install libopen3d-dev 或手动编译 Open3D"
    fi
fi

# 2e. wheeltec_robot_urdf (可选)
if [ ! -d "${SRC}/wheeltec_robot_urdf" ]; then
    echo "[INFO] wheeltec_robot_urdf not found — octo_planner 已内置 URDF, 无需额外安装"
fi

# ---- 3. 编译 ----
echo ""
echo "[5/5] 编译所有包..."
cd "${WS}"
source /opt/ros/${ROS_DISTRO}/setup.bash

# 先编译导航+FAST-LIO2 (核心)
colcon build --symlink-install \
    --packages-select jie_map_msgs jie_octomap octo_planner fast_lio 2>&1

# 再编译 EGO-Planner + open3d_loc
colcon build --symlink-install \
    --packages-select ego_planner open3d_loc 2>&1

echo ""
echo "========================================"
echo "  部署完成!  所有组件在一个 workspace:"
echo "  ${WS}"
echo ""
echo "  仿真启动 (A* + EGO):"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ${WS}/install/setup.bash"
echo "    bash ${SRC}/jie_3d_nav/start_ego_sim.sh"
echo ""
echo "  实机启动 (需 LiDAR + 底盘):"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ${WS}/install/setup.bash"
echo "    bash ${SRC}/jie_3d_nav/start_real.sh /path/to/map.pcd"
echo ""
echo "  包含组件:"
echo "    全局规划:  A* on OctoMap (jie_octomap)"
echo "    局部规划:  EGO-Planner (B样条优化避障)"
echo "    重定位:    FAST-LIO2 (LiDAR-IMU里程计) + open3d_loc (ICP)"
echo "    控制:      Stanley Ackermann Tracker"
echo "========================================"
