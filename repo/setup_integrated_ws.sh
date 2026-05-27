#!/usr/bin/env bash
# ================================================================
# 集成 Workspace 编译脚本
# 定位: FAST_LIO_LOCALIZATION_HUMANOID (open3d_loc + fast_lio)
# 导航: jie_3d_nav (jie_octomap + octo_planner)
#
# 使用方法:
#   bash setup_integrated_ws.sh
#   source /opt/ros/humble/setup.bash
#   source ~/integrated_ws/install/setup.bash
#   ros2 launch octo_planner integrated_nav.launch.py
# ================================================================

set -eo pipefail

WS_DIR="${HOME}/integrated_ws"
SRC_DIR="${WS_DIR}/src"
PCD_FILE="${1:-/home/und/桌面/CorrectedFrontendMap_cut.pcd}"

echo "======================================"
echo "  FAST-LIO + jie_3d_nav 集成部署"
echo "  Workspace: ${WS_DIR}"
echo "  PCD 地图: ${PCD_FILE}"
echo "======================================"

# ---- 1. 创建 workspace 目录结构 ----
mkdir -p "${SRC_DIR}"

# ---- 2. 链接 jie_3d_nav ----
JIE_3D_NAV_DIR="$(cd "$(dirname "$0")" && pwd)"
if [ ! -L "${SRC_DIR}/jie_3d_nav" ] && [ ! -d "${SRC_DIR}/jie_3d_nav" ]; then
    ln -sf "${JIE_3D_NAV_DIR}" "${SRC_DIR}/jie_3d_nav"
    echo "[OK] 已链接 jie_3d_nav → ${SRC_DIR}/jie_3d_nav"
else
    echo "[OK] jie_3d_nav 已存在"
fi

# ---- 3. 克隆 FAST_LIO_LOCALIZATION_HUMANOID ----
if [ ! -d "${SRC_DIR}/FAST_LIO_LOCALIZATION_HUMANOID" ]; then
    echo ""
    echo "正在克隆 FAST_LIO_LOCALIZATION_HUMANOID (Humble 分支)..."
    cd "${SRC_DIR}"
    git clone --depth 1 --branch humble --single-branch \
        https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git 2>&1
    echo "[OK] 克隆完成"
else
    echo "[OK] FAST_LIO_LOCALIZATION_HUMANOID 已存在"
fi

# ---- 4. 配置 Open3D ----
FASTLIO_DIR="${SRC_DIR}/FAST_LIO_LOCALIZATION_HUMANOID"
if [ -f "${FASTLIO_DIR}/open3d_loc/CMakeLists.txt" ]; then
    # 查找系统已安装的 Open3D
    O3D_CMAKE_DIR=""
    for candidate in \
        /usr/lib/x86_64-linux-gnu/cmake/Open3D \
        /usr/local/lib/cmake/Open3D \
        "${HOME}/open3d141/lib/cmake/Open3D"; do
        if [ -f "${candidate}/Open3DConfig.cmake" ]; then
            O3D_CMAKE_DIR="${candidate}"
            break
        fi
    done

    if [ -n "${O3D_CMAKE_DIR}" ]; then
        sed -i "s|set(Open3D_DIR .*)|set(Open3D_DIR \"${O3D_CMAKE_DIR}\")|" \
            "${FASTLIO_DIR}/open3d_loc/CMakeLists.txt"
        echo "[OK] Open3D 路径: ${O3D_CMAKE_DIR}"
    else
        echo "[WARN] 未找到 Open3D CMake 配置文件"
        echo "  请手动修改 ${FASTLIO_DIR}/open3d_loc/CMakeLists.txt 中的 Open3D_DIR"
    fi
fi

# ---- 5. 安装系统依赖 ----
echo ""
echo "安装系统依赖..."
echo "ros" | sudo -S apt-get install -y \
    libc++-dev libc++abi-dev \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs \
    libyaml-cpp-dev libboost-filesystem-dev libboost-system-dev \
    2>/dev/null || true
echo "[OK] 系统依赖已安装"

# ---- 6. 复制 PCD 地图到定位目录 ----
MAP_DATA_DIR="${FASTLIO_DIR}/data"
mkdir -p "${MAP_DATA_DIR}"
if [ -f "${PCD_FILE}" ]; then
    cp "${PCD_FILE}" "${MAP_DATA_DIR}/map.pcd"
    echo "[OK] PCD 地图已复制到 ${MAP_DATA_DIR}/map.pcd"

    # 尝试转 ply 格式（open3d_loc 默认用 ply，但也支持 pcd）
    python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('${PCD_FILE}')
o3d.io.write_point_cloud('${MAP_DATA_DIR}/map.ply', pcd)
print('ply 格式转换完成')
" 2>/dev/null && echo "[OK] 已生成 map.ply" || echo "[INFO] 跳过 ply 转换（open3d 支持直接读 pcd）"
else
    echo "[WARN] PCD 文件不存在: ${PCD_FILE}"
    echo "  请手动放置地图文件到 ${MAP_DATA_DIR}/"
fi

# ---- 7. 创建 OctoMap 地图目录 ----
MAP_PKG_DIR="${HOME}/maps/default"
mkdir -p "${MAP_PKG_DIR}"
echo "[OK] OctoMap 地图包目录: ${MAP_PKG_DIR}"

# ---- 8. 编译 ----
echo ""
echo "开始编译 integrated_ws..."
cd "${WS_DIR}"
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
    --packages-select jie_map_msgs jie_octomap octo_planner 2>&1

echo ""
echo "======================================"
echo "  部署完成!"
echo ""
echo "  启动命令:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ${WS_DIR}/install/setup.bash"
echo "    ros2 launch octo_planner integrated_nav.launch.py"
echo ""
echo "  测试模式 (无定位, 用静态 TF):"
echo "    ros2 launch octo_planner integrated_nav.launch.py launch_localization:=false"
echo "======================================"
