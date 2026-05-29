# R550 Plus 阿克曼 3D 导航系统

基于 ROS 2 Humble，全局 A* + OctoMap + EGO-Planner + FAST-LIO2 + open3d_loc

## 硬件配置

- 底盘: Wheeltec R550 Plus (阿克曼转向)
- LiDAR: Hesai 16 线旋转式
- IMU: STM32 (底盘内置)
- 外参: LiDAR → IMU 平移 `[-0.4, 0.14, 0.0]` m, 无旋转

## 快速开始

```bash
git clone https://github.com/UNDFGS/wheeltec_jie_test.git
cd wheeltec_jie_test/repo
bash setup_workspace.sh
```

## 仿真

```bash
source /opt/ros/humble/setup.bash
source ~/integrated_ws/install/setup.bash
bash start_ego_sim.sh
```

RViz 中: 2D Pose Estimate → 2D Goal Pose → 自动规划导航

## 实机

```bash
source /opt/ros/humble/setup.bash
source ~/integrated_ws/install/setup.bash
bash start_real.sh /path/to/map.pcd
```

操作: 2D Pose Estimate 给初值 → 等 open3d_loc ICP 收敛 → 2D Goal Pose 出发

## 架构

```
全局规划:  A* on OctoMap (jie_octomap)
局部规划:  EGO-Planner B样条优化避障
重定位:    FAST-LIO2 LiDAR-IMU 里程计 + open3d_loc ICP 匹配 PCD 地图
控制:      Stanley 控制器 → /cmd_vel → 底盘
```

TF 链: `map → odom → base_footprint → base_link`

## 仿真 vs 实机模式

| | 仿真 | 实机 |
|---|---|---|
| 定位 | 静态 TF `map=odom` | FAST-LIO2 + open3d_loc |
| 运动 | 运动学模拟器 | 底盘驱动 `/cmd_vel` |
| 点云 | OctoMap 采样 | LiDAR `/point_cloud_raw` |
| launch | `start_ego_sim.sh` | `start_real.sh` |
| 参数 | `use_sim:=true` | `use_sim:=false launch_localization:=true` |

## 关键文件

```
repo/
├── octo_planner/launch/ego_nav.launch.py    # 导航主 launch
├── octo_planner/scripts/ackermann_tracker.py # Stanley 控制器
├── octo_planner/scripts/ego_bridge.py        # EGO 桥接
├── octo_planner/scripts/kinematic_simulator.py # 阿克曼运动学模拟
├── octo_planner/urdf/autoware_akm_ultra_single_antenna_robot.urdf
├── third_party/fast_lio/                     # FAST-LIO2 源码
├── config/fastlio_wheeltec_params.yaml       # FAST-LIO2 配置
├── start_real.sh                             # 实机一键启动
├── start_ego_sim.sh                          # A*+EGO 仿真
├── start_akm_sim.sh                          # A* only 仿真
└── setup_workspace.sh                        # 新设备部署
```

## 依赖

| 模块 | 来源 | 方式 |
|---|---|---|
| jie_3d_nav | 本仓库 | 内置 |
| FAST-LIO2 | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) (ROS2 fork) | `third_party/` 内置 |
| EGO-Planner | [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) ros2_version | 部署脚本自动 clone |
| open3d_loc | [deepglint/FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID) humble | 部署脚本自动 clone |
| Hesai 驱动 | 厂商 | 用户自行安装 |
| Wheeltec 驱动 | 厂商 | 用户自行安装 |

## 参数调优

### 阿克曼控制 (`ego_nav.launch.py`)

| 参数 | 默认 | 说明 |
|---|---|---|
| `wheel_base` | 0.527 m | 轴距 |
| `max_steering_angle` | 0.785 rad | 最大转角 45° |
| `max_speed` | 0.60 m/s | 最大速度 |
| `goal_tolerance` | 0.3 m | 到达判定 |
| `k_cross_track` | 0.8 | 横向修正强度 |
| `k_heading` | 1.0 | 航向修正强度 |
| `lookahead_dist` | 1.5 m | 前视距离 |

### FAST-LIO2 (`config/fastlio_wheeltec_params.yaml`)

| 参数 | 值 |
|---|---|
| LiDAR 话题 | `/point_cloud_raw` |
| IMU 话题 | `/imu/data_raw` |
| 外参平移 | `[-0.4, 0.14, 0.0]` |
| 外参旋转 | 单位矩阵 |
| IMU 噪声 | `acc_cov: 0.01, gyr_cov: 0.01` |
