"""
轮式机器人 OctoMap 导航仿真
- 差速驱动机器人 URDF + 运动学模拟器
- PCD → OctoMap → A* 路径规划 → AckermannTracker → /cmd_vel → 模拟器
- RViz 可视化：OctoMap + 机器人模型 + 路径 + 跟踪点
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    octo_planner_share = get_package_share_directory("octo_planner")
    jie_octomap_share = get_package_share_directory("jie_octomap")

    # ---- 默认 PCD 路径 ----
    pcd_map_path_arg = DeclareLaunchArgument(
        "pcd_map_path",
        default_value="/home/und/桌面/merged_all_clean.pcd",
        description="PCD 点云地图路径",
    )

    # ---- 机器人模型 ----
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="autoware_akm_ultra_single_antenna_robot",
        description="Wheeltec URDF 模型名 (不含 .urdf 后缀)",
    )

    # ---- 加载 URDF ----
    wheeltec_share = get_package_share_directory("wheeltec_robot_urdf")
    urdf_file = os.path.join(wheeltec_share, "urdf", "autoware_akm_ultra_single_antenna_robot.urdf")
    if not os.path.isfile(urdf_file):
        # 回退到自带简化模型
        urdf_file = os.path.join(octo_planner_share, "urdf", "wheeled_robot.urdf")
    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # 发布所有可动关节的默认状态（轮子转动/转向角 = 0）
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False, "rate": 50}],
    )

    # ---- TF: map → odom (静态，模拟无漂移定位) ----
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
    )

    # ---- 运动学模拟器: odom → base_link (积分 cmd_vel) ----
    kinematic_simulator = Node(
        package="octo_planner",
        executable="kinematic_simulator.py",
        name="kinematic_simulator",
        output="screen",
        parameters=[{
            "wheel_base": 0.527,     # R550 Plus 轴距
            "track_width": 0.40,
            "max_steering_angle": 0.785,  # ~45deg, 最小转弯半径 ~0.53m
            "min_creep_speed": 0.12,      # 阿克曼转向最小蠕行
            "publish_rate": 50.0,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "initial_x": 0.0,
            "initial_y": 0.0,
            "initial_yaw_deg": 0.0,
        }],
    )

    # ---- PCD → OctoMap ----
    pcd_to_octomap = Node(
        package="jie_octomap",
        executable="pcd_to_octomap_node",
        name="pcd_to_octomap",
        output="screen",
        parameters=[{
            "pcd_file_cmd_topic": "/pcd_file_cmd",
            "octomap_topic": "/octomap",
            "frame_id": "map",
            "resolution": 0.2,
            "voxel_downsample_m": 0.1,
            "min_points_per_voxel": 2,
            "min_cluster_voxels": 2,
        }],
    )

    # ---- 地图包管理器 ----
    map_package_manager = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
    )

    # ---- OctoMap 可视化 ----
    occupied_marker = Node(
        package="jie_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[{
            "octomap_topic": "/octomap",
            "marker_topic": "/octomap_occupied_markers",
            "frame_id": "map",
        }],
    )

    # ---- A* 全局路径规划 ----
    planner = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        parameters=[{
            "octomap_topic": "/octomap",
            "start_topic": "/start_point",
            "goal_topic": "/goal_point",
            "path_topic": "/planned_path",
            "path_marker_topic": "/planned_path_marker",
            "preblocked_marker_topic": "/preblocked_cells_markers",
            "traversable_marker_topic": "/traversable_cells_markers",
            "risk_cost_topic": "/risk_cost_cells",
            "frame_id": "map",
            "map_id": "sim_map",
            "source_world_file": "",
            "robot_radius": 0.30,
            "max_iterations": 500000,
            "snap_search_radius_cells": 12,
            "require_ground_support": True,
            "strict_direct_ground_support": False,
            "ground_support_xy_radius_cells": 1,
            "ground_support_depth_cells": 1,
            "lowest_traversable_only": True,
            "enable_preblocked_costmap": False,
        }],
    )

    # ---- AckermannTracker (Stanley 控制器，专为阿克曼) ----
    controller = Node(
        package="octo_planner",
        executable="ackermann_tracker.py",
        name="ackermann_tracker",
        output="screen",
        parameters=[{
            "wheel_base": 0.527,
            "max_steering_angle": 0.785,
            "min_turning_radius": 0.53,
            "max_speed": 0.60,
            "min_speed": 0.10,
            "k_cross_track": 0.8,
            "k_heading": 1.0,
            "lookahead_dist": 1.5,
            "goal_tolerance": 0.3,
            "replan_deviation": 3.0,
            "control_rate": 20.0,
            "map_frame": "map",
            "base_frame": "base_link",
        }],
    )

    # ---- map_viewer_gui ----
    map_viewer_gui = Node(
        package="jie_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        parameters=[{
            "tf_parent_frame": "map",
            "tf_child_frame": "base_link",
        }],
    )

    # ---- RViz2 ----
    rviz_config = os.path.join(octo_planner_share, "rviz", "sim_nav.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # ---- 启动顺序：先加载 PCD，再启动 RViz ----
    return LaunchDescription([
        pcd_map_path_arg,
        robot_model_arg,
        robot_state_publisher,
        joint_state_publisher,
        static_map_to_odom,
        kinematic_simulator,
        pcd_to_octomap,
        map_package_manager,
        occupied_marker,
        planner,
        controller,
        map_viewer_gui,
        # RViz 延迟启动，等 OctoMap 先发布
        TimerAction(period=5.0, actions=[rviz]),
    ])
