"""
EGO-Planner + jie_3d_nav 两级导航仿真

全局层: A* on OctoMap → /planned_path
局部层: EGO-Planner (B样条优化 + 避障) → /ego_local_path
控制层: AckermannTracker (Stanley) → /cmd_vel
模拟层: KinematicSimulator (阿克曼运动学)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    octo_planner_share = get_package_share_directory("octo_planner")
    jie_octomap_share = get_package_share_directory("jie_octomap")
    ego_planner_share = get_package_share_directory("ego_planner")

    pcd_map_path_arg = DeclareLaunchArgument(
        "pcd_map_path",
        default_value="/home/und/桌面/merged_all_clean.pcd",
        description="PCD 点云地图路径",
    )
    launch_localization_arg = DeclareLaunchArgument(
        "launch_localization",
        default_value="false",
        description="启动 FAST-LIO + open3d_loc 重定位 (需要 LiDAR 硬件)",
    )
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="仿真模式 (True=运动模拟+OctoMap点云, False=实机LiDAR+雷达点云)",
    )

    # ---- URDF (Wheeltec R550 Plus) ----
    wheeltec_share = get_package_share_directory("wheeltec_robot_urdf")
    urdf_file = os.path.join(wheeltec_share, "urdf", "autoware_akm_ultra_single_antenna_robot.urdf")
    if not os.path.isfile(urdf_file):
        urdf_file = os.path.join(octo_planner_share, "urdf", "wheeled_robot.urdf")
    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    # ---- 基础 TF + 模型 ----
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_desc}],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        name="joint_state_publisher", output="screen",
    )

    # ---- 定位: 仿真用静态TF / 实机用 FAST-LIO+open3d_loc ----
    # 仿真模式: map→odom 直接等同
    static_map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
        condition=UnlessCondition(LaunchConfiguration("launch_localization")),
    )

    # ---- 阿克曼运动学模拟 (仅仿真) ----
    kinematic_sim = Node(
        package="octo_planner", executable="kinematic_simulator.py",
        name="kinematic_simulator", output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim")),
        parameters=[{
            "wheel_base": 0.527, "track_width": 0.40,
            "max_steering_angle": 0.785, "min_creep_speed": 0.12,
            "publish_rate": 50.0,
            "odom_frame": "odom", "base_frame": "base_link",
            "initial_x": 0.0, "initial_y": 0.0, "initial_yaw_deg": 0.0,
        }],
    )

    # ---- FAST-LIO2 里程计 (实机) ----
    # 使用 wheeltec_params.yaml (已标定 LiDAR→IMU 外参 [-0.4, 0.14, 0.0])
    fast_lio_share = get_package_share_directory("fast_lio")
    fast_lio_cfg = os.path.join(fast_lio_share, "config", "wheeltec_params.yaml")
    fast_lio_node = Node(
        package="fast_lio", executable="fastlio_mapping",
        name="fast_lio", output="screen",
        condition=IfCondition(LaunchConfiguration("launch_localization")),
        parameters=[fast_lio_cfg, {
            "use_sim_time": False,
        }],
    )
    # FAST-LIO2 发布 TF: odom_fast_lio2 → body, 桥接到导航所需的 odom 帧
    fastlio_to_odom_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="fastlio_to_odom_bridge",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "odom_fast_lio2"],
        condition=IfCondition(LaunchConfiguration("launch_localization")),
    )

    # ---- open3d_loc 全局重定位 (ICP匹配预建PCD地图) ----
    # 通过 RViz "2D Pose Estimate" 给初始位姿, ICP 精确定位
    open3d_loc_node = Node(
        package="open3d_loc", executable="global_localization_node",
        name="global_localization_node", output="screen",
        condition=IfCondition(LaunchConfiguration("launch_localization")),
        parameters=[{
            "path_map": LaunchConfiguration("pcd_map_path"),
            "pcd_queue_maxsize": 10,
            "voxelsize_coarse": 0.01,
            "voxelsize_fine": 0.2,
            "threshold_fitness": 0.5,
            "threshold_fitness_init": 0.5,
            "loc_frequence": 2.5,
            "save_scan": False,
            "hidden_removal": False,
            "maxpoints_source": 80000,
            "maxpoints_target": 400000,
            "filter_odom2map": False,
            "kalman_processVar2": 0.001,
            "kalman_estimatedMeasVar2": 0.02,
            "confidence_loc_th": 0.7,
            "dis_updatemap": 3.5,
        }],
    )

    # 实机 TF 链: wheeltec驱动→odom→base_footprint, URDF→base_footprint→base_link
    # FAST-LIO2→odom_fast_lio2→body, 桥接TF: odom→odom_fast_lio2 (由上面 fastlio_to_odom_tf 发布)

    # ---- OctoMap + 全局 A* ----
    pcd_to_octomap = Node(
        package="jie_octomap", executable="pcd_to_octomap_node",
        name="pcd_to_octomap", output="screen",
        parameters=[{
            "pcd_file_cmd_topic": "/pcd_file_cmd", "octomap_topic": "/octomap",
            "frame_id": "map", "resolution": 0.2,
            "voxel_downsample_m": 0.1, "min_points_per_voxel": 2,
            "min_cluster_voxels": 2,
        }],
    )
    map_pkg_mgr = Node(
        package="jie_octomap", executable="map_package_manager",
        name="map_package_manager", output="screen",
    )
    occupied_marker = Node(
        package="jie_octomap", executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers", output="screen",
        parameters=[{
            "octomap_topic": "/octomap",
            "marker_topic": "/octomap_occupied_markers", "frame_id": "map",
        }],
    )
    global_planner = Node(
        package="octo_planner", executable="jie_path_node",
        name="jie_path_node", output="screen",
        parameters=[{
            "octomap_topic": "/octomap",
            "start_topic": "/start_point", "goal_topic": "/goal_point",
            "path_topic": "/planned_path",
            "path_marker_topic": "/planned_path_marker",
            "preblocked_marker_topic": "/preblocked_cells_markers",
            "traversable_marker_topic": "/traversable_cells_markers",
            "risk_cost_topic": "/risk_cost_cells",
            "frame_id": "map", "map_id": "sim_map",
            "robot_radius": 0.30, "max_iterations": 500000,
            "snap_search_radius_cells": 12,
            "require_ground_support": True,
            "strict_direct_ground_support": False,
            "ground_support_xy_radius_cells": 1,
            "ground_support_depth_cells": 1,
            "lowest_traversable_only": True,
            "enable_preblocked_costmap": False,
        }],
    )

    # ---- 阿克曼跟踪器 ----
    ackermann_tracker = Node(
        package="octo_planner", executable="ackermann_tracker.py",
        name="ackermann_tracker", output="screen",
        parameters=[{
            "wheel_base": 0.527, "max_steering_angle": 0.785,
            "min_turning_radius": 0.53,
            "max_speed": 0.60, "min_speed": 0.10,
            "k_cross_track": 0.8, "k_heading": 1.0,
            "lookahead_dist": 1.5, "goal_tolerance": 0.3,
            "replan_deviation": 3.0, "control_rate": 20.0,
            "map_frame": "map", "base_frame": "base_link",
        }],
    )

    # ---- GUI ----
    map_viewer_gui = Node(
        package="jie_octomap", executable="map_viewer_gui",
        name="map_viewer_gui", output="screen",
        parameters=[{
            "tf_parent_frame": "map", "tf_child_frame": "base_link",
        }],
    )

    # ---- EGO-Planner 局部规划层 ----
    ego_planner_node = Node(
        package="ego_planner", executable="ego_planner_node",
        name="ego_planner_node", output="screen",
        remappings=[
            ("grid_map/odom", "/grid_map/odom"),
            ("odom_world", "/grid_map/odom"),
            ("grid_map/cloud", "/grid_map/cloud"),
            ("planning/bspline", "/planning/bspline"),
            ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
        ],
        parameters=[{
            # FSM
            "fsm/flight_type": 2,
            "fsm/thresh_replan_time": 1.0,
            "fsm/thresh_no_replan_meter": 1.0,
            "fsm/planning_horizon": 10.0,
            "fsm/planning_horizen_time": 5.0,
            "fsm/emergency_time": 1.0,
            "fsm/realworld_experiment": False,
            "fsm/fail_safe": True,
            "fsm/waypoint_num": 0,
            # Grid map (局部感知范围)
            "grid_map/resolution": 0.1,
            "grid_map/map_size_x": 60.0,
            "grid_map/map_size_y": 40.0,
            "grid_map/map_size_z": 5.0,
            "grid_map/local_update_range_x": 8.0,
            "grid_map/local_update_range_y": 8.0,
            "grid_map/local_update_range_z": 3.0,
            "grid_map/obstacles_inflation": 0.1,
            "grid_map/local_map_margin": 10,
            "grid_map/ground_height": -0.1,
            "grid_map/virtual_ceil_height": 2.0,
            "grid_map/visualization_truncate_height": 1.5,
            "grid_map/frame_id": "map",
            "grid_map/pose_type": 1,  # use odom pose
            "grid_map/p_hit": 0.65, "grid_map/p_miss": 0.35,
            "grid_map/p_min": 0.12, "grid_map/p_max": 0.90,
            "grid_map/p_occ": 0.80,
            "grid_map/min_ray_length": 0.1, "grid_map/max_ray_length": 5.0,
            # Planner manager
            "manager/max_vel": 1.0,
            "manager/max_acc": 1.5,
            "manager/max_jerk": 3.0,
            "manager/control_points_distance": 0.4,
            "manager/feasibility_tolerance": 0.05,
            "manager/planning_horizon": 10.0,
            "manager/use_distinctive_trajs": False,
            "manager/drone_id": 0,
            # Optimization
            "optimization/lambda_smooth": 1.0,
            "optimization/lambda_collision": 0.5,
            "optimization/lambda_feasibility": 0.1,
            "optimization/lambda_fitness": 1.0,
            "optimization/dist0": 0.5,
            "optimization/swarm_clearance": 0.5,
            "optimization/max_vel": 1.0,
            "optimization/max_acc": 1.5,
            # B-Spline
            "bspline/limit_vel": 1.0,
            "bspline/limit_acc": 1.5,
            "bspline/limit_ratio": 1.1,
            # Prediction
            "prediction/obj_num": 0,
            "prediction/lambda": 1.0,
            "prediction/predict_rate": 1.0,
        }],
    )

    # ---- 桥接节点 (OctoMap→点云, A*→waypoint, TF→odom) ----
    ego_bridge = Node(
        package="octo_planner", executable="ego_bridge.py",
        name="ego_bridge", output="screen",
        parameters=[{
            "map_frame": "map", "base_frame": "base_link",
            "cloud_publish_rate": 5.0,
            "cloud_sample_count": 8000,
            "cloud_range": 30.0,
        }],
    )

    # ---- RViz2 ----
    rviz_config = os.path.join(octo_planner_share, "rviz", "sim_nav.rviz")
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        pcd_map_path_arg,
        launch_localization_arg,
        use_sim_arg,
        robot_state_publisher,
        joint_state_publisher,
        static_map_to_odom,
        kinematic_sim,
        fast_lio_node,
        fastlio_to_odom_tf,
        open3d_loc_node,
        pcd_to_octomap,
        map_pkg_mgr,
        occupied_marker,
        global_planner,
        ackermann_tracker,
        map_viewer_gui,
        ego_planner_node,
        ego_bridge,
        TimerAction(period=5.0, actions=[rviz]),
    ])
