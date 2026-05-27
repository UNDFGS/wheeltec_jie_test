"""
集成导航 launch 文件
定位: FAST-LIO (LIO) + open3d_loc (ICP 重定位)
导航: jie_octomap (OctoMap) + jie_path_node (A* 规划) + d1_controller (路径跟踪)

TF 链: map → odom → base_link

使用方式:
  # 完整模式 (定位 + 导航)
  ros2 launch octo_planner integrated_nav.launch.py

  # 仅导航 (测试模式，无定位)
  ros2 launch octo_planner integrated_nav.launch.py launch_localization:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 参数声明 ----
    launch_localization_arg = DeclareLaunchArgument(
        "launch_localization",
        default_value="true",
        description="启动 FAST-LIO + open3d_loc 定位",
    )
    launch_planner_arg = DeclareLaunchArgument(
        "launch_planner",
        default_value="true",
        description="启动 jie_path_node 路径规划",
    )
    launch_controller_arg = DeclareLaunchArgument(
        "launch_controller",
        default_value="true",
        description="启动 d1_controller 路径跟踪",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value="true",
        description="启动 map_viewer_gui 地图查看",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="启动 RViz2 可视化",
    )
    pcd_map_path_arg = DeclareLaunchArgument(
        "pcd_map_path",
        default_value="/home/und/桌面/CorrectedFrontendMap_cut.pcd",
        description="PCD 点云地图路径（定位和导航共用）",
    )
    map_package_dir_arg = DeclareLaunchArgument(
        "map_package_dir",
        default_value="/home/und/maps/default",
        description="OctoMap 地图包保存/加载目录",
    )

    # ---- 包路径 ----
    octo_planner_share = get_package_share_directory("octo_planner")
    jie_octomap_share = get_package_share_directory("jie_octomap")

    # ---- 定位模块 (FAST-LIO + open3d_loc) ----
    # 注意: 需要先编译 FAST_LIO_LOCALIZATION_HUMANOID 包
    # 如果包不存在，定位部分会跳过

    # 尝试包含 FAST-LIO 定位
    fastlio_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("open3d_loc"),
                "launch",
                "localization_3d_g1.launch.py",
            ])
        ]),
        condition=IfCondition(LaunchConfiguration("launch_localization")),
    )

    # ---- TF 发布 (测试模式：无实际定位时用静态 TF) ----
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
        condition=UnlessCondition(LaunchConfiguration("launch_localization")),
    )
    static_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "base_link"],
        condition=UnlessCondition(LaunchConfiguration("launch_localization")),
    )

    # ---- PCD 导入（从点云生成 OctoMap） ----
    launch_pcd_import_arg = DeclareLaunchArgument(
        "launch_pcd_import",
        default_value="true",
        description="启动 pcd_to_octomap_node 从 PCD 生成 OctoMap",
    )

    pcd_to_octomap_node = Node(
        package="jie_octomap",
        executable="pcd_to_octomap_node",
        name="pcd_to_octomap",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_pcd_import")),
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

    # ---- 导航模块 ----

    # 地图包管理器 (自动加载已保存的 OctoMap)
    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[{
            "autoload_package_path": LaunchConfiguration("map_package_dir"),
        }],
    )

    # OctoMap 占用体素可视化
    occupied_marker_node = Node(
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

    # 全局路径规划器
    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[{
            "octomap_topic": "/octomap",
            "start_topic": "/start_point",
            "goal_topic": "/goal_point",
            "path_topic": "/planned_path",
            "path_marker_topic": "/planned_path_marker",
            "preblocked_marker_topic": "/preblocked_cells_markers",
            "edited_occupied_marker_topic": "/edited_occupied_markers",
            "traversable_marker_topic": "/traversable_cells_markers",
            "risk_cost_topic": "/risk_cost_cells",
            "frame_id": "map",
            "map_id": "loaded_map",
            "source_world_file": "",
            "robot_radius": 0.35,
            "max_iterations": 500000,
            "snap_search_radius_cells": 12,
            "require_ground_support": True,
            "strict_direct_ground_support": False,
            "ground_support_xy_radius_cells": 1,
            "ground_support_depth_cells": 1,
            "lowest_traversable_only": True,
            "enable_preblocked_costmap": False,
            "preblocked_costmap_radius_cells": 1,
            "preblocked_costmap_weight": 2.5,
        }],
    )

    # 局部路径跟踪控制器
    controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_controller")),
        parameters=[{
            "path_topic": "/planned_path",
            "start_navigation_topic": "/start_navigation",
            "stop_navigation_topic": "/stop_navigation",
            "require_start_command": True,
            "cmd_vel_topic": "/cmd_vel",
            "manual_cmd_vel_topic": "/web_cmd_vel",
            "tracking_point_marker_topic": "/tracking_point_marker",
            "enable_tracking_debug_view": True,
            "map_frame": "map",
            # 使用 base_link（匹配 FAST-LIO 输出）
            "base_frame": "base_link",
            "base_frame_candidates": "base_link",
            # 通用平台，无 D1 偏移
            "robot_center_offset_frame": "base_link",
            "robot_center_offset_x": 0.0,
            "robot_center_offset_y": 0.0,
            "robot_center_offset_z": 0.0,
            # 增益参数
            "linear_gain": 0.8,
            "lateral_gain": 0.6,
            "heading_gain": 0.6,
            "cross_track_angular_gain": 0.4,
            "final_yaw_gain": 0.3,
            "enable_lateral_motion": True,
            "max_linear_speed": 0.5,
            "max_lateral_speed": 0.3,
            "max_angular_speed": 0.8,
            "align_final_yaw": True,
            "goal_position_tolerance": 0.2,
            "goal_yaw_tolerance": 0.15,
        }],
    )

    # 地图查看 GUI（显示路径）
    map_viewer_gui_node = Node(
        package="jie_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
        parameters=[{
            "tf_parent_frame": "map",
            "tf_child_frame": "base_link",
        }],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        arguments=["-d", os.path.join(
            jie_octomap_share, "rviz", "odin1_loc.rviz"
        )],
    )

    return LaunchDescription([
        # 参数
        launch_localization_arg,
        launch_planner_arg,
        launch_controller_arg,
        launch_map_gui_arg,
        launch_rviz_arg,
        pcd_map_path_arg,
        map_package_dir_arg,
        launch_pcd_import_arg,
        # 定位
        pcd_to_octomap_node,
        fastlio_localization,
        static_map_to_odom,
        static_odom_to_base,
        # 导航
        map_package_manager_node,
        occupied_marker_node,
        planner_node,
        controller_node,
        map_viewer_gui_node,
        rviz_node,
    ])
