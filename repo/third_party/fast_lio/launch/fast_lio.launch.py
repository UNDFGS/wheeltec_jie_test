import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # ===================== 1. 路径初始化 =====================
    # 获取 fast_lio 功能包的安装路径
    package_path = get_package_share_directory('fast_lio')
    
    # 获取 wheeltec urdf 模型包的安装路径
    # 注意：确保你已经 colcon build 过这个包
    urdf_pkg_path = get_package_share_directory('wheeltec_robot_urdf')

    # 设定参数文件的默认位置 (install/fast_lio/share/fast_lio/config/)
    default_config_path = os.path.join(package_path, 'config')
    
    # 【关键修改】：将默认文件名改为你实际使用的 'wheeltec_params.yaml'
    default_config_file = 'wheeltec_params.yaml'

    # 设置默认的 URDF 模型文件（根据你之前的 ls 结果，默认使用 senior_akm）
    default_robot_model = 'wheeltec_3d.urdf'
    urdf_file_path = '/home/und/fastlio2/src/fast_lio/urdf/wheeltec_3d.urdf'
        # RViz配置文件路径
    rviz_config_path = '/home/und/fastlio2/src/fast_lio/rviz/und_rviz_config.rviz'

    # ===================== 2. 参数声明 =====================
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')

    # 声明启动参数，允许在命令行通过 key:=value 覆盖
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='是否使用仿真时间（跑数据集/录制的 bag 时设为 true）')

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='配置文件所在目录')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value=default_config_file,
        description='具体的配置文件名')

    # ===================== 3. 节点配置 =====================

    # A. 加载并发布机器人模型 (URDF)
    # 这会发布 base_link -> laser_link 等静态 TF 变换
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # B. Fast-LIO2 核心节点
    fast_lio_node = Node(
       package='fast_lio',
       executable='fastlio_mapping',
       name='fastlio_mapping',
       parameters=[
        PathJoinSubstitution([config_path, config_file]),
        {'use_sim_time': use_sim_time}
       ],
       output='screen',
       sigterm_timeout='12000',   # ⭐ SIGINT→SIGTERM 的等待时间，改为 120 秒
       sigkill_timeout='3000',    # ⭐ SIGTERM→SIGKILL 的等待时间，再给 30 秒兜底
     )
    # C. ⭐ 高级 GPS Odom 节点
    gps_node = Node(
        package='fast_lio',
        executable='gps_odom_node',
        name='gps_odom_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'gpsTopic': '/gps/fix'},
            {'imu_topic': '/imu/data_raw'},
            {'gps_status_threshold': 1},          # ← 改成 1（你的 bag 必须）
            {'filter_alpha': 0.85},
            {'yaw_offset': 0.0},
            {'time_sync_tolerance': 0.05}
        ]
    )
    
     # D. ⭐ RViz2 节点（新增）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]  # -d 参数加载配置文件
    )


    #E. Lio_post2Kitti
    kitti_exporter_node = Node(
    package='fast_lio',
    executable='kitti_exporter',
    name='kitti_exporter',
    output='screen',
    parameters=[{
        'odom_topic': '/Odometry'   # ⚠️ 根据你的实际话题改
    }])

    # ===================== 4. 启动描述符构建 =====================
    ld = LaunchDescription()

    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)

    # 添加运行节点
    ld.add_action(robot_state_publisher_node)
    ld.add_action(fast_lio_node)
    ld.add_action(gps_node)

    ld.add_action(rviz_node)  # 添加RViz节点
    ld.add_action(kitti_exporter_node ) 
    


    return ld
