cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 设置编译选项
add_compile_options(-std=c++17 -O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 基础依赖包
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  cv_bridge
)

find_package(ament_cmake REQUIRED)
foreach(pkg ${dependencies})
  find_package(${pkg} REQUIRED)
endforeach()

# 第三方库依赖
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(PythonLibs REQUIRED)  # 修复 Python.h 问题
find_package(OpenMP REQUIRED)    # 修复 omp_get_wtime 问题

# 添加这两行
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)  # 添加 OpenCV
find_package(cv_bridge REQUIRED)        # ← 新增这一行
find_package(g2o REQUIRED)

find_package(patchworkpp REQUIRED) # 添加 patchwork++ 依赖


# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Pose6D.msg")
ament_export_dependencies(rosidl_default_runtime)

# ------------------------------------------------------------------------------
# 1. Fast-LIO2 核心节点
# ------------------------------------------------------------------------------
add_executable(fastlio_mapping 
  src/laserMapping.cpp 
  include/ikd-Tree/ikd_Tree.cpp 
  src/preprocess.cpp
  src/Scancontext.cpp   # 加这一行
)

target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}          # ← 新增这一行
  ${cv_bridge_INCLUDE_DIRS}   # ← 新增这一行
)

target_link_libraries(fastlio_mapping 
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES} 
  Eigen3::Eigen
  OpenMP::OpenMP_CXX  # 链接 OpenMP
  ${OpenCV_LIBS}                  # ← 新增这一行
  ${cv_bridge_LIBRARIES}      # ← 新增这一行
  /opt/ros/humble/include 
  g2o::core                 # <--- 注意：改成 g2o::core
  g2o::stuff                # <--- 改成 g2o::stuff
  g2o::types_slam3d         # <--- 改成 g2o::types_slam3d
  g2o::solver_eigen         # <--- 改成 g2o::solver_eigen
  patchworkpp::ground_seg_cores
)

# ------------------------------------------------------------------------------
# 2. GPS 处理节点
# ------------------------------------------------------------------------------
add_executable(gps_odom_node src/simpleGpsOdom.cpp)

target_include_directories(gps_odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(gps_odom_node 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen
  OpenMP::OpenMP_CXX  # 链接 OpenMP
)


# ------------------------------------------------------------------------------
# 3. KITTI 导出节点
# ------------------------------------------------------------------------------
add_executable(kitti_exporter toolkit/kitti_exporter.cpp)

target_include_directories(kitti_exporter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(kitti_exporter
  Eigen3::Eigen
)

ament_target_dependencies(kitti_exporter
  rclcpp
  nav_msgs
)

install(TARGETS kitti_exporter DESTINATION lib/${PROJECT_NAME})




ament_target_dependencies(fastlio_mapping ${dependencies})
ament_target_dependencies(gps_odom_node ${dependencies})

# 自定义接口链接
list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")
if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(fastlio_mapping ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rosidl_target_interfaces(gps_odom_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
  target_link_libraries(gps_odom_node ${cpp_typesupport_target})
endif()

# 安装
install(TARGETS fastlio_mapping gps_odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ config launch urdf DESTINATION share/${PROJECT_NAME})



ament_package()
