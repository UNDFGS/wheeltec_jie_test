// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

// ================== 🔥【2026.3.26 添加回环模块】==================
#include "Scancontext.h"
#include <visualization_msgs/msg/marker_array.hpp>  // 🔥 回环连线可视化所需，MarkerArray 类型的头文件
// =======================================================


// ================== 🔥【2026.3.26 修改：多线程通信所需高级库】起始 ==================
#include <queue>                // 队列，用于充当主线程和后台线程的通信篮子
#include <atomic>               // 原子变量，用于线程安全的退出标志位
#include <condition_variable>   // 条件变量，用于实现瞬间休眠和瞬间唤醒，替代低效的轮询
// ================== 🔥【2026.3.26 修改：多线程通信所需高级库】结束 ==================


// ================== 🔥【2026.3.27 修改：引入坐标变换库】起始 ==================
#include <pcl/common/transforms.h> // 用于将点云从 Body 系转换到 World 系
// ================== 🔥【2026.3.27 修改：引入坐标变换库】结束 ==================

// ================== 🔥【2026.3.27 修改：引入断言库】起始 ==================
#include <cassert> // 用于增加运行时安全检查 (SC 索引同步校验)
// ================== 🔥【2026.3.27 修改：引入断言库】结束 ==================

// ================== 🔥【2026.3.27 新增：引入 GICP 精配准库】起始 ==================
#include <pcl/registration/gicp.h> // 引入 PCL 库自带的 GICP（广义迭代最近点）算法，用于点云精配准
// ================== 🔥【2026.3.27 新增：引入 GICP 精配准库】结束 ==================


// ================== 🔥 2026.3.30 新增 g2o 位姿图优化库 ==================
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <g2o/core/robust_kernel_impl.h>

#include <pcl/kdtree/kdtree_flann.h>

// ================== 🔥【Patchwork++ 地面分割】 ==================
#include <patchwork/patchworkpp.h>   // 注意路径是 patchwork/，扩展名是 .h

// ================== 🔥【新增：计算协方差与质心所需】 ==================
#include <pcl/common/centroid.h>


#include <iomanip> 

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
// bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;

// 替换为：
// ================== 🔥【2026.3.27 修改：修复 flg_exit 的线程安全问题】起始 ==================
bool lidar_pushed, flg_first_scan = true, flg_EKF_inited;
std::atomic<bool> flg_exit{false}; // 改为原子变量，保障信号处理函数与主线程的安全通信
// ================== 🔥【2026.3.27 修改：修复 flg_exit 的线程安全问题】结束 ==================

bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool    is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;


//2026.3.25 关键帧

// ================== 🔥【关键帧】==================

// ================== 🔥【2026.3.30 修改：KeyFrame 增加 LiDAR 位姿和原始里程计位姿】起始 ==================
struct KeyFrame
{
    // --- 可被 PGO 修改的位姿（用于可视化、SC 检索、g2o 顶点初值）---
    Eigen::Vector3d position;     // LiDAR 在世界系的位置（会被 PGO 回写更新）
    Eigen::Matrix3d rotation;     // LiDAR 在世界系的旋转（会被 PGO 回写更新）
    
    // --- 纯净的原始里程计位姿（永远不被 PGO 修改，专门用于 g2o 里程计边计算）---
    Eigen::Vector3d odom_position; // 前端 EKF 算出的 LiDAR 位姿，存入后永不覆写
    Eigen::Matrix3d odom_rotation; // 前端 EKF 算出的 LiDAR 旋转，存入后永不覆写
    
    double time;                  // 该帧的时间戳
    PointCloudXYZI::Ptr cloud;    // 该帧的点云数据（LiDAR body 系）
};
// ================== 🔥【2026.3.30 修改：KeyFrame 增加 LiDAR 位姿和原始里程计位姿】结束 ==================

// ================== 🔥【关键帧】==================

// ================== 🔥【2026.3.27 新增：回环诊断信息结构体】起始 ==================
// 这个结构体用于记录线程3每一次尝试匹配的结果，是日后撰写论文、绘制对比折线图/消融实验的极佳数据源！
struct LoopEdge
{
    int from_id;                        // 发起回环的当前新帧 ID
    int to_id;                          // 被匹配到的历史老帧 ID
    Eigen::Affine3d relative_pose;      // ICP 精配准算出的相对位姿约束矩阵（最终将加入位姿图的边）
    double fitness_score;               // ICP 的匹配质量分数 (均方误差)，越小代表贴合得越完美
    double icp_time_ms;                 // 本次 ICP 消耗的时间 (单位：毫秒)
    bool accepted;                      // 一个布尔标志，记录本次回环是否通过了严苛的校验从而被系统采纳
};
// ================== 🔥【2026.3.27 新增：回环诊断信息结构体】结束 ==================

// ================== 🔥【2026.3.31 新增：回环候选传输结构体，替代 pair<int,int>】起始 ==================
// 用于线程2 → 线程3 的队列传输，携带 SC 距离和偏航角差等额外信息
struct LoopCandidate
{
    int current_id;         // 当前帧 ID
    int loop_id;            // 匹配到的历史帧 ID
    double sc_distance;     // Scan Context 匹配距离（越小越相似）
    float yaw_diff_rad;     // SC 返回的偏航角差（弧度）
    std::string source;     // "SC" 或 "Radius"，标识回环来源
};
// ================== 🔥【2026.3.31 新增：回环候选传输结构体，替代 pair<int,int>】结束 ==================



pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// void SigHandle(int sig)
// {
//     flg_exit = true;
//     std::cout << "catch sig %d" << sig << std::endl;
//     sig_buffer.notify_all();
//     rclcpp::shutdown();
// }

void SigHandle(int sig)
{
    flg_exit = true;
    sig_buffer.notify_all();
    rclcpp::shutdown();  // 必须保留！没有它 rclcpp::spin() 永远不返回，进程会被强杀
}


inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}




void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}




double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}



void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}


int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}




PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom_fast_lio2";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    
}


void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "base_imu";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "odom_fast_lio2";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom_fast_lio2";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "odom_fast_lio2";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "odom_fast_lio2";
    odomAftMapped.child_frame_id = "base_imu";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "odom_fast_lio2";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "base_imu";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "odom_fast_lio2";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);   // ✅ 使用函数参数
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

 
        // ================== 🔥【2026.3.25 新增：第三步 - 关键帧参数读取与初始化发布器】开始 ==================
        // 声明关键帧距离阈值参数，如果没有在 yaml 文件中配置，则默认值为 1.0 米
        this->declare_parameter<double>("mapping.keyframe_dist", 1.0);
        
        // 声明关键帧角度阈值参数，如果没有在 yaml 文件中配置，则默认值为 10.0 度
        this->declare_parameter<double>("mapping.keyframe_angle", 10.0);

        // 从 ROS2 参数服务器中获取真实的参数值，并赋给刚才定义的类成员变量 keyframe_dist_threshold
        this->get_parameter("mapping.keyframe_dist", keyframe_dist_threshold);
        
        // 声明一个临时变量，用于接收以“度（Degree）”为单位的角度参数
        double angle_deg; 
        
        // 从 ROS2 参数服务器中获取角度值
        this->get_parameter("mapping.keyframe_angle", angle_deg);
        
        // Eigen 的旋转矩阵计算强制要求使用“弧度（Radian）”，因此这里将获取到的角度值乘以 (PI/180) 转为弧度
        keyframe_angle_threshold = angle_deg * M_PI / 180.0; 
        
        // 在终端打印出一行绿色的 INFO 日志，告知开发者当前生效的关键帧提取阈值是多少
        RCLCPP_INFO(this->get_logger(), "【系统通知】关键帧阈值设置完成: 距离 > %.2f m, 角度 > %.2f deg", keyframe_dist_threshold, angle_deg);

        // 实例化关键帧可视化 Marker 的发布器，话题名称定为 "/keyframes_marker"，消息队列长度设为 10
        pubKeyframes_ = this->create_publisher<visualization_msgs::msg::Marker>("/keyframes_marker", 10);

        // 👇 新增：初始化批量关键帧发布器
        pubKeyframesArray_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/keyframes_marker_array", 10);
        // ================== 🔥【2026.3.25 新增：第三步 - 关键帧参数读取与初始化发布器】结束 ==================

        // ================== 🔥【回环连线可视化：初始化发布器】==================
        
        // 初始化回环连线发布器，话题名 /loop_closure_lines
        // 队列长度设为 10，对于可视化来说已经足够
        pubLoopClosureLines_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_lines", 10);
        
    

         // ================== 🔥【2026.3.26 读取回环检测参数】==================
        this->declare_parameter<bool>("loop_closure.enable", false);
        this->declare_parameter<double>("loop_closure.sc_dist_thres", 0.2);

        this->get_parameter("loop_closure.enable", loop_closure_enable);
        this->get_parameter("loop_closure.sc_dist_thres", sc_dist_thres);
        
        // 把阈值传给 SCManager 
        scManager.SC_DIST_THRES = sc_dist_thres;

        if (loop_closure_enable) {
            RCLCPP_INFO(this->get_logger(), "【系统通知】Scan Context 回环检测模块已开启！阈值: %.2f", sc_dist_thres);
        } else {
            RCLCPP_WARN(this->get_logger(), "【系统通知】回环检测模块未开启。");
        }
        // =======================================================
        // ================== 【KD-Tree 半径回环检测：参数读取】起始 ==================
        this->declare_parameter<double>("loop_closure.radius_search_dist", 10.0);
        this->declare_parameter<double>("loop_closure.radius_search_time_diff", 30.0);
        this->declare_parameter<int>("loop_closure.radius_min_frame_gap", 50);

        this->get_parameter("loop_closure.radius_search_dist", radius_search_dist_);
        this->get_parameter("loop_closure.radius_search_time_diff", radius_search_time_diff_);
        this->get_parameter("loop_closure.radius_min_frame_gap", radius_min_frame_gap_);

        RCLCPP_INFO(this->get_logger(),
            "【系统通知】KD-Tree 半径回环: 搜索半径=%.1fm, 最小时间差=%.0fs, 最小帧间距=%d",
            radius_search_dist_, radius_search_time_diff_, radius_min_frame_gap_);
        // ================== 【KD-Tree 半径回环检测：参数读取】结束 ==================

        // ================== 🔥【2026.3.27 新增：ICP 精匹配参数读取】起始 ==================
        // 下面声明的这些参数最好写进你的 yaml 文件里，这样日后调参不用重新编译
        this->declare_parameter<double>("loop_closure.icp_fitness_score", 0.3); // 声明 fitness score 阈值参数
        this->declare_parameter<int>("loop_closure.icp_max_iterations", 100);    // 声明最大迭代次数参数
        this->declare_parameter<double>("loop_closure.icp_max_corr_dist", 3.0); // 声明最大对应点搜索距离参数

        this->get_parameter("loop_closure.icp_fitness_score", icp_fitness_score_threshold_); // 获取真实的 fitness 阈值
        this->get_parameter("loop_closure.icp_max_iterations", icp_max_iterations_);         // 获取真实的迭代次数
        this->get_parameter("loop_closure.icp_max_corr_dist", icp_max_corr_dist_);           // 获取真实的最大距离
        this->declare_parameter<double>("loop_closure.overlap_ratio_threshold", 0.2);
        this->declare_parameter<double>("loop_closure.overlap_search_radius", 1.0);
        this->get_parameter("loop_closure.overlap_ratio_threshold", overlap_ratio_threshold_);
        this->get_parameter("loop_closure.overlap_search_radius", overlap_search_radius_);

        RCLCPP_INFO(this->get_logger(), 
            "【系统通知】ICP 参数: fitness阈值=%.2f, 最大迭代=%d, 最大搜索距离=%.1fm",
            icp_fitness_score_threshold_, icp_max_iterations_, icp_max_corr_dist_); // 打印日志确认参数加载成功
        // ================== 🔥【2026.3.27 新增：ICP 精匹配参数读取】结束 ==================

        // ================== 【scan-to-submap：参数读取】起始 ==================
        this->declare_parameter<int>("loop_closure.submap_search_num", 12);
        this->declare_parameter<double>("loop_closure.submap_voxel_size", 0.2);
        this->get_parameter("loop_closure.submap_search_num", submap_search_num_);
        this->get_parameter("loop_closure.submap_voxel_size", submap_voxel_size_);
        RCLCPP_INFO(this->get_logger(),
            "【系统通知】Submap: 前后各 %d 帧, 体素 %.2fm",
            submap_search_num_, submap_voxel_size_);
        // ================== 【scan-to-submap：参数读取】结束 ==================

        // ================== 🔥【2026.3.30 新增：参数】起始 ==================
        // ================== 🔥 g2o 图优化参数 ==================
        this->declare_parameter<int>("loop_closure.pgo_max_iterations", 30);
        this->get_parameter("loop_closure.pgo_max_iterations", pgo_max_iterations_);
        RCLCPP_INFO(this->get_logger(), "【系统通知】PGO 参数: 最大优化迭代=%d", pgo_max_iterations_);
        // ================== 🔥【2026.3.30 新增：参数】结束 ==================






        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id ="odom_fast_lio2";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        // memset(point_selected_surf, true, sizeof(point_selected_surf));
        // memset(res_last, -1000.0f, sizeof(res_last));
        // downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        // downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        // memset(point_selected_surf, true, sizeof(point_selected_surf));

        // ================== 🔥【2026.3.30 修改：修复原版 memset 对 float 数组的错误用法】起始 ==================
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        std::fill(res_last, res_last + 100000, -1000.0f); // memset 按字节填充，对 float 无效，改用 std::fill
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        // 0.15m 是甜点值：比前端的 0.5m 密 37 倍，但比原始点云省 95% 内存
        downSizeFilterKeyFrame.setLeafSize(0.15f, 0.15f, 0.15f);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        // 注意：原版代码在这里重复执行了一次 memset，属于冗余，已删除
        // ================== 🔥【2026.3.30 修改：修复原版 memset 对 float 数组的错误用法】结束 ==================




        // memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;





        //2026.3.25 - 新增：一个改动
            /*是之前在代码编写、或者从 ROS1 移植到 ROS2 时，因为“复制粘贴”没有删干净而留下的垃圾代码。
                🤔 为什么回调没有被执行两次？

                在 C++ 和 ROS2 的底层逻辑中，sub_pcl_livox_、sub_pcl_pc_ 和 sub_imu_ 都是智能指针（SharedPtr）。
                当程序执行到第二套订阅代码时，它会重新实例化一个新的订阅器，并覆盖掉原来的指针。在这个覆盖的瞬间，第一套订阅器因为没有指针指向它了，就被 C++ 的内存管理机制直接销毁了。

                所以，虽然它不会导致回调函数被执行两次，但这依然是极度不规范的写法，不仅白白浪费了初始化时的系统资源，还会让看代码的人感到困惑。
                ✂️ 应该删除哪一套？留哪一套？

                结论：果断删掉第一套，保留第二套！

                第一套（使用了类似 10, 20 这种简单的整数队列长度）是相对老旧的、偏向 ROS1 风格的简写。

                第二套（使用了 rclcpp::QoS(rclcpp::KeepLast(20)).reliable()）是极其标准且推荐的 ROS2 写法。它显式地指定了 QoS（服务质量）的策略为 reliable（可靠传输），这对于 LiDAR 和 IMU 这种高频、不容丢失的关键传感器数据来说至关重要。
            */

        /*** ROS subscribe initialization ***/
        // if (p_pre->lidar_type == AVIA)
        // {
        //     sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        // }
        // else
        //  {
        //    sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        //  }
        // sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        
        // if (p_pre->lidar_type == AVIA)
        // {
        //      sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(), livox_pcl_cbk);
        // }
        //  else
        //  {
        //     sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(), standard_pcl_cbk);
        // }
        // sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(rclcpp::KeepLast(100)).reliable(), imu_cbk);





        // ================== 🔥 ROS2 传感器话题订阅初始化 ==================
        if (p_pre->lidar_type == AVIA)
        {
            // 针对 Livox 雷达的自定义消息格式，使用可靠的 QoS 策略，队列长度 20
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(), livox_pcl_cbk);
        }
        else
        {
            // 针对常规机械/固态雷达的标准 PointCloud2 消息格式
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(), standard_pcl_cbk);
        }
        
        // 订阅 IMU 数据，因为 IMU 频率极高（通常 200Hz+），所以队列长度设为 100，确保不丢帧
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(rclcpp::KeepLast(100)).reliable(), imu_cbk);



        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");

        // ================== 🔥【Patchwork++ 地面分割：参数与初始化】起始 ==================
        this->declare_parameter<bool>("ground_segmentation.enable", false);
        this->declare_parameter<double>("ground_segmentation.sensor_height", 1.0);
        this->declare_parameter<double>("ground_segmentation.max_range", 60.0);
        this->declare_parameter<double>("ground_segmentation.min_range", 1.0);
        this->declare_parameter<double>("ground_segmentation.th_dist", 0.15);

        this->get_parameter("ground_segmentation.enable", ground_seg_enable_);

        if (ground_seg_enable_)
        {
            // 使用真实 API 的参数结构
            patchwork::Params pw_params;

            // 用户可调的关键参数
            this->get_parameter("ground_segmentation.sensor_height", pw_params.sensor_height);
            this->get_parameter("ground_segmentation.max_range", pw_params.max_range);
            this->get_parameter("ground_segmentation.min_range", pw_params.min_range);
            this->get_parameter("ground_segmentation.th_dist", pw_params.th_dist);

            // 其他参数使用默认值（来自 Params 构造函数的推荐值）
            pw_params.verbose = false;
            pw_params.enable_RNR = true;        // 反射噪声移除（默认开）
            pw_params.enable_RVPF = true;       // 垂直结构检测（默认开）
            pw_params.enable_TGR = true;        // 时序地面回滚（默认开）
            pw_params.uprightness_thr = 0.707;  // cos(45°)，果园起伏地形可改 0.6

            patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(pw_params);

            RCLCPP_INFO(this->get_logger(),
                "🌱【Patchwork++】地面分割已启用 | 传感器高度: %.2fm | max_range: %.1fm | th_dist: %.3f",
                pw_params.sensor_height, pw_params.max_range, pw_params.th_dist);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "🌱【Patchwork++】地面分割模块未启用");
        }
        // ================== 🔥【Patchwork++ 地面分割：参数与初始化】结束 ==================



        // ================== 🔥【2026.3.26 新增：启动后台回环线程】起始 ==================
        loop_thread_ = std::thread(&LaserMappingNode::loopClosureThread, this);
        RCLCPP_INFO(this->get_logger(), "【系统通知】后台回环线程已安全启动，不会阻塞前端里程计！");
        // ================== 🔥【2026.3.26 新增：启动后台回环线程】结束 ==================

        // ================== 🔥【2026.3.26 新增：启动图优化线程 (线程3)】起始 ==================
        pgo_thread_ = std::thread(&LaserMappingNode::poseGraphOptimizationThread, this);
        RCLCPP_INFO(this->get_logger(), "【系统通知】线程3 (Pose Graph Optimization) 已启动，等待接收回环任务！");
        // ================== 🔥【2026.3.26 新增：启动图优化线程 (线程3)】结束 ==================
    }

    ~LaserMappingNode()
    {
        // ================== 🔥【2026.3.26 修改：完美防死锁的全局线程退出机制】起始 ==================
        
        // 1. 将运行标志位改为 false，告诉所有后台线程“系统要退出了”
        loop_thread_running_ = false; 
        
        // 2. 唤醒所有正在沉睡的条件变量，确保它们从 wait() 状态惊醒并检测到 loop_thread_running_ == false
        cv_keyframes_.notify_all();  // 唤醒线程2 (回环检测)
        cv_loop_edges_.notify_all(); // 唤醒线程3 (图优化)
        
        // 3. 安全等待两个后台线程的生命周期彻底终结，防止段错误
        if (loop_thread_.joinable()) {
            loop_thread_.join(); 
        }
        if (pgo_thread_.joinable()) {
            pgo_thread_.join(); 
        }

        if (!keyframes.empty()) {
            RCLCPP_INFO(this->get_logger(), "📦 节点退出，正在保存 PGO 优化地图和轨迹...");
            save_corrected_map();
            save_corrected_trajectory();
            save_corrected_frontend_map();
        }
        
        // ================== 🔥【2026.3.26 修改：完美防死锁的全局线程退出机制】结束 ==================

        fout_out.close();
        fout_pre.close();
        if (fp != nullptr) {
            fclose(fp);
         }
    }

private:


   // ================== 🔥【2026.3.26 修改：独立的后台回环检测线程 (条件变量 0 CPU 消耗版)】起始 ==================
    void loopClosureThread()
    {
        // 废弃了 rclcpp::Rate(1.0); 这种低效的固定频率轮询
        
        // 外层循环依赖于原子变量 loop_thread_running_，一旦析构函数将其置为 false，线程就会准备退出
        while (loop_thread_running_) 
        {
            int current_id = -1; // 用来存从篮子里拿出来的关键帧 ID

            // ---- 1. 使用 condition_variable 进入 0 功耗沉睡，直到被唤醒 ----
            {
                // 注意：条件变量配合的必须是 unique_lock，而不是 lock_guard，因为它需要在等待时自动解锁
                std::unique_lock<std::mutex> lock(mtx_keyframes_);
                
                // wait 函数会让当前线程彻底挂起（0% CPU）。
                // 只有当：1. 主线程 notify_one()，且 2. 下面大括号里的条件为 true 时，它才会继续往下走。
                cv_keyframes_.wait(lock, [this]() {
                    // 唤醒的两个合法条件：要么队列里有东西了，要么是析构函数要求强制退出了
                    return !new_keyframes_queue_.empty() || !loop_thread_running_;
                });

                // 如果线程是被析构函数唤醒的（意味着程序要关闭了），立即 break 退出整个 while 循环
                if (!loop_thread_running_) break;

                // 如果没在 yaml 开启回环，但队列被塞了东西，就把队列清空，然后 continue 重新进入休眠
                if (!loop_closure_enable) {
                    while (!new_keyframes_queue_.empty()) new_keyframes_queue_.pop();
                    continue; 
                }

                // 取出最前面的 ID，并把它从篮子里丢掉
                current_id = new_keyframes_queue_.front();
                new_keyframes_queue_.pop();
                
            } // <--- lock 作用域结束，由于刚才弹出了队列，锁已经不需要了，安全释放，让前台畅通无阻

            // ---- 2. 加锁，从历史数据库中取出这个关键帧的数据 ----
            KeyFrame current_kf;
            {
                std::lock_guard<std::mutex> lock(mtx_keyframes_);
                current_kf = keyframes[current_id];
            }

           // ---- 3. 提取 Scan Context 存入数据库 ----
            // 完美规避：因为只有这一个后台线程在 Pop 和 Save，所以 SC 内部的索引绝对与 keyframes 一一对应！
            // pcl::PointCloud<pcl::PointXYZI> cloud_for_sc;
            // pcl::copyPointCloud(*current_kf.cloud, cloud_for_sc);
            // scManager.makeAndSaveScancontextAndKeys(cloud_for_sc);


            // ================== 🔥【2026.3.30 修改：SC 输入点云重力对齐，适配山地果园等起伏地形】起始 ==================
            // 问题：body 系的 z 轴随车体倾斜，在坡地上 SC 的环结构（按高度分层）会被打乱
            // 修复：用关键帧的世界旋转矩阵将点云旋转到重力对齐系（只旋转不平移，保持原点在传感器处）
            // SC 本身对 yaw 具有旋转不变性（ring-key + 列平移匹配），所以施加完整旋转不影响 yaw 匹配
            pcl::PointCloud<pcl::PointXYZI> cloud_for_sc;
            pcl::copyPointCloud(*current_kf.cloud, cloud_for_sc);

            Eigen::Affine3d gravity_align = Eigen::Affine3d::Identity();
            gravity_align.linear() = current_kf.rotation; // 只取旋转，不取平移
            pcl::transformPointCloud(cloud_for_sc, cloud_for_sc, gravity_align.matrix());

            scManager.makeAndSaveScancontextAndKeys(cloud_for_sc);
            // ================== 🔥【2026.3.30 修改：SC 输入点云重力对齐，适配山地果园等起伏地形】结束 ==================


            // ================== 🔥【2026.3.28 修改：增加 SC 索引同步安全校验】起始 ==================
            // 运行时强制安全校验：确保 SC 内部数据库大小等于当前帧的逻辑总数量 (current_id + 1)
            // ⚠️ 注意：如果你的 SCManager 代码中 polarcontexts_ 是 private 变量导致编译报错，
            // 请去 Scancontext.h 里把它移到 public 区，或者在这里暂时注释掉这一行。
            assert(scManager.polarcontexts_.size() == (size_t)(current_id + 1) 
                   && "严重错误: Scan Context 索引与 keyframes 数据库脱节！");
            // ================== 🔥【2026.3.28 修改：增加 SC 索引同步安全校验】结束 ==================


           // ================== 【双通道回环检测：SC + KD-Tree 半径搜索】起始 ==================

            // 用一个 lambda 统一提交候选到线程3，避免重复代码
            auto submitLoopCandidate = [&](int cur_id, int hist_id, double sc_dist, float yaw_diff, const std::string& source)
            {
                RCLCPP_WARN(this->get_logger(),
                    "🔥[%s] 发现闭环候选: 帧 %d <-> 帧 %d",
                    source.c_str(), cur_id, hist_id);
                {
                    std::lock_guard<std::mutex> lock(mtx_loop_edges_);
                    LoopCandidate candidate;
                    candidate.current_id = cur_id;
                    candidate.loop_id    = hist_id;
                    candidate.sc_distance = sc_dist;
                    candidate.yaw_diff_rad = yaw_diff;
                    candidate.source     = source;  // ← 传递来源标识
                    loop_edges_queue_.push(candidate);
                }
                cv_loop_edges_.notify_one();
            };

            // ---- 通道 1：Scan Context 描述子匹配（擅长大回环）----
            std::pair<int, float> detectResult = scManager.detectLoopClosureID();
            int sc_loop_id = detectResult.first;
            float yaw_diff_rad = detectResult.second;

            // ================== 【SC 去重：同一个历史帧只提交一次】起始 ==================
            if (sc_loop_id != -1
                && std::abs(current_id - sc_loop_id) >= radius_min_frame_gap_
                && radius_loop_submitted_.find(sc_loop_id) == radius_loop_submitted_.end())
            {
                radius_loop_submitted_.insert(sc_loop_id);
                submitLoopCandidate(current_id, sc_loop_id,
                    scManager.sc_last_distance_, yaw_diff_rad, "SC");
            }
            // ================== 【SC 去重：同一个历史帧只提交一次】结束 ==================
            



            // ---- 通道 2：KD-Tree 半径搜索（擅长近距离回程）----
            // 原理：用当前帧的 3D 位置，在所有历史关键帧中搜索距离 < radius_search_dist_ 的帧
            // 然后筛选出时间间隔足够大的（排除时序相邻帧），作为回环候选提交给线程3
            {
                // 加锁拷贝必要数据
                Eigen::Vector3d cur_pos_copy;
                struct RadiusCandidate { int id; Eigen::Vector3d pos; double time; };
                std::vector<RadiusCandidate> all_kf_snapshot;
                double cur_time_copy;
                {
                    std::lock_guard<std::mutex> lock(mtx_keyframes_);
                    if (current_id < (int)keyframes.size()) {
                        cur_pos_copy = keyframes[current_id].position;
                        cur_time_copy = keyframes[current_id].time;
                    }
                    all_kf_snapshot.reserve(keyframes.size());
                    for (int i = 0; i < (int)keyframes.size(); i++) {
                        all_kf_snapshot.push_back({i, keyframes[i].position, keyframes[i].time});
                    }
                } // 解锁

                int radius_count_this_frame = 0;

                // 遍历所有历史帧，找距离近且时间远的
                for (const auto& kf_snap : all_kf_snapshot)
                {
                    // 跳过帧间距不够的
                    if (std::abs(current_id - kf_snap.id) < radius_min_frame_gap_)
                        continue;

                    // 跳过时间间隔不够的
                    if (std::abs(cur_time_copy - kf_snap.time) < radius_search_time_diff_)
                        continue;

                    // 计算欧氏距离
                    double dist = (cur_pos_copy - kf_snap.pos).norm();
                    if (dist > radius_search_dist_)
                        continue;

                    // 跳过已经提交过的（防止同一个历史帧被反复提交）
                    if (radius_loop_submitted_.count(kf_snap.id))
                        continue;

                    // 跳过 SC 已经找到的同一帧（避免重复）
                    if (kf_snap.id == sc_loop_id)
                        continue;

                    // ================== 🔥【走廊防护：行驶距离比过滤】起始 ==================
                    // 真回环：走了很远又回到原点，travel_ratio >> 1（通常 > 3）
                    // 走廊同侧假回环：一直在往前走，travel_ratio ≈ 1~1.5
                    {
                        double odom_travel = 0.0;
                        int id_lo = std::min(current_id, kf_snap.id);
                        int id_hi = std::max(current_id, kf_snap.id);
                        // 用 snapshot 里的 pos 逐帧累加轨迹弧长
                        for (int ti = id_lo; ti < id_hi && (ti + 1) < (int)all_kf_snapshot.size(); ti++) {
                            odom_travel += (all_kf_snapshot[ti + 1].pos - all_kf_snapshot[ti].pos).norm();
                        }
                        double euclidean_dist = (cur_pos_copy - kf_snap.pos).norm() + 1e-3;
                        double travel_ratio = odom_travel / euclidean_dist;

                        if (travel_ratio < 3.0) {
                            // 没走回头路 → 大概率是走廊同侧，跳过
                            continue;
                        }
                    }
                    // ================== 🔥【走廊防护：行驶距离比过滤】结束 ==================

                    // ====== 🔥【新增：回环帧间隔节流】======
                    // 新关键帧距离上次成功提交的回环 < 3 帧时，跳过本次提交
                    // 避免短时间内大量相似回环把端点焊死
                    static int last_loop_submit_id = -100;
                    if (current_id - last_loop_submit_id < 3) {
                        continue;  // 跳过本次候选
                    }
                    // 注意：这里只是先跳过，下面 submitLoopCandidate 成功后再更新 last_loop_submit_id
                    // ====== 🔥【新增】结束 ======

                    // 找到了！记录并提交
                    radius_loop_submitted_.insert(kf_snap.id);
                    last_loop_submit_id = current_id;   // ← 更新
                    submitLoopCandidate(current_id, kf_snap.id,
                        0.1, 0.0f, "Radius");  // sc_distance 给一个合理默认值
                        // ================== 🔥【限流修复：每帧最多 1 个半径候选，防止雪崩】起始 ==================
                    // 原来每帧最多 3 个候选，导致同一个新关键帧同时匹配历史多帧
                    // → 触发多次 PGO → 累积校正量叠加 → 轨迹被拉飞
                    // 改为每帧最多 1 个，强制串行处理，让系统有"喘息"时间
                    radius_count_this_frame++;
                    if (radius_count_this_frame >= 1) {
                        break;
                    }
                    // ================== 🔥【限流修复：每帧最多 1 个半径候选，防止雪崩】结束 ==================

     
                }
            }

            // ================== 【双通道回环检测：SC + KD-Tree 半径搜索】结束 ==================ccc
        }
    }
    // ================== 🔥【2026.3.26 修改：独立的后台回环检测线程 (条件变量 0 CPU 消耗版)】结束 ==================


    // ================== 【几何一致性检查：拒绝交叉回环】起始 ==================
    bool isConsistentWithHistory(int new_from, int new_to)
    {
        std::lock_guard<std::mutex> lock(mtx_loop_pairs_);
        for (const auto& lp : loop_closure_pairs_)
        {
            int diff_from = new_from - lp.from_id;
            int diff_to   = new_to   - lp.to_id;
            if ((diff_from > 3 && diff_to < -3) || (diff_from < -3 && diff_to > 3))
            {
                RCLCPP_WARN(this->get_logger(),
                    "🚫 拒绝交叉！新边(%d→%d) 与已有边(%d→%d) 矛盾",
                    new_from, new_to, lp.from_id, lp.to_id);
                return false;
            }
        }
        return true;
    }
    // ================== 【几何一致性检查：拒绝交叉回环】结束 ==================



    // ================== 🔥【2026.3.26 新增：独立的图优化与ICP线程 (线程3)】起始 ==================
    void poseGraphOptimizationThread()
    {
        // 只要全局运行标志为 true，线程3就会一直在后台待命
        while (loop_thread_running_) 
        {
            // ================== 🔥【2026.3.31 修改：接收 LoopCandidate 结构体】起始 ==================
            LoopCandidate loop_candidate;

            // ---- 1. 使用 condition_variable 沉睡，等待线程2投递回环任务 ----
            {
                std::unique_lock<std::mutex> lock(mtx_loop_edges_);
                cv_loop_edges_.wait(lock, [this]() {
                    return !loop_edges_queue_.empty() || !loop_thread_running_;
                });
                if (!loop_thread_running_) break;
                loop_candidate = loop_edges_queue_.front();
                loop_edges_queue_.pop();
            }

            int current_id = loop_candidate.current_id;
            int loop_id = loop_candidate.loop_id;
            double sc_distance = loop_candidate.sc_distance;
            // float yaw_diff_rad = loop_candidate.yaw_diff_rad; // 预留给未来 yaw 补偿用
            // ================== 🔥【2026.3.31 修改：接收 LoopCandidate 结构体】结束 ==================


            RCLCPP_INFO(this->get_logger(), "🟢 [线程3 - 图优化] 被唤醒！开始处理回环边: 帧 %d <----> 帧 %d", current_id, loop_id);

           // ================== 【scan-to-submap：点云准备 + GICP + 校验】起始 ==================

            // ---- 2. 加锁，一次性拷贝当前帧 + 历史帧 + 邻居帧数据 ----
            KeyFrame current_kf_copy;
            KeyFrame history_kf_copy;
            struct NeighborSnap { 
                Eigen::Vector3d pos; Eigen::Matrix3d rot; 
                Eigen::Vector3d odom_pos; Eigen::Matrix3d odom_rot;  // P0修复2: 纯净odom位姿
                PointCloudXYZI::Ptr cloud; 
            };
            std::vector<NeighborSnap> neighbors;
            
            // ================== 🔥【粗到精GICP + 自适应submap】起始 ==================
            // 根据odom距离自适应submap范围和配准策略，解决大回环时init_guess差导致的配准失败
            bool is_large_loop = false;
            int actual_submap_num = submap_search_num_;
            float actual_voxel = submap_voxel_size_;
            double odom_dist = 0.0;
            
            {
                std::lock_guard<std::mutex> lock(mtx_keyframes_);
                current_kf_copy = keyframes[current_id];
                history_kf_copy = keyframes[loop_id];
                
                odom_dist = (current_kf_copy.odom_position - history_kf_copy.odom_position).norm();
                is_large_loop = (odom_dist > 15.0) || (loop_candidate.source == "SC");
                
                if (is_large_loop) {
                    // 大回环：submap范围扩大到至少能覆盖odom距离的一半，上限50帧
                    int expanded_num = std::max(submap_search_num_, std::min((int)(odom_dist * 0.5), 50));
                    actual_submap_num = expanded_num;
                    actual_voxel = 0.25f; // 大回环也用较细体素，保证GICP能看见真实偏移
                    RCLCPP_INFO(this->get_logger(),
                        "🗺️ [大回环] 扩大submap: %d → %d 帧 | odom距离=%.1fm | 来源=%s",
                        submap_search_num_, actual_submap_num, odom_dist, loop_candidate.source.c_str());
                }
                
                int kf_sz = (int)keyframes.size();
                int s = std::max(0, loop_id - actual_submap_num);
                int e = std::min(kf_sz - 1, loop_id + actual_submap_num);
                neighbors.reserve(e - s + 1);
                for (int i = s; i <= e; i++)
                    neighbors.push_back({keyframes[i].position, keyframes[i].rotation, 
                                                    keyframes[i].odom_position, keyframes[i].odom_rotation, 
                                                    keyframes[i].cloud});
            }

            // ================== 🔥【P0修复2：submap/GICP全用odom位姿，与g2o顶点同系】起始 ==================
            // ---- 3. history odom位姿的逆（submap 坐标原点）----
            Eigen::Affine3d T_history_inv = Eigen::Affine3d::Identity();
            T_history_inv.linear()      = history_kf_copy.odom_rotation.transpose();
            T_history_inv.translation() = -(history_kf_copy.odom_rotation.transpose() * history_kf_copy.odom_position);

            // ---- 4. 构建 submap：邻居帧 → history 局部系 → 合并 → 降采样 ----
            PointCloudXYZI::Ptr submap_raw(new PointCloudXYZI());
            for (const auto& nb : neighbors) {
                Eigen::Affine3d T_nb = Eigen::Affine3d::Identity();
                T_nb.linear() = nb.odom_rot; T_nb.translation() = nb.odom_pos;  // ← odom位姿
                PointCloudXYZI::Ptr tmp(new PointCloudXYZI());
                pcl::transformPointCloud(*nb.cloud, *tmp, (T_history_inv * T_nb).matrix());
                *submap_raw += *tmp;
            }
            PointCloudXYZI::Ptr submap_ds(new PointCloudXYZI());
            pcl::VoxelGrid<PointType> vg;
            vg.setLeafSize(actual_voxel, actual_voxel, actual_voxel);
            vg.setInputCloud(submap_raw);
            vg.filter(*submap_ds);

            RCLCPP_INFO(this->get_logger(),
                "🗺️ [线程3] Submap(odom系): %zu 帧 | %zu → %zu 点 | 体素=%.2f",
                neighbors.size(), submap_raw->points.size(), submap_ds->points.size(), actual_voxel);

            // ---- 5. 当前帧(odom系) → history odom局部系 ----
            Eigen::Isometry3d T_cur_odom = Eigen::Isometry3d::Identity();
            T_cur_odom.linear() = current_kf_copy.odom_rotation;
            T_cur_odom.translation() = current_kf_copy.odom_position;
            Eigen::Affine3d init_guess = T_history_inv * T_cur_odom;
            // ================== 🔥【P0修复2】结束 ==================

            PointCloudXYZI::Ptr cloud_cur_body(new PointCloudXYZI());
            pcl::copyPointCloud(*current_kf_copy.cloud, *cloud_cur_body);
            PointCloudXYZI::Ptr cloud_cur_in_hist(new PointCloudXYZI());
            pcl::transformPointCloud(*cloud_cur_body, *cloud_cur_in_hist, init_guess.matrix());

            // ---- 6. 粗到精GICP：scan-to-submap ----
            Eigen::Matrix4f final_tf = Eigen::Matrix4f::Identity();
            bool coarse_converged = false;
            
            // 【 coarse 阶段】大回环时用大搜索距离快速粗配准
            if (is_large_loop) {
                pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_coarse;
                gicp_coarse.setMaximumIterations(40);
                gicp_coarse.setTransformationEpsilon(1e-4);
                gicp_coarse.setEuclideanFitnessEpsilon(1e-3);
                float coarse_dist = std::max((float)icp_max_corr_dist_, std::min((float)(odom_dist * 0.5f), 35.0f));
                gicp_coarse.setMaxCorrespondenceDistance(coarse_dist);
                gicp_coarse.setRotationEpsilon(5e-2);
                gicp_coarse.setCorrespondenceRandomness(40);
                gicp_coarse.setInputSource(cloud_cur_in_hist);
                gicp_coarse.setInputTarget(submap_ds);

                PointCloudXYZI::Ptr aligned_coarse(new PointCloudXYZI());
                auto coarse_start = std::chrono::high_resolution_clock::now();
                gicp_coarse.align(*aligned_coarse);
                auto coarse_end = std::chrono::high_resolution_clock::now();
                double coarse_ms = std::chrono::duration<double, std::milli>(coarse_end - coarse_start).count();
                
                if (gicp_coarse.hasConverged() && gicp_coarse.getFitnessScore() < 3.0) {
                    Eigen::Matrix4f coarse_tf = gicp_coarse.getFinalTransformation();
                    float coarse_corr = Eigen::Vector3f(coarse_tf(0,3), coarse_tf(1,3), coarse_tf(2,3)).norm();
                    if (coarse_corr > 0.05f) {
                        final_tf = coarse_tf;
                        coarse_converged = true;
                        pcl::transformPointCloud(*cloud_cur_in_hist, *cloud_cur_in_hist, final_tf);
                        RCLCPP_INFO(this->get_logger(),
                            "🎯 [粗GICP] 收敛 | MSE=%.3f | 修正=%.2fm | 耗时=%.1fms | 搜索=%.1fm",
                            gicp_coarse.getFitnessScore(), coarse_corr, coarse_ms, coarse_dist);
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                            "⚠️ [粗GICP] 收敛但修正≈0 (%.2fm)，丢弃coarse结果，fine将从纯净odom init_guess开始",
                            coarse_corr);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(),
                        "⚠️ [粗GICP] 未收敛或质量差 | 收敛=%s | MSE=%.3f",
                        gicp_coarse.hasConverged() ? "是" : "否", gicp_coarse.getFitnessScore());
                }
            }

            // 【 fine 阶段】精配准，用 coarse 结果或原始 init_guess 作为起点
            pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
            gicp.setMaximumIterations(80);
            gicp.setTransformationEpsilon(1e-6);
            gicp.setEuclideanFitnessEpsilon(1e-6);
            gicp.setMaxCorrespondenceDistance(icp_max_corr_dist_);
            gicp.setRotationEpsilon(1e-3);
            gicp.setCorrespondenceRandomness(20);
            gicp.setInputSource(cloud_cur_in_hist);  // 若 coarse 成功，已被 coarse_tf 变换过
            gicp.setInputTarget(submap_ds);

            PointCloudXYZI::Ptr aligned_cloud(new PointCloudXYZI());
            auto icp_start = std::chrono::high_resolution_clock::now();
            gicp.align(*aligned_cloud);
            auto icp_end = std::chrono::high_resolution_clock::now();
            double icp_time_ms = std::chrono::duration<double, std::milli>(icp_end - icp_start).count();

            if (coarse_converged) {
                final_tf = gicp.getFinalTransformation() * final_tf;
            } else {
                final_tf = gicp.getFinalTransformation();
            }
            // ================== 🔥【粗到精GICP + 自适应submap】结束 ==================

            // ---- 7. 校验：收敛 + 重叠率 + fitness + 修正量 ----
            double fitness_score = gicp.getFitnessScore();
            bool converged = gicp.hasConverged();

            double overlap_ratio = 0.0;
            if (converged && aligned_cloud->size() > 0) {
                pcl::KdTreeFLANN<PointType> tree;
                tree.setInputCloud(submap_ds);
                int inliers = 0;
                std::vector<int> idx(1);
                std::vector<float> dist(1);
                double r2 = overlap_search_radius_ * overlap_search_radius_;
                for (const auto& pt : aligned_cloud->points) {
                    tree.nearestKSearch(pt, 1, idx, dist);
                    if (dist[0] < r2) inliers++;
                }
                overlap_ratio = (double)inliers / (double)aligned_cloud->size();
            }

            // 【修正量统计】 coarse + fine 的总修正
            double total_corr_trans = Eigen::Vector3f(final_tf(0,3), final_tf(1,3), final_tf(2,3)).norm();
            double total_corr_rot_deg = std::acos(std::min(1.0,
                std::max(-1.0, (double)(final_tf.block<3,3>(0,0).trace() - 1.0) / 2.0))) * 180.0 / M_PI;

            // ================== 🔥【走廊防护：Submap 退化检测】起始 ==================
            // 对 submap 点云做 PCA，如果最小/最大特征值之比很小，
            // 说明场景沿某方向极度退化（走廊/隧道），GICP 在该方向的解不可信
            bool is_degenerate = false;
            float degeneracy_ratio = 1.0f;
            {
                Eigen::Vector4f centroid;
                Eigen::Matrix3f covariance;
                pcl::computeMeanAndCovarianceMatrix(*submap_ds, covariance, centroid);

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
                Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues(); // 升序：λ0 ≤ λ1 ≤ λ2

                degeneracy_ratio = eigenvalues(0) / (eigenvalues(2) + 1e-6f);

                // 典型走廊：一个方向极度拉长 → ratio < 0.05
                if (degeneracy_ratio < 0.015f) {
                    is_degenerate = true;
                    RCLCPP_WARN(this->get_logger(),
                        "⚠️ [退化检测] 帧 %d <-> %d 场景退化！特征值比=%.4f (%.1f / %.1f / %.1f)",
                        current_id, loop_id, degeneracy_ratio,
                        eigenvalues(0), eigenvalues(1), eigenvalues(2));
                }
            }
            // ================== 🔥【走廊防护：Submap 退化检测】结束 ==================

            // ================== 🔥【走廊防护：分级校验门槛】起始 ==================
           bool icp_success = false;
           if (is_degenerate) {
                // 极度退化但 fitness 极优 + 修正极小，说明本来就在原地，安全接受
                icp_success = converged
                    && fitness_score < 0.10        // 必须近乎完美匹配
                    && overlap_ratio > 0.95        // 95%+ 重叠
                    && total_corr_trans < 0.3            // 几乎不动
                    && total_corr_rot_deg < 2.0;         // 几乎不转
            } else {
               // 高质量回环允许大修正（fitness 和 overlap 已经把关）
                bool quality_excellent = (fitness_score < 0.50) && (overlap_ratio > 0.85);
                double corr_trans_limit = quality_excellent ? 15.0 : 2.5;
                double corr_rot_limit   = quality_excellent ? 45.0 : 10.0;
                
                icp_success = converged
                    && (quality_excellent || fitness_score < icp_fitness_score_threshold_)
                    && overlap_ratio > overlap_ratio_threshold_
                    && total_corr_trans < corr_trans_limit
                    && total_corr_rot_deg < corr_rot_limit;
            }
            // ================== 🔥【走廊防护：分级校验门槛】结束 ==================

            




            // ---- 8. 诊断记录 ----
            {
                LoopEdge edge;
                edge.from_id = current_id;
                edge.to_id = loop_id;
                edge.fitness_score = fitness_score;
                edge.icp_time_ms = icp_time_ms;
                edge.accepted = icp_success;
                if (icp_success) {
                    Eigen::Affine3d corr;
                    corr.matrix() = final_tf.cast<double>();
                    edge.relative_pose = corr * init_guess;
                } else {
                    edge.relative_pose = Eigen::Affine3d::Identity();
                }
                std::lock_guard<std::mutex> lock(mtx_loop_edges_history_);
                loop_edges_history_.push_back(edge);
            }

            // ---- 9. 失败跳过 ----
            if (!icp_success) {
                RCLCPP_WARN(this->get_logger(),
                    "❌ [线程3] 校验未通过！帧 %d <-> %d | 退化:%s(%.4f) | 收敛:%s | fitness:%.4f(阈值:%.2f) | "
                    "重叠率:%.1f%%(阈值:%.0f%%) | 修正:%.2fm/%.1fdeg | 耗时:%.1fms",
                    current_id, loop_id,
                    is_degenerate ? "是" : "否", degeneracy_ratio,
                    converged ? "是" : "否",
                    fitness_score, is_degenerate ? 0.30 : icp_fitness_score_threshold_,
                    overlap_ratio * 100.0, is_degenerate ? 50.0 : overlap_ratio_threshold_ * 100.0,
                    total_corr_trans, total_corr_rot_deg, icp_time_ms);
                continue;
            }

            // ================== 【scan-to-submap：点云准备 + GICP + 校验】结束 ==================

            // --- 7. 提取最终的回环相对位姿约束 ---
            Eigen::Affine3d correction; // 声明修正量仿射矩阵
            correction.matrix() = final_tf.cast<double>(); // 提取 GICP 计算出的总修正矩阵
            Eigen::Affine3d T_history_to_current = correction * init_guess; // 从 Target(历史) 到 Source(当前) 的绝对相对约束

            Eigen::Vector3d loop_translation = T_history_to_current.translation();
            Eigen::Matrix3d loop_rotation = T_history_to_current.linear();


            
          

            // --- 8. 从 fitness score 自适应计算信息矩阵权重 ---
            // ================== 🔥【质量严惩版：MSE 大则权重暴跌 + 距离感知】起始 ==================
            double score = gicp.getFitnessScore();
            double time_gap = std::abs(current_kf_copy.time - history_kf_copy.time);
            double spatial_dist = (current_kf_copy.position - history_kf_copy.position).norm();

              // ====== 🔥【新增：近距离回环衰减】======
            // 距离 < 3m 的"近距离回环"质量虽好，但容易在端点扎堆
            // 短距离回环权重大幅衰减，避免端点被多条边焊死
            double distance_attenuation = 1.0;
            if (spatial_dist < 3.0) {
                distance_attenuation = 0.3;   // 近回环 → 30% 权重
            } else if (spatial_dist < 5.0) {
                distance_attenuation = 0.6;   // 中近回环 → 60% 权重
            }
            // ====== 🔥【新增：近距离回环衰减】结束 ======

            // 【1. 基础置信度：MSE 越大权重越小，且对大 MSE 做重罚（二次方惩罚）】
            // 对比：MSE=0.1 → weight_base=~100；MSE=1 → ~1；MSE=3 → ~0.11（几乎无影响）
            // 这样"勉强对齐"的回环在 g2o 里权重自然很小，不会霸道拉拽
            double base_information = 1.0 / (score * score + 0.01);

            // 【2. 质量硬门槛】：MSE > 1.5 的回环直接降权到 0.1 倍（果园走廊 GICP 凑合解）
            double quality_penalty = 1.0;
            if (score > 1.5) {
                quality_penalty = 0.1;
            } else if (score > 0.8) {
                quality_penalty = 0.3;
            }

            // 【3. 长时间回环奖励，但封顶 1.5 倍（原来 2.0 太激进）】
            double value_bonus = 1.0;
            if (time_gap > 60.0) {
                value_bonus = std::min(1.0 + (time_gap - 60.0) / 300.0, 1.5);
            }

            // 【4. 组装 g2o 6x6 信息矩阵】
            Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Zero(6, 6);

            // ================== 🔥【P0修复5：回环权重与里程计边脱钩，优秀回环主导优化】起始 ==================
            // 问题：里程计边 100 × 300 条 = 总刚度 30000，回环边 1000 无法拉动长链
            // 修复：优秀回环权重提升到 5000~10000，让单条回环边就能对抗整条里程计链
            double loop_multiplier = 10.0;
            if (score < 0.3 && overlap_ratio > 0.85) {
                loop_multiplier = 50.0;   // 优秀回环：5000+ 权重
            } else if (score < 0.6) {
                loop_multiplier = 20.0;  // 良好回环：2000+ 权重
            }
            double weight_trans = base_information * value_bonus * quality_penalty * distance_attenuation * loop_multiplier;
            weight_trans = std::min(weight_trans, 10000.0); // 上限防止数值爆炸
            information_matrix(0, 0) = weight_trans;
            information_matrix(1, 1) = weight_trans;
            information_matrix(2, 2) = weight_trans;

            double weight_rot = weight_trans * 2.0;
            information_matrix(3, 3) = weight_rot;
            information_matrix(4, 4) = weight_rot;
            information_matrix(5, 5) = weight_rot;
            // ================== 🔥【P0修复5】结束 ==================

            RCLCPP_INFO(this->get_logger(),
                "📊 回环建立: MSE=%.3f | 距离=%.1fm | 时间差=%.1fs | 质量惩罚=%.2f | 奖励=%.2fx | 乘数=%.0f | 平移权=%.1f | 旋转权=%.1f",
                score, spatial_dist, time_gap, quality_penalty, value_bonus, loop_multiplier, weight_trans, weight_rot);
            // ================== 🔥【质量严惩版：MSE 大则权重暴跌 + 距离感知】结束 ==================


            // // ================== 【一致性检查】起始 ==================
            // if (!isConsistentWithHistory(current_id, loop_id))
            // {
            //     RCLCPP_WARN(this->get_logger(),
            //         "🚫 帧 %d <-> %d 通过 ICP 但被一致性检查拒绝", current_id, loop_id);
            //     continue;
            // }
            // // ================== 【一致性检查】结束 ==================

            // ================== 【记录连线 + 质量信息】起始 ==================
            {
                std::lock_guard<std::mutex> lock(mtx_loop_pairs_);
                LoopPairViz viz;
                viz.from_id = current_id;
                viz.to_id = loop_id;
                viz.fitness_score = score;
                viz.corr_trans = total_corr_trans;
                loop_closure_pairs_.push_back(viz);
            }
            // ================== 【记录连线 + 质量信息】结束 ==================
            


            



           
            // ================== 🔥 g2o 位姿图优化 ==================
            
            auto pgo_start = std::chrono::high_resolution_clock::now();

            // --- 1. 加锁，拷贝当前所有关键帧位姿的快照 ---
            std::vector<Eigen::Isometry3d> poses_snapshot;
            int num_keyframes = 0;
            {
                std::lock_guard<std::mutex> lock(mtx_keyframes_);
                num_keyframes = (int)keyframes.size();
                poses_snapshot.reserve(num_keyframes);

                // ================== 🔥【致命修复：顶点初值必须用纯净 odom】起始 ==================
                // 关键原理：g2o 顶点初值、里程计边、回环边必须在同一个坐标系下
                // 之前 bug：顶点初值用 keyframes[i].position（被 PGO 校正过），
                //           里程计边用 odom_position（纯净），
                //           两者坐标系不一致，g2o 会被里程计边"强行拖拽"到纯净 odom 系
                //           导致所有历史 PGO 校正被反向抹除，新一轮校正量巨大
                // 修复：顶点初值用 odom_position/odom_rotation，与里程计边严格同系
                //       回环边 measurement 基于 position（已校正系），与 odom 顶点产生真实残差
                for (int i = 0; i < num_keyframes; i++) {
                    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                    pose.linear() = keyframes[i].odom_rotation;
                    pose.translation() = keyframes[i].odom_position;
                    poses_snapshot.push_back(pose);
                }
                // ================== 🔥【致命修复：顶点初值必须用纯净 odom】结束 ==================

            } // 解锁，后面全部在私有数据上操作

            if (num_keyframes < 3) {
                RCLCPP_WARN(this->get_logger(), "关键帧不足 3 个，跳过图优化");
                continue;
            }

            // --- 2. 构建 g2o 优化器 ---
            auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
            auto blockSolver = std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
            auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

            g2o::SparseOptimizer optimizer;
            optimizer.setAlgorithm(algorithm);
            optimizer.setVerbose(false);

            // --- 3. 添加所有关键帧作为顶点 ---
            for (int i = 0; i < num_keyframes; i++) {
                auto* vertex = new g2o::VertexSE3();
                vertex->setId(i);
                vertex->setEstimate(poses_snapshot[i]);
                if (i == 0) vertex->setFixed(true); // 锚定第一帧
                optimizer.addVertex(vertex);
            }

            // ================== 🔥【2026.3.30 修改：里程计边使用纯净原始位姿，防止 PGO 反馈污染】起始 ==================
            // --- 4. 添加里程计边（用永远不被 PGO 修改的 odom_position/odom_rotation 计算）---
            // ================== 🔥【P0修复6：里程计边降权，释放PGO修正自由度】起始 ==================
            // 问题：odom边 100 × 300 条 = 总刚度 30000，回环边 1000 被淹没，PGO无法大幅修正
            // 修复：odom边降到 50/100，让长链总刚度降低，优秀回环边(5000+)能真正拉动轨迹
            // 短程odom本身很准，即使权重50也能保持局部形状；回环才是修正全局漂移的关键
            Eigen::MatrixXd odom_info = Eigen::MatrixXd::Zero(6, 6);
            odom_info(0,0) = 50.0;
            odom_info(1,1) = 50.0;
            odom_info(2,2) = 50.0;
            odom_info(3,3) = 100.0;
            odom_info(4,4) = 100.0;
            odom_info(5,5) = 100.0;
            // ================== 🔥【P0修复6】结束 ==================
            // ================== 🔥【2026.4.1 修改：科学解耦里程计边权重】结束 ==================
            
            // 先拷贝一份原始里程计位姿快照（加锁在前面已经做过了，这里用 poses_snapshot 同期的数据）
            std::vector<Eigen::Isometry3d> odom_snapshot;
            {
                
                std::lock_guard<std::mutex> lock(mtx_keyframes_);
                odom_snapshot.reserve(num_keyframes);
                for (int i = 0; i < num_keyframes; i++) {
                    Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();
                    odom_pose.linear() = keyframes[i].odom_rotation;       // 保持 odom 相对量
                    odom_pose.translation() = keyframes[i].odom_position;
                    odom_snapshot.push_back(odom_pose);
                }
            
            }

            

            for (int i = 0; i < num_keyframes - 1; i++) {
                // 关键区别：用原始 odom 位姿计算相对变换，而非被 PGO 回写过的 position/rotation
                Eigen::Isometry3d T_relative = odom_snapshot[i].inverse() * odom_snapshot[i + 1];
                
                auto* edge = new g2o::EdgeSE3();
                edge->setVertex(0, optimizer.vertex(i));
                edge->setVertex(1, optimizer.vertex(i + 1));
                edge->setMeasurement(T_relative);
                edge->setInformation(odom_info);
                optimizer.addEdge(edge);
            }
            // ================== 🔥【2026.3.30 修改：里程计边使用纯净原始位姿，防止 PGO 反馈污染】结束 ==================

            // --- 5. 添加本次回环边 ---
            // ================== 🔥【2026.4.1 修改：为新回环边添加鲁棒核函数】起始 ==================
            {
                Eigen::Isometry3d loop_measurement = Eigen::Isometry3d::Identity();
                loop_measurement.linear() = T_history_to_current.linear();
                loop_measurement.translation() = T_history_to_current.translation();

                auto* loop_edge = new g2o::EdgeSE3();
                loop_edge->setVertex(0, optimizer.vertex(loop_id));
                loop_edge->setVertex(1, optimizer.vertex(current_id));
                loop_edge->setMeasurement(loop_measurement);
                loop_edge->setInformation(information_matrix); 

                // ================== 【自适应 Huber 核：让大回环也能拉动】起始 ==================
                // 固定 delta=3 对 44m 大回环会把权重压到 1%，真回环拉不动
                // Huber 比 Cauchy 温和：残差 > delta 时按 δ/|e| 线性衰减（非二次）
                // delta 随两顶点距离变化，保证真回环残差在 delta 以内时完全不衰减
                double adaptive_delta = std::max(2.0, spatial_dist * 0.3);
                auto* rk = new g2o::RobustKernelHuber();
                rk->setDelta(adaptive_delta);
                loop_edge->setRobustKernel(rk);
                RCLCPP_INFO(this->get_logger(),
                    "🛡️ 鲁棒核 Huber delta=%.2f (距离 %.1fm)",
                    adaptive_delta, spatial_dist);
                // ================== 【自适应 Huber 核】结束 ==================

                optimizer.addEdge(loop_edge);

                // ================== 🔬【yaw 重复性诊断】打印新回环边的 yaw 测量值 ==================
                // 同一对(loop_id, current_id)在不同次运行里 GICP 算出的 yaw 应当几乎完全一致
                // 如果差异大，说明这条回环边本身就是"摇摆边"，是 yaw 偏移的根源
                {
                    double meas_yaw_deg = std::atan2(loop_measurement.linear()(1,0),
                                                     loop_measurement.linear()(0,0)) * 180.0 / M_PI;
                    Eigen::Vector3d meas_t = loop_measurement.translation();
                    RCLCPP_WARN(this->get_logger(),
                        "🔬 [回环边-新] LOOP_ID=L%d_to_%d  meas_yaw=%.3f° meas_t=(%.3f,%.3f,%.3f) "
                        "delta=%.2f spatial=%.1fm",
                        loop_id, current_id,
                        meas_yaw_deg, meas_t(0), meas_t(1), meas_t(2),
                        adaptive_delta, spatial_dist);
                }
                // ================== 🔬【yaw 重复性诊断】结束 ==================
            }
            // ================== 🔥【2026.4.1 修改：为新回环边添加鲁棒核函数】结束 ==================




           // --- 6. 添加历史上已经通过校验的回环边（这样图会越来越准） ---
            {
                std::lock_guard<std::mutex> lock(mtx_loop_edges_history_);
                for (const auto& hist_edge : loop_edges_history_) {
                    if (!hist_edge.accepted) continue;
                    // 跳过刚才已经加过的这条
                    if (hist_edge.from_id == current_id && hist_edge.to_id == loop_id) continue;
                    // 防越界
                    if (hist_edge.from_id >= num_keyframes || hist_edge.to_id >= num_keyframes) continue;

                    // ================== 🔥【P0修复3：历史边保留原始GICP测量的drift信息】起始 ==================
                    // 不能重新计算为odom相对位姿——那会抹掉GICP当时发现的drift！
                    // hist_edge.relative_pose 已在odom系下（GICP用odom位姿构建submap），直接复用
                    Eigen::Isometry3d meas = Eigen::Isometry3d::Identity();
                    meas.linear() = hist_edge.relative_pose.linear();
                    meas.translation() = hist_edge.relative_pose.translation();
                    // ================== 🔥【P0修复3】结束 ==================

                    // 权重对齐当前回环边量级，优秀历史边也给予高权重
                    double w_base = 1.0 / (hist_edge.fitness_score * hist_edge.fitness_score + 0.01);
                    double hist_multiplier = 10.0;
                    if (hist_edge.fitness_score < 0.3) {
                        hist_multiplier = 50.0;   // 优秀历史回环保持高权重
                    } else if (hist_edge.fitness_score < 0.6) {
                        hist_multiplier = 20.0;
                    }
                    double w_trans_hist = w_base * hist_multiplier;
                    w_trans_hist = std::min(w_trans_hist, 10000.0);
                    double w_rot_hist   = w_trans_hist * 2.0;

                    Eigen::MatrixXd info = Eigen::MatrixXd::Zero(6, 6);
                    info(0,0) = info(1,1) = info(2,2) = w_trans_hist;
                    info(3,3) = info(4,4) = info(5,5) = w_rot_hist;

                    auto* e = new g2o::EdgeSE3();
                    e->setVertex(0, optimizer.vertex(hist_edge.to_id));
                    e->setVertex(1, optimizer.vertex(hist_edge.from_id));
                    e->setMeasurement(meas);
                    e->setInformation(info);

                    // 历史边也用自适应 Huber
                    // 用 poses_snapshot 计算距离（已在锁内拷贝完毕，无需再取 keyframes 锁）
                    double hist_dist = 2.0;
                    if (hist_edge.from_id < (int)poses_snapshot.size() &&
                        hist_edge.to_id < (int)poses_snapshot.size()) {
                        hist_dist = (poses_snapshot[hist_edge.from_id].translation()
                                - poses_snapshot[hist_edge.to_id].translation()).norm();
                    }
                    double hist_delta = std::max(2.0, hist_dist * 0.3);

                    auto* rk_hist = new g2o::RobustKernelHuber();
                    rk_hist->setDelta(hist_delta);
                    e->setRobustKernel(rk_hist);

                    optimizer.addEdge(e);
                }
            }
            // --- 7. 执行优化 ---
            optimizer.initializeOptimization();
            optimizer.optimize(pgo_max_iterations_);

            auto pgo_end = std::chrono::high_resolution_clock::now();
            double pgo_time_ms = std::chrono::duration<double, std::milli>(pgo_end - pgo_start).count();

            // ================== 🔥【2026.3.30 修改：PGO 回写 + 计算前端校正增量】起始 ==================
            // --- 8. 读取优化后的位姿，回写到全局 keyframes，并提取前端校正量 ---
            {
                // ================== 🔥【P0修复D：PGO回写时强制正交化rotation】起始 ==================
                // g2o数值迭代后rotation可能偏离SO(3)，导致Isometry3d::inverse()计算错误
                // 这是37m/130°虚假跳变的直接根源
                std::lock_guard<std::mutex> lock(mtx_keyframes_);
                int n_to_update = std::min(num_keyframes, (int)keyframes.size());
                for (int i = 0; i < n_to_update; i++) {
                    auto* v = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(i));
                    if (v) {
                        Eigen::Isometry3d optimized = v->estimate();
                        Eigen::Matrix3d R = optimized.linear();
                        // SVD正交化: R = U * V^T
                        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
                        Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
                        // 确保行列式为+1（右手系）
                        if (R_ortho.determinant() < 0) {
                            Eigen::Matrix3d V = svd.matrixV().transpose();
                            V.row(2) *= -1;
                            R_ortho = svd.matrixU() * V;
                        }
                        keyframes[i].position = optimized.translation();
                        keyframes[i].rotation = R_ortho;
                    }
                }
                // ================== 🔥【P0修复D】结束 ==================

                // ================== 🔥【P0修复1：记录实际优化的回环目标帧ID】起始 ==================
                // 用current_id而非n_to_update-1，避免竞态条件下keyframes新增导致的错位
                auto* v_target = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(current_id));
                if (v_target) {
                    Eigen::Isometry3d latest_opt = v_target->estimate();
                    std::lock_guard<std::mutex> lock2(mtx_correction_);
                    correction_target_kf_id_ = current_id;   // ← 前端用此ID取对应keyframe
                    correction_pos_ = latest_opt.translation();
                    correction_rot_ = latest_opt.linear();
                    has_loop_correction_ = true;
                }
                // ================== 🔥【P0修复1】结束 ==================
            }




            // ================== 【可视化升级：按质量分色 + 粗细】起始 ==================
            {
                std::scoped_lock lock(mtx_keyframes_, mtx_loop_pairs_);
                visualization_msgs::msg::MarkerArray marker_array;
                for (int i = 0; i < (int)loop_closure_pairs_.size(); i++) {
                    const auto& lp = loop_closure_pairs_[i];
                    if (lp.from_id >= (int)keyframes.size() || lp.to_id >= (int)keyframes.size()) continue;

                    visualization_msgs::msg::Marker lm;
                    lm.header.frame_id = "odom_fast_lio2";
                    lm.header.stamp = this->get_clock()->now();
                    lm.ns = "loop_lines"; lm.id = i;
                    lm.type = visualization_msgs::msg::Marker::LINE_LIST;
                    lm.action = visualization_msgs::msg::Marker::ADD;

                    // 按 fitness score 分三档颜色和粗细：
                    //   绿色粗线：MSE < 1.0（优秀匹配）
                    //   黄色中线：MSE 1.0 ~ 3.0（一般匹配）
                    //   红色细线：MSE > 3.0（较差匹配，被 Cauchy 核压制）
                    if (lp.fitness_score < 1.0) {
                        lm.scale.x = 0.20;
                        lm.color.r = 0.0f; lm.color.g = 1.0f; lm.color.b = 0.0f; lm.color.a = 1.0f;
                    } else if (lp.fitness_score < 3.0) {
                        lm.scale.x = 0.12;
                        lm.color.r = 1.0f; lm.color.g = 1.0f; lm.color.b = 0.0f; lm.color.a = 0.8f;
                    } else {
                        lm.scale.x = 0.06;
                        lm.color.r = 1.0f; lm.color.g = 0.2f; lm.color.b = 0.0f; lm.color.a = 0.5f;
                    }

                    geometry_msgs::msg::Point pa, pb;
                    pa.x = keyframes[lp.from_id].position.x();
                    pa.y = keyframes[lp.from_id].position.y();
                    pa.z = keyframes[lp.from_id].position.z();
                    pb.x = keyframes[lp.to_id].position.x();
                    pb.y = keyframes[lp.to_id].position.y();
                    pb.z = keyframes[lp.to_id].position.z();
                    lm.points.push_back(pa);
                    lm.points.push_back(pb);
                    marker_array.markers.push_back(lm);
                }
                pubLoopClosureLines_->publish(marker_array);
            // ================== 【可视化升级：按质量分色 + 粗细】结束 ==================
            



                // 👇 2. 新增：刷新所有历史关键帧球体的位置
                visualization_msgs::msg::MarkerArray kf_marker_array;
                for (int i = 0; i < (int)keyframes.size(); i++) {
                    visualization_msgs::msg::Marker kf_marker;
                    kf_marker.header.frame_id = "odom_fast_lio2";
                    kf_marker.header.stamp = this->get_clock()->now();
                    kf_marker.ns = "keyframes"; 
                    kf_marker.id = i; // ID 一一对应，RViz 会自动把旧位置的球拉到新位置
                    kf_marker.type = visualization_msgs::msg::Marker::SPHERE;
                    kf_marker.action = visualization_msgs::msg::Marker::ADD;
                    
                    // 使用 PGO 刚刚回写修正过的新位置！
                    kf_marker.pose.position.x = keyframes[i].position.x();
                    kf_marker.pose.position.y = keyframes[i].position.y();
                    kf_marker.pose.position.z = keyframes[i].position.z();
                    
                    // 补充合法的四元数
                    kf_marker.pose.orientation.w = 1.0; 
                    kf_marker.pose.orientation.x = 0.0;
                    kf_marker.pose.orientation.y = 0.0;
                    kf_marker.pose.orientation.z = 0.0;
                    
                    kf_marker.scale.x = 0.3; kf_marker.scale.y = 0.3; kf_marker.scale.z = 0.3;
                    kf_marker.color.r = 1.0; kf_marker.color.a = 1.0; 
                    
                    kf_marker_array.markers.push_back(kf_marker);
                }
                pubKeyframesArray_->publish(kf_marker_array);
            }
            // ================== 🔥【2026.3.30 修改：PGO 后刷新可视化连线】结束 ==================

            pgo_end = std::chrono::high_resolution_clock::now();
            pgo_time_ms = std::chrono::duration<double, std::milli>(pgo_end - pgo_start).count();

            RCLCPP_INFO(this->get_logger(), 
                "✅ [PGO] 优化完成！帧 %d <-> %d | 顶点:%d | 耗时:%.1fms",
                current_id, loop_id, num_keyframes, pgo_time_ms);
            // ================== 🔥【2026.3.30 修改：PGO 回写 + 计算前端校正增量】结束 ==================


        }
    }
    // ================== 🔥【2026.3.26 新增：独立的图优化与ICP线程 (线程3)】结束 ==================



    void timer_callback()
    {
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();

            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;

            
            // ================================================================================
            // 🔥【PGO→前端校正：ikd-tree 重建 + EKF 状态修正】
            //
            //    插入位置：laserMapping.cpp → timer_callback() 函数内部
            //    在 "flg_EKF_inited = ..." 之后、"/*** Segment the map in lidar FOV ***/" 之前
            //
            //    即：第 2054 行（flg_EKF_inited = ...）之后
            //        第 2055 行（/*** Segment the map in lidar FOV ***/）之前
            //
            //    原理参考 FAST-LIO-LC (Yanliang Wang, 2022)：
            //      当后端 PGO 优化完成后，用优化后的关键帧位姿重建局部地图（ikd-tree），
            //      同时用"增量嫁接"公式修正当前 EKF 状态，消除长期漂移。
            // ================================================================================


           // ================== 🔥【PGO→前端校正：ikd-tree 重建 + EKF 状态修正】起始 ==================
            {
                // static 变量放在最外层作用域，生命周期覆盖整个函数调用
                static rclcpp::Time last_correction_time(0, 0, RCL_ROS_TIME);
                static Eigen::Vector3d last_correction_pos = Eigen::Vector3d::Zero();
                const double CORR_MIN_INTERVAL_S = 15.0;   // 改动 6 时再调此值
                const double CORR_MIN_MOVE_M = 5.0;

                // 先查冷却，冷却期内不消费标志
                auto now = this->get_clock()->now();
                double dt = (now - last_correction_time).seconds();
                double dmove = (state_point.pos - last_correction_pos).norm();
                bool in_cooldown = (dt < CORR_MIN_INTERVAL_S && dmove < CORR_MIN_MOVE_M);

                bool need_correction = false;   // ← 只声明一次
                if (!in_cooldown) {
                    // 不在冷却才消费标志（锁粒度最小化，只保护共享变量读写）
                    std::lock_guard<std::mutex> lock_corr(mtx_correction_);
                    if (has_loop_correction_) {
                        need_correction = true;
                        has_loop_correction_ = false;
                    }
                }

                if (need_correction) {
                    last_correction_time = now;
                    last_correction_pos = state_point.pos;
                }
                if (need_correction && flg_EKF_inited)
                {
                     RCLCPP_WARN(this->get_logger(),
                       "🔧 [PGO校正] 检测到后端优化完成，开始评估校正量...");
                    auto correction_start = std::chrono::high_resolution_clock::now();

                        // ====== 【新增】第零步：先算出预期校正目标，做安全护栏判断 ======
                    Eigen::Isometry3d T_kf_odom = Eigen::Isometry3d::Identity();
                    Eigen::Isometry3d T_kf_pgo  = Eigen::Isometry3d::Identity();
                    // ================== 🔥【P0修复1：前端校正使用PGO实际优化的目标帧】起始 ==================
                    bool has_keyframes_for_corr = false;
                    int target_kf_id = -1;
                    {
                        std::lock_guard<std::mutex> lock_corr(mtx_correction_);
                        target_kf_id = correction_target_kf_id_;
                    }
                    {
                        std::lock_guard<std::mutex> lock_kf(mtx_keyframes_);
                        if (!keyframes.empty() && target_kf_id >= 0 && target_kf_id < (int)keyframes.size()) {
                            has_keyframes_for_corr = true;
                            const auto& target_kf = keyframes[target_kf_id];
                            T_kf_odom.linear()      = target_kf.odom_rotation;
                            T_kf_odom.translation() = target_kf.odom_position;
                            T_kf_pgo.linear()       = target_kf.rotation;
                            T_kf_pgo.translation()  = target_kf.position;
                        }
                    }
                    // ================== 🔥【P0修复1】结束 ==================

                    if (!has_keyframes_for_corr) {
                        RCLCPP_WARN(this->get_logger(), "🔧 [PGO校正] 关键帧为空，跳过本次校正");
                    } else {
                        // ============ 【坐标系修复】统一到 LiDAR 世界系 ============
                        // 关键：state_point 是 IMU 位姿，但 keyframes 和 accumulated_loop_correction_
                        // 全部是 LiDAR 位姿。必须先把 state_point 加外参转成 LiDAR 世界位姿，
                        // 才能和 T_kf_odom / T_kf_pgo / accumulated 一起进入公式。
                        Eigen::Isometry3d T_current_polluted = Eigen::Isometry3d::Identity();
                        T_current_polluted.linear() = 
                            (state_point.rot * state_point.offset_R_L_I).toRotationMatrix();
                        T_current_polluted.translation() = state_point.pos 
                            + state_point.rot.toRotationMatrix() * state_point.offset_T_L_I;

                        // ================== 🔥【彻底修复：用独立 pure_odom 链算当前帧 LiDAR 系纯净位姿】 ==================
                        // 不再"反向剥离"——那个公式只在 PGO 是全局刚体变换时成立，
                        // 但 PGO 把每个关键帧调到不同位置，公式会坏掉。
                        // 现在 pure_odom_state_* 是独立维护的、永远干净的 IMU 系纯 odom 位姿。
                        Eigen::Isometry3d T_current_pure_odom = Eigen::Isometry3d::Identity();
                        T_current_pure_odom.linear() = (pure_odom_state_rot_ 
                            * state_point.offset_R_L_I).toRotationMatrix();
                        T_current_pure_odom.translation() = pure_odom_state_pos_ 
                            + pure_odom_state_rot_.toRotationMatrix() * state_point.offset_T_L_I;

                        // 目标 LiDAR 位姿：pure_odom 上应用本次 PGO 全量校正
                        Eigen::Isometry3d T_corrected_lidar =
                            T_kf_pgo * T_kf_odom.inverse() * T_current_pure_odom;

                        // 【护栏】计算 LiDAR 位姿的跳变（两边都是 LiDAR 系，可直接相减）
                        double jump_trans = (T_corrected_lidar.translation() 
                                           - T_current_polluted.translation()).norm();
                        double jump_rot_deg = Eigen::AngleAxisd(
                            T_corrected_lidar.linear() * T_current_polluted.linear().transpose()
                        ).angle() * 180.0 / M_PI;

                        const double MAX_JUMP_TRANS = 30.0;
                        const double MAX_JUMP_ROT   = 45.0;

                        if (jump_trans > MAX_JUMP_TRANS || jump_rot_deg > MAX_JUMP_ROT) {
                            RCLCPP_ERROR(this->get_logger(),
                                "⛔ [安全护栏] 校正幅度异常 (%.2fm/%.1f°)，放弃本次校正！"
                                "（不重建 ikd-tree，不改 state，等待下次回环）",
                                jump_trans, jump_rot_deg);
                            // 整块跳过
                        } else {
                            RCLCPP_WARN(this->get_logger(),
                                "✅ [PGO校正] 跳变检查通过 (%.2fm/%.1f°)，开始重建地图和修正状态...",
                                jump_trans, jump_rot_deg);

                            // ========== 第一步：重建 ikd-tree ==========
                            struct KFSnap {
                                Eigen::Vector3d pos;
                                Eigen::Matrix3d rot;
                                PointCloudXYZI::Ptr cloud;
                            };
                            std::vector<KFSnap> kf_snaps;
                            Eigen::Vector3d latest_pos;
                            {
                                std::lock_guard<std::mutex> lock_kf(mtx_keyframes_);
                                // 上面已确认非空，这里直接拷贝
                                latest_pos = keyframes.back().position;
                                kf_snaps.reserve(keyframes.size());
                                for (const auto& kf_item : keyframes) {
                                    kf_snaps.push_back({kf_item.position,
                                                        kf_item.rotation,
                                                        kf_item.cloud});
                                }
                            }

                            PointCloudXYZI::Ptr submap_for_rebuild(new PointCloudXYZI());
                            // ================== 🔥【修复：ikd-tree重建包含全部keyframes】起始 ==================
                            // 原50m半径只覆盖局部，导致远处地图丢失！改为包含全部keyframes
                            // 降采样后点数可控，重建时间可接受
                            double rebuild_radius = 1000.0;  // 覆盖整个轨迹
                            int total_snaps = (int)kf_snaps.size();
                            int included_count = 0;
                            for (int i = 0; i < total_snaps; i++) {
                                double dist_to_latest = (kf_snaps[i].pos - latest_pos).norm();
                                bool is_nearby = (dist_to_latest < rebuild_radius);
                                bool is_recent = (i >= total_snaps - 10);
                                if (!is_nearby && !is_recent) continue;
                                included_count++;
                                // ================== 🔥【修复：ikd-tree重建包含全部keyframes】结束 ==================

                                Eigen::Affine3d T_world = Eigen::Affine3d::Identity();
                                T_world.linear()      = kf_snaps[i].rot;
                                T_world.translation() = kf_snaps[i].pos;

                                PointCloudXYZI::Ptr cloud_in_world(new PointCloudXYZI());
                                pcl::transformPointCloud(*kf_snaps[i].cloud,
                                                        *cloud_in_world,
                                                        T_world.matrix());
                                *submap_for_rebuild += *cloud_in_world;
                            }

                            pcl::VoxelGrid<PointType> vg_rebuild;
                            vg_rebuild.setLeafSize(filter_size_map_min,
                                                filter_size_map_min,
                                                filter_size_map_min);
                            vg_rebuild.setInputCloud(submap_for_rebuild);
                            vg_rebuild.filter(*submap_for_rebuild);

                            RCLCPP_INFO(this->get_logger(),
                                "🗺️ [PGO校正] Submap 构建完成: %zu 点 (来自 %d/%d 关键帧)，准备重建 ikd-tree...",
                                submap_for_rebuild->points.size(), included_count, total_snaps);

                            ikdtree.set_downsample_param(filter_size_map_min);
                            ikdtree.Build(submap_for_rebuild->points);
                            Localmap_Initialized = false;

                            RCLCPP_INFO(this->get_logger(),
                                "✅ [PGO校正] ikd-tree 重建完成！新树大小: %d",
                                ikdtree.size());

                            // 【注意】不清理pcl_wait_pub，否则RViz中历史地图会全部消失
                            // 旧点保留在旧位置，新点用校正后的位姿加入，运行时会有轻微重影
                            // 全局一致的地图在退出时通过save_corrected_map()输出

                            // ========== 第二步：更新累积校正量 + state ==========
                            // 【严格修复】显式分离旧/新校正量，确保数学一致性
                            Eigen::Isometry3d C_old = accumulated_loop_correction_;
                            Eigen::Isometry3d C_new = T_kf_pgo * T_kf_odom.inverse();
                            accumulated_loop_correction_ = C_new;
                            
                            RCLCPP_INFO(this->get_logger(),
                                "🔧 [累积校正] 旧=%.3fm | 新=%.3fm | 本次跳变: %.3fm",
                                C_old.translation().norm(), C_new.translation().norm(), jump_trans);

                            // ============ 【坐标系修复】LiDAR 位姿 → IMU 位姿 ============
                            // state_point.pos/rot 是 IMU 世界位姿，不能直接塞 LiDAR 位姿
                            // 关系：T_lidar_world = T_imu_world × T_imu_to_lidar
                            //       → T_imu_world = T_lidar_world × T_imu_to_lidar⁻¹
                            //       → R_imu = R_lidar × R_I_L⁻¹
                            //         t_imu = t_lidar - R_imu × t_I_L
                            Eigen::Quaterniond q_lidar_corrected(T_corrected_lidar.rotation());
                            Eigen::Quaterniond q_imu_corrected = 
                                q_lidar_corrected * state_point.offset_R_L_I.conjugate();
                            Eigen::Vector3d t_imu_corrected = 
                                T_corrected_lidar.translation() 
                                - q_imu_corrected * state_point.offset_T_L_I;

                            state_ikfom state_corrected = kf.get_x();
                            state_corrected.pos = t_imu_corrected;    // ← IMU 位置
                            state_corrected.rot = q_imu_corrected;    // ← IMU 旋转
                            kf.change_x(state_corrected);

                            esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_corrected = kf.get_P();
                            P_corrected.setIdentity();
                            P_corrected(6,6)   = P_corrected(7,7)   = P_corrected(8,8)   = 0.00001;
                            P_corrected(9,9)   = P_corrected(10,10) = P_corrected(11,11) = 0.00001;
                            P_corrected(15,15) = P_corrected(16,16) = P_corrected(17,17) = 0.0001;
                            P_corrected(18,18) = P_corrected(19,19) = P_corrected(20,20) = 0.001;
                            P_corrected(21,21) = P_corrected(22,22) = 0.00001;
                            kf.change_P(P_corrected);

                            state_point = kf.get_x();
                            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

                            // ================== 🔥【彻底修复：PGO 改写 state 后同步刷新 last_state 缓存】起始 ==================
                            // PGO 把 state_point 跳到了新位姿，
                            // 如果不刷新 last_state_for_pure_odom_，下一帧的 delta = last⁻¹ × now
                            // 会把整个 PGO jump 当成"odom 增量"算进 pure_odom，那就完全坏了。
                            // 这里把 last_state 同步到刚被 PGO 改写后的 state，让下一帧增量从这里干净开始。
                            // pure_odom_state_* 本身保持不变 —— 它就是要继续按"如果没有 PGO 会怎样"推进。
                            last_state_pos_for_pure_odom_ = state_point.pos;
                            last_state_rot_for_pure_odom_ = state_point.rot;
                            // ================== 🔥【彻底修复：PGO 改写 state 后同步刷新 last_state 缓存】结束 ==================

                            auto correction_end = std::chrono::high_resolution_clock::now();
                            double correction_ms = std::chrono::duration<double, std::milli>(
                                correction_end - correction_start).count();

                            RCLCPP_INFO(this->get_logger(),
                                "✅ [PGO校正] EKF 状态修正完成！LiDAR 平移修正量: %.4fm | 耗时: %.1fms",
                                jump_trans, correction_ms);

   

                            // Path 重写（关键帧是 LiDAR 位姿，这里直接用没问题）
                            {
                                std::lock_guard<std::mutex> lock_kf(mtx_keyframes_);
                                path.poses.clear();
                                for (const auto& k : keyframes) {
                                    geometry_msgs::msg::PoseStamped p;
                                    p.header.frame_id = "odom_fast_lio2";
                                    p.header.stamp = get_ros_time(k.time);
                                    p.pose.position.x = k.position.x();
                                    p.pose.position.y = k.position.y();
                                    p.pose.position.z = k.position.z();
                                    Eigen::Quaterniond q(k.rotation);
                                    p.pose.orientation.x = q.x();
                                    p.pose.orientation.y = q.y();
                                    p.pose.orientation.z = q.z();
                                    p.pose.orientation.w = q.w();
                                    path.poses.push_back(p);
                                }
                                pubPath_->publish(path);
                            }
                        }  // ← 关闭护栏通过的 else
                    }      // ← 关闭 has_keyframes_for_corr 的 else
                }
            }
        



            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();


            // ================== 🔥【Patchwork++ 前端去地面：EKF 前分割，带安全兜底】起始 ==================
            // 注意：Patchwork++ 新版 API 使用 Eigen::MatrixXf 接口，需要 PCL ↔ Eigen 双向转换
            if (ground_seg_enable_ && patchworkpp_ != nullptr && flg_EKF_inited)
            {
                auto seg_start = std::chrono::high_resolution_clock::now();
                int points_before = feats_undistort->points.size();

                // ---- 1. PCL → Eigen::MatrixXf ----
                // Patchwork++ 要求 N×3 矩阵（只要 XYZ），或 N×4（含 intensity）
                // 我们用 N×4 以便 RNR（反射噪声移除）利用 intensity 信息
                Eigen::MatrixXf cloud_eigen(points_before, 4);
                for (int i = 0; i < points_before; i++) {
                    cloud_eigen(i, 0) = feats_undistort->points[i].x;
                    cloud_eigen(i, 1) = feats_undistort->points[i].y;
                    cloud_eigen(i, 2) = feats_undistort->points[i].z;
                    cloud_eigen(i, 3) = feats_undistort->points[i].intensity;
                }

                // ---- 2. 调用 Patchwork++ 核心 ----
                patchworkpp_->estimateGround(cloud_eigen);

                // ---- 3. 取回结果（Eigen::MatrixX3f）----
                Eigen::MatrixX3f ground_eigen    = patchworkpp_->getGround();
                Eigen::MatrixX3f nonground_eigen = patchworkpp_->getNonground();

                int points_after = nonground_eigen.rows();

                // 🛡️ 安全兜底：非地面点太少就不替换
                const int MIN_NONGROUND_POINTS = 500;
                if (points_after >= MIN_NONGROUND_POINTS)
                {
                    // ---- 4. Eigen → PCL：用 nonground 替换 feats_undistort ----
                    // 注意：Patchwork++ 返回的是去除了 intensity 的 N×3，我们复用原点的 intensity
                    // 但由于 getNonground 没有返回索引，只能新建点云，intensity 设 0
                    // （如果需要保留 intensity，要用 getNongroundIndices 版本）
                    PointCloudXYZI::Ptr cloud_nonground(new PointCloudXYZI());
                    cloud_nonground->points.reserve(points_after);
                    for (int i = 0; i < points_after; i++) {
                        PointType pt;
                        pt.x = nonground_eigen(i, 0);
                        pt.y = nonground_eigen(i, 1);
                        pt.z = nonground_eigen(i, 2);
                        pt.intensity = 0.0f;   // 无法从 Eigen 反推，置零
                        cloud_nonground->points.push_back(pt);
                    }
                    cloud_nonground->width = points_after;
                    cloud_nonground->height = 1;
                    cloud_nonground->is_dense = false;

                    // 覆盖 feats_undistort，后续流程自动基于无地面点云工作
                    feats_undistort->clear();
                    *feats_undistort = *cloud_nonground;

                    int removed = points_before - points_after;
                    total_ground_points_removed_ += removed;
                    total_gs_frames_processed_++;

                    if (total_gs_frames_processed_ % 50 == 0) {
                        auto seg_end = std::chrono::high_resolution_clock::now();
                        double seg_ms = std::chrono::duration<double, std::milli>(
                            seg_end - seg_start).count();
                        double avg_removed = (double)total_ground_points_removed_
                                             / total_gs_frames_processed_;
                        RCLCPP_INFO(this->get_logger(),
                            "🌱 [Patchwork++] 帧 %d | %d → %d (去地面 %d 点) | 均值 %.0f | "
                            "耗时 %.1f ms | PW++ 自测 %lf us",
                            total_gs_frames_processed_, points_before, points_after,
                            removed, avg_removed, seg_ms, patchworkpp_->getTimeTaken());
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(),
                        "⚠️ [Patchwork++] 非地面点仅 %d（<%d），回退完整点云",
                        points_after, MIN_NONGROUND_POINTS);
                }
            }
            // ================== 🔥【Patchwork++ 前端去地面：EKF 前分割，带安全兜底】结束 ==================

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            // ================== 🔥【彻底修复：推进独立的 pure_odom 链】起始 ==================
            // 关键：必须在任何 PGO change_x 之前推进 pure_odom，
            // 因为后面的 PGO 校正块会改写 state_point，会破坏 last_state 的连续性。
            // 这里 state_point 是本帧 EKF 收敛后、未被 PGO 污染的最新值，是干净的。
            {
                Eigen::Vector3d state_now_pos = state_point.pos;        // IMU 系
                Eigen::Quaterniond state_now_rot = state_point.rot;     // IMU 系
                if (!pure_odom_initialized_) {
                    // 第一次：用当前 state 直接初始化 pure_odom（pure_odom 起点 = state 起点）
                    pure_odom_state_pos_ = state_now_pos;
                    pure_odom_state_rot_ = state_now_rot;
                    last_state_pos_for_pure_odom_ = state_now_pos;
                    last_state_rot_for_pure_odom_ = state_now_rot;
                    pure_odom_initialized_ = true;
                } else {
                    // 计算 EKF state 从上一帧到本帧的相对增量（IMU 系）
                    // delta_R = R_last⁻¹ × R_now,  delta_t = R_last⁻¹ × (t_now - t_last)
                    Eigen::Quaterniond delta_R =
                        last_state_rot_for_pure_odom_.conjugate() * state_now_rot;
                    Eigen::Vector3d delta_t =
                        last_state_rot_for_pure_odom_.conjugate() * (state_now_pos - last_state_pos_for_pure_odom_);
                    // 把这个增量累加到 pure_odom 链上
                    pure_odom_state_pos_ = pure_odom_state_pos_ + pure_odom_state_rot_ * delta_t;
                    pure_odom_state_rot_ = (pure_odom_state_rot_ * delta_R).normalized();
                    // 更新缓存：下一帧用本帧的 state 算下一次增量
                    last_state_pos_for_pure_odom_ = state_now_pos;
                    last_state_rot_for_pure_odom_ = state_now_rot;
                }
            }
            // ================== 🔥【彻底修复：推进独立的 pure_odom 链】结束 ==================

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);


            // ================== 🔥【校正前端地图：逐帧存储 body 系点云 + 原始位姿】起始 ==================
            // 此时 state_point 是本帧 EKF 最终收敛的最高精度位姿。
            // 将 body 系点云（降采样后）和 LiDAR 世界位姿存入 all_frames_for_map_，
            // 供退出时的 save_corrected_frontend_map() 逐帧校正后重新拼接。
            {
                FrameForCorrectedMap frame_snap; 
                //不降采样
                frame_snap.cloud_body.reset(new PointCloudXYZI());
                pcl::copyPointCloud(*feats_undistort, *frame_snap.cloud_body);

                // ================== 🔥【彻底修复：直接用独立的 pure_odom 链】起始 ==================
                // 不再"反向剥离"——那是错的（accumulated_loop_correction_ 不是全局刚体变换）
                // 现在 pure_odom_state_* 是独立维护的、永远干净的 IMU 系 odom 位姿，
                // 加外参后转成 LiDAR 系，直接保存。
                Eigen::Matrix3d pure_lidar_rot = (pure_odom_state_rot_ 
                    * state_point.offset_R_L_I).toRotationMatrix();
                Eigen::Vector3d pure_lidar_pos = pure_odom_state_pos_ 
                    + pure_odom_state_rot_.toRotationMatrix() * state_point.offset_T_L_I;
                frame_snap.lidar_pos = pure_lidar_pos;
                frame_snap.lidar_rot = pure_lidar_rot;
                frame_snap.time = Measures.lidar_end_time;
                // ================== 🔥【彻底修复：直接用独立的 pure_odom 链】结束 ==================

                all_frames_for_map_.push_back(std::move(frame_snap));
            }
            // ================== 🔥【校正前端地图：逐帧存储 body 系点云 + 原始位姿】结束 ==================
 


            // ================== 🔥【2026.3.25 新增：第四步 - 核心关键帧提取与保存逻辑】开始 ==================
            // 注意：此时刚刚经历完卡尔曼滤波更新(kf.update_iterated_dyn_share_modified)，这里的位姿是前端能提供的最高精度！

            // 1. 获取当前精确状态信息（LiDAR 世界位姿 = IMU 位姿 × 外参）
            // cur_pos/cur_rot 基于 state_point —— 这是"PGO 校正后的"位姿（用于显示和 SC 检索）
            Eigen::Vector3d cur_pos = state_point.pos 
                + state_point.rot.toRotationMatrix() * state_point.offset_T_L_I;
            Eigen::Matrix3d cur_rot = (state_point.rot * state_point.offset_R_L_I).toRotationMatrix();
            double cur_time = Measures.lidar_end_time;

            // ================== 🔥【彻底修复：直接用独立 pure_odom 链算 LiDAR 系纯净位姿】起始 ==================
            // 不再"反向剥离 cur_pos" —— 那个公式只在 PGO 是全局刚体变换时成立，
            // 但 PGO 把每个关键帧调整到不同位置，公式就坏了，导致 odom_position 被污染。
            // 现在 pure_odom_state_* 是独立维护的、永远干净的 IMU 系纯 odom 位姿。
            Eigen::Matrix3d pure_odom_rot = (pure_odom_state_rot_ 
                * state_point.offset_R_L_I).toRotationMatrix();
            Eigen::Vector3d pure_odom_pos = pure_odom_state_pos_ 
                + pure_odom_state_rot_.toRotationMatrix() * state_point.offset_T_L_I;
            // ================== 🔥【彻底修复：直接用独立 pure_odom 链】结束 ==================


    

            bool save_this_frame = false; // 定义一个局部标志位，用于判定当前帧是否符合关键帧的提取标准，初始设为 false

            // 2. 核心的关键帧判定逻辑：决定到底要不要保留这一帧
            if (is_first_keyframe) // 判断当前是否是系统启动后接收到的第一帧数据
            {
                save_this_frame = true; // 第一帧无条件设为关键帧，作为整个里程计和位姿图的起点（锚点）
            }

            else // 如果不是第一帧，则需要与上一个保存的关键帧进行位姿差别的计算
            {
                // 计算平移变化量：当前位置减去上一关键帧位置得到向量，再用 norm() 取其欧氏距离（单位：米）
                double dist = (cur_pos - last_keyframe.position).norm();
                
                // 计算旋转变化矩阵：公式为 (R_last 的转置) 乘以 (R_current)，得到两帧之间的相对旋转
                Eigen::Matrix3d R_diff = last_keyframe.rotation.transpose() * cur_rot;
                
                // 利用 Eigen 库将相对旋转矩阵转化为轴角表示，并提取出旋转的绝对角度值（单位：弧度）
                double angle = Eigen::AngleAxisd(R_diff).angle();

                // 判断核心条件：如果移动的距离(dist)或转弯的角度(angle)超过了我们在 yaml/类中预设的阈值
                if (dist > keyframe_dist_threshold || angle > keyframe_angle_threshold)
                {
                    save_this_frame = true; // 触发关键帧提取条件，将标志位设为 true，核准提取！
                }
            }

            // 3. 执行物理保存与二次下采样（极致优化内存，同时为 Scan Context 完美铺垫）
            if (save_this_frame) // 如果经过上面的判定，当前帧被确认为关键帧
            {
        

                // ================== 🔥【2026.3.30 修改：存储 LiDAR 世界位姿（含外参补偿）】起始 ==================
                KeyFrame kf_to_save;
                kf_to_save.position = cur_pos;                   // 当前显示位姿（会被 PGO 覆写）
                kf_to_save.rotation = cur_rot;                   // 当前显示旋转（会被 PGO 覆写）
                kf_to_save.odom_position = pure_odom_pos;        // 🔥 真正纯净的 odom（反向剥离累积校正后）
                kf_to_save.odom_rotation = pure_odom_rot;        // 🔥 真正纯净的 odom（反向剥离累积校正后）
                kf_to_save.time = cur_time;
                
                kf_to_save.cloud.reset(new PointCloudXYZI());
                // ================== 🔥【2026.3.31 修改：参考 FAST-LIO-SAM，存完整 feats_undistort】起始 ==================
                // FAST-LIO-SAM 的做法：关键帧直接存完整去畸变点云，不做任何下采样
                // 每帧约 1~3 万点 × 16 字节 ≈ 0.5MB，200 帧也只占 ~100MB 内存，完全可控
                // SC 和 ICP 在使用时各自临时下采样，不影响速度
                // 地图密度在保存阶段控制（同时输出稠密版和滤波版）
                pcl::copyPointCloud(*feats_undistort, *kf_to_save.cloud);
                // ================== 🔥【2026.3.31 修改：参考 FAST-LIO-SAM，存完整 feats_undistort】结束 ==================
                // ================== 🔥【2026.3.30 修改：存储 LiDAR 世界位姿（含外参补偿）】结束 ==================




              // ================== 🔥【2026.3.26 修改：前台安全提交与条件唤醒】起始 ==================
                
                // 提前定义两个局部变量，用来在锁内接住当前帧的 ID 和点数，供锁外安全使用
                int saved_id = -1;
                int saved_cloud_size = 0;

                // 加锁操作：保护全局关键帧数据库，防止和后台线程起冲突
                {
                    std::lock_guard<std::mutex> lock(mtx_keyframes_);
                    
                    // 将组装好的关键帧压入全局历史数据库的最末端
                    keyframes.push_back(kf_to_save); 

                    // ⚠️ 修复锁外访问 UB：在锁依然生效时，安全地获取当前刚压入的索引 ID 和点云大小
                    saved_id = (int)keyframes.size() - 1;
                    saved_cloud_size = kf_to_save.cloud->points.size();

                    // 把最新一帧的 ID (数组下标) 扔进篮子
                    new_keyframes_queue_.push(saved_id);
                } // <--- lock 作用域结束，锁在这里安全释放

                // 🌟 核心升级：敲响铃铛！通过条件变量瞬间唤醒正在后台沉睡的 loopClosureThread，0延迟开始处理！
                cv_keyframes_.notify_one(); 

                // 刷新“上一关键帧”的记录指针，让未来的帧和当前这帧进行比较
                last_keyframe = kf_to_save; 
                
                // 第一帧特权关闭，后续所有帧都必须老老实实走距离和角度的计算考核
                is_first_keyframe = false; 

                // 在终端打印一行清爽的日志（全部使用锁内拷贝出来的安全局部变量，消除 UB 隐患）
                std::cout << "[回环前端] 新增关键帧 ID: " << saved_id 
                          << " | 此时点云数量: " << saved_cloud_size 
                          << " | 总数: " << saved_id + 1 << std::endl;

                // ---- 以下代码全部是为了让你在 RViz 里能直观看见漂亮的红色轨迹点而写 ----
                visualization_msgs::msg::Marker marker; 
                marker.header.frame_id = "odom_fast_lio2"; 
                marker.header.stamp = this->get_clock()->now(); 
                marker.ns = "keyframes"; 
                
                // ⚠️ 修复 Marker ID 差 1 的 Bug：直接使用安全的 saved_id
                marker.id = saved_id; 
                
                marker.type = visualization_msgs::msg::Marker::SPHERE; 
                marker.action = visualization_msgs::msg::Marker::ADD; 
                
                marker.pose.position.x = state_point.pos(0); 
                marker.pose.position.y = state_point.pos(1); 
                marker.pose.position.z = state_point.pos(2);

                // 👇 新增：必须赋予一个合法的单位四元数，防止 RViz2 报错或渲染异常
                marker.pose.orientation.w = 1.0; 
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                
                
                marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3; 
                marker.color.r = 1.0; marker.color.a = 1.0; 
                
                pubKeyframes_->publish(marker); 
                
                // ================== 🔥【2026.3.26 修改：前台安全提交与条件唤醒】结束 ==================


            

            }
            // ================== 🔥【2026.3.25 新增：第四步 - 核心关键帧提取与保存逻辑】结束 ==================









            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }



    // ================== 🔥【2026.3.31 修改：参考 FAST-LIO-SAM，保存稠密+滤波两版全局地图】起始 ==================
    void save_corrected_map()
    {
        // --- 锁内只拷贝位姿和点云指针（shared_ptr 拷贝极快），锁外再做变换 ---
        struct KFSnapshot { 
            Eigen::Vector3d pos; 
            Eigen::Matrix3d rot; 
            PointCloudXYZI::Ptr cloud; 
        };
        std::vector<KFSnapshot> snapshots;
        {
            std::lock_guard<std::mutex> lock(mtx_keyframes_);
            snapshots.reserve(keyframes.size());
            for (const auto& kf : keyframes) {
                snapshots.push_back({kf.position, kf.rotation, kf.cloud});
            }
        } // 锁立即释放

        // --- 用 PGO 优化后的位姿，将每帧 LiDAR 点云拼到世界系 ---
        PointCloudXYZI::Ptr global_map(new PointCloudXYZI());
        for (int i = 0; i < (int)snapshots.size(); i++) {
            PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI());
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.linear() = snapshots[i].rot;
            T.translation() = snapshots[i].pos;
            pcl::transformPointCloud(*snapshots[i].cloud, *cloud_world, T.matrix());
            *global_map += *cloud_world;

            if (i % 20 == 0) {
                RCLCPP_INFO(this->get_logger(), "🗺️ 拼接进度: %d / %zu 帧...", i, snapshots.size());
            }
        }

        pcl::PCDWriter writer;

        // 1. 保存稠密版全局地图（和 FAST-LIO-SAM 的 GlobalMap.pcd 一样）
        std::string dense_path = root_dir + "PCD/GlobalMap.pcd";
        writer.writeBinary(dense_path, *global_map);
        RCLCPP_INFO(this->get_logger(), "✅ 稠密地图已保存: %s | 点数: %zu", 
                    dense_path.c_str(), global_map->points.size());

        // 2. 保存滤波版全局地图（和 FAST-LIO-SAM 的 filterGlobalMap.pcd 一样）
        PointCloudXYZI::Ptr global_map_ds(new PointCloudXYZI());
        pcl::VoxelGrid<PointType> ds;
        ds.setLeafSize(0.2f, 0.2f, 0.2f);
        ds.setInputCloud(global_map);
        ds.filter(*global_map_ds);

        std::string filter_path = root_dir + "PCD/filterGlobalMap.pcd";
        writer.writeBinary(filter_path, *global_map_ds);
        RCLCPP_INFO(this->get_logger(), "✅ 滤波地图已保存: %s | 点数: %zu", 
                    filter_path.c_str(), global_map_ds->points.size());
    }
    // ================== 🔥【2026.3.31 修改：参考 FAST-LIO-SAM，保存稠密+滤波两版全局地图】结束 ==================
 

    void save_corrected_trajectory()
    {
        struct PoseSnapshot { Eigen::Matrix3d R; Eigen::Vector3d t; double time; };   // ← 加 time
        std::vector<PoseSnapshot> opt_poses, odom_poses;
        {
            std::lock_guard<std::mutex> lock(mtx_keyframes_);
            for (const auto& kf : keyframes) {
                opt_poses.push_back({kf.rotation, kf.position, kf.time});      // ← 加 kf.time
                odom_poses.push_back({kf.odom_rotation, kf.odom_position, kf.time});  // ← 加 kf.time
            }
        }

        // 保存优化后轨迹（KITTI 格式：每行 12 个数，3x4 行优先变换矩阵）
        std::string opt_path = root_dir + "Lio_pose/corrected_trajectory.kitti";
        std::ofstream ofs(opt_path);
        for (const auto& p : opt_poses) {
            ofs << p.R(0,0) << " " << p.R(0,1) << " " << p.R(0,2) << " " << p.t(0) << " "
                << p.R(1,0) << " " << p.R(1,1) << " " << p.R(1,2) << " " << p.t(1) << " "
                << p.R(2,0) << " " << p.R(2,1) << " " << p.R(2,2) << " " << p.t(2) << std::endl;
        }
        ofs.close();

        // 追加保存 TUM 格式（带时间戳，供 evo 时间同步用）
        std::string opt_tum_path = root_dir + "Lio_pose/corrected_trajectory.tum";
        std::ofstream ofs_tum(opt_tum_path);
        ofs_tum << std::fixed << std::setprecision(9);
        for (const auto& p : opt_poses) {
            Eigen::Quaterniond q(p.R);
            ofs_tum << p.time << " "
                    << p.t(0) << " " << p.t(1) << " " << p.t(2) << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        ofs_tum.close();

        // 保存原始里程计轨迹（对比用）
        std::string odom_path = root_dir + "Lio_pose/original_trajectory.kitti";
        std::ofstream ofs2(odom_path);
        for (const auto& p : odom_poses) {
            ofs2 << p.R(0,0) << " " << p.R(0,1) << " " << p.R(0,2) << " " << p.t(0) << " "
                 << p.R(1,0) << " " << p.R(1,1) << " " << p.R(1,2) << " " << p.t(1) << " "
                 << p.R(2,0) << " " << p.R(2,1) << " " << p.R(2,2) << " " << p.t(2) << std::endl;
        }
        ofs2.close();

        std::string odom_tum_path = root_dir + "Lio_pose/original_trajectory.tum";
        std::ofstream ofs2_tum(odom_tum_path);
        ofs2_tum << std::fixed << std::setprecision(9);
        for (const auto& p : odom_poses) {
            Eigen::Quaterniond q(p.R);
            ofs2_tum << p.time << " "
                    << p.t(0) << " " << p.t(1) << " " << p.t(2) << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        ofs2_tum.close();

        RCLCPP_INFO(this->get_logger(), "✅ 轨迹已保存！优化后: %s | 原始: %s | 帧数: %zu",
                    opt_path.c_str(), odom_path.c_str(), opt_poses.size());

        // ================== 🔬【yaw 重复性诊断】打印每帧 PGO 修正量 ==================
        // 目的：跑两次 bag 后对比哪几个关键帧 yaw 修正不稳定（提示嫌疑回环边）
        // 只打印 |yaw_diff| > 0.5° 的帧，减少噪音；同时输出 csv 便于对比
        auto computeYawDeg = [](const Eigen::Matrix3d& R) -> double {
            // ZYX yaw
            return std::atan2(R(1,0), R(0,0)) * 180.0 / M_PI;
        };
        auto wrapDeg = [](double d) -> double {
            while (d > 180.0) d -= 360.0;
            while (d < -180.0) d += 360.0;
            return d;
        };

        std::string diag_path = root_dir + "Lio_pose/pgo_yaw_diagnosis.csv";
        std::ofstream ofs_diag(diag_path);
        ofs_diag << "frame_id,time,x_odom,y_odom,z_odom,yaw_odom_deg,"
                 << "x_pgo,y_pgo,z_pgo,yaw_pgo_deg,"
                 << "trans_diff_m,yaw_diff_deg\n";
        ofs_diag << std::fixed << std::setprecision(6);

        int n_significant = 0;
        double max_yaw_diff = 0.0;
        int max_yaw_diff_frame = -1;
        RCLCPP_WARN(this->get_logger(),
            "🔬 [yaw诊断] 打印 PGO 修正：仅 |yaw_diff|>0.5° 或 |trans_diff|>0.3m 的帧");
        for (size_t i = 0; i < opt_poses.size() && i < odom_poses.size(); ++i) {
            double yaw_o = computeYawDeg(odom_poses[i].R);
            double yaw_p = computeYawDeg(opt_poses[i].R);
            double yaw_diff = wrapDeg(yaw_p - yaw_o);
            double trans_diff = (opt_poses[i].t - odom_poses[i].t).norm();

            // 全部写入 csv（便于离线分析）
            ofs_diag << i << "," << opt_poses[i].time << ","
                     << odom_poses[i].t(0) << "," << odom_poses[i].t(1) << "," << odom_poses[i].t(2) << ","
                     << yaw_o << ","
                     << opt_poses[i].t(0) << "," << opt_poses[i].t(1) << "," << opt_poses[i].t(2) << ","
                     << yaw_p << ","
                     << trans_diff << "," << yaw_diff << "\n";

            // 控制台仅打印显著修正
            if (std::abs(yaw_diff) > 0.5 || trans_diff > 0.3) {
                RCLCPP_WARN(this->get_logger(),
                    "📍 帧%3zu  pos=(%6.2f,%6.2f,%6.2f) yaw_odom=%7.2f° yaw_pgo=%7.2f° "
                    "→ Δyaw=%+.2f° Δtrans=%.3fm",
                    i,
                    odom_poses[i].t(0), odom_poses[i].t(1), odom_poses[i].t(2),
                    yaw_o, yaw_p, yaw_diff, trans_diff);
                n_significant++;
                if (std::abs(yaw_diff) > std::abs(max_yaw_diff)) {
                    max_yaw_diff = yaw_diff;
                    max_yaw_diff_frame = (int)i;
                }
            }
        }
        ofs_diag.close();

        RCLCPP_WARN(this->get_logger(),
            "🔬 [yaw诊断完毕] 显著修正帧数=%d/%zu | 最大 yaw 修正=%.2f° (帧%d) | csv已保存=%s",
            n_significant, opt_poses.size(), max_yaw_diff, max_yaw_diff_frame, diag_path.c_str());
        // ================== 🔬【yaw 重复性诊断】结束 ==================
    }
    
    // ================== 🔥【2026.3.31 新增：PGO 优化后的全局地图和轨迹保存】结束 ==================

    // ================== 🔥【校正前端地图：退出时逐帧校正并拼接保存】起始 ==================
    // 原理：对运行时存储的每一帧，找到时间最近的关键帧，
    //       用"增量嫁接"公式（来自 FAST-LIO-LC）将原始位姿校正为 PGO 优化后的位姿，
    //       然后重新将 body 系点云变换到世界系，拼接成全局一致的前端地图。
    //
    //       与 save_corrected_map()（后端关键帧地图）的区别：
    //       - save_corrected_map：只用关键帧（每 1m/10deg 一帧），帧数少、有间隔
    //       - 本函数：用 所有前端帧（每帧都有），密度高、覆盖完整
    void save_corrected_frontend_map()
    {
        if (all_frames_for_map_.empty()) {
            RCLCPP_WARN(this->get_logger(), "🗺️ [校正前端地图] 无存储帧，跳过");
            return;
        }
 
        // ---- 1. 加锁拷贝关键帧的 odom 位姿和 PGO 位姿（用于嫁接公式）----
        struct KFRef {
            double time;
            Eigen::Vector3d odom_pos, pgo_pos;
            Eigen::Matrix3d odom_rot, pgo_rot;
        };
        std::vector<KFRef> kf_refs;
        {
            std::lock_guard<std::mutex> lock(mtx_keyframes_);
            kf_refs.reserve(keyframes.size());
            for (const auto& kf_item : keyframes) {
                KFRef ref;
                ref.time     = kf_item.time;
                ref.odom_pos = kf_item.odom_position;   // 永远不被 PGO 修改的纯净位姿
                ref.odom_rot = kf_item.odom_rotation;
                ref.pgo_pos  = kf_item.position;         // PGO 优化后的位姿
                ref.pgo_rot  = kf_item.rotation;
                kf_refs.push_back(ref);
            }
        } // 锁释放
 
        if (kf_refs.empty()) {
            RCLCPP_WARN(this->get_logger(), "🗺️ [校正前端地图] 无关键帧，跳过");
            return;
        }
 
        RCLCPP_INFO(this->get_logger(),
            "🗺️ [校正前端地图] 开始校正 %zu 帧 (参考 %zu 个关键帧)...",
            all_frames_for_map_.size(), kf_refs.size());
 
        // ---- 2. 逐帧校正并拼接 ----
        PointCloudXYZI::Ptr corrected_frontend_map(new PointCloudXYZI());
 
        // 先统计所有关键帧的PGO校正量，用于诊断
        double max_kf_corr = 0.0, avg_kf_corr = 0.0;
        int max_kf_idx = 0;
        for (int ki = 0; ki < (int)kf_refs.size(); ki++) {
            double corr = (kf_refs[ki].pgo_pos - kf_refs[ki].odom_pos).norm();
            avg_kf_corr += corr;
            if (corr > max_kf_corr) {
                max_kf_corr = corr;
                max_kf_idx = ki;
            }
        }
        avg_kf_corr /= kf_refs.size();
        RCLCPP_INFO(this->get_logger(),
            "🗺️ [校正前端地图] 关键帧PGO校正统计: 平均=%.3fm | 最大=%.3fm(帧%d) | 关键帧总数=%zu",
            avg_kf_corr, max_kf_corr, max_kf_idx, kf_refs.size());

        for (int fi = 0; fi < (int)all_frames_for_map_.size(); fi++)
        {
            const auto& frame = all_frames_for_map_[fi];
 
            // 找时间最近的关键帧
            int nearest_kf = 0;
            double min_dt = std::abs(frame.time - kf_refs[0].time);
            for (int j = 1; j < (int)kf_refs.size(); j++) {
                double dt = std::abs(frame.time - kf_refs[j].time);
                if (dt < min_dt) {
                    min_dt = dt;
                    nearest_kf = j;
                }
            }
 
            // 增量嫁接公式：T_corrected = T_odom(frame) × T_odom(kf)⁻¹ × T_pgo(kf)
            Eigen::Isometry3d T_frame_odom = Eigen::Isometry3d::Identity();
            T_frame_odom.linear()      = frame.lidar_rot;
            T_frame_odom.translation() = frame.lidar_pos;
 
            Eigen::Isometry3d T_kf_odom = Eigen::Isometry3d::Identity();
            T_kf_odom.linear()      = kf_refs[nearest_kf].odom_rot;
            T_kf_odom.translation() = kf_refs[nearest_kf].odom_pos;
 
            Eigen::Isometry3d T_kf_pgo = Eigen::Isometry3d::Identity();
            T_kf_pgo.linear()      = kf_refs[nearest_kf].pgo_rot;
            T_kf_pgo.translation() = kf_refs[nearest_kf].pgo_pos;
 
            Eigen::Isometry3d T_corrected = T_kf_pgo * T_kf_odom.inverse() * T_frame_odom;
 
            // 用校正后的位姿，将 body 系点云变换到世界系
            PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI());
            Eigen::Affine3d T_aff = Eigen::Affine3d::Identity();
            T_aff.linear()      = T_corrected.linear();
            T_aff.translation() = T_corrected.translation();
            pcl::transformPointCloud(*frame.cloud_body, *cloud_world, T_aff.matrix());
            *corrected_frontend_map += *cloud_world;
 
            // 进度日志
            if (fi % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "🗺️ [校正前端地图] 进度: %d / %zu 帧...",
                            fi, all_frames_for_map_.size());
            }
        }
 
        // ---- 3. 全局降采样后保存 ----
        PointCloudXYZI::Ptr corrected_map_ds(new PointCloudXYZI());
        pcl::VoxelGrid<PointType> ds_final;
        ds_final.setLeafSize(0.2f, 0.2f, 0.2f);
        ds_final.setInputCloud(corrected_frontend_map);
        ds_final.filter(*corrected_map_ds);
 
        pcl::PCDWriter writer;
 
        // 保存稠密版
        std::string dense_path = root_dir + "PCD/CorrectedFrontendMap.pcd";
        writer.writeBinary(dense_path, *corrected_frontend_map);
        RCLCPP_INFO(this->get_logger(),
            "✅ [校正前端地图] 稠密版已保存: %s | 点数: %zu",
            dense_path.c_str(), corrected_frontend_map->points.size());
 
        // 保存滤波版
        std::string filter_path = root_dir + "PCD/CorrectedFrontendMap_filtered.pcd";
        writer.writeBinary(filter_path, *corrected_map_ds);
        RCLCPP_INFO(this->get_logger(),
            "✅ [校正前端地图] 滤波版已保存: %s | 点数: %zu",
            filter_path.c_str(), corrected_map_ds->points.size());
    }
    // ================== 🔥【校正前端地图：退出时逐帧校正并拼接保存】结束 ==================



void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        // ================== 🔥【2026.3.31 修改：同时保存前端地图 + PGO 地图 + 轨迹】起始 ==================
        RCLCPP_INFO(this->get_logger(), "📦 开始保存地图和轨迹...");
        
        // 1. 保存前端实时地图（和原版 FAST-LIO2 一样）
        if (pcd_save_en) {
            save_to_pcd();
            RCLCPP_INFO(this->get_logger(), "✅ 前端地图已保存: %s", map_file_path.c_str());
        }
        
        // 2. 保存 PGO 优化后的全局一致地图
        save_corrected_map();
        
        // 3. 保存优化前/后的轨迹（供 EVO 评估）
        save_corrected_trajectory();
        
        res->success = true;
        res->message = "All maps and trajectories saved.";
        // ================== 🔥【2026.3.31 修改：同时保存前端地图 + PGO 地图 + 轨迹】结束 ==================
    }


private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;



    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;


        //2025.3.25 关键帧相关
    // ================== 🔥【2026.3.25 新增：第二步 - 关键帧类成员变量】开始 ==================
    // ================== 🔥【2026.3.27 修改：升级为 Deque 并增加连线锁】起始 ==================
        
    // 🌟 修复严重 Bug：使用 deque 替代 vector！
    // deque 在 push_back 时不会发生整体内存搬移（Reallocation），
    // 彻底杜绝了主线程的耗时尖刺，也保证了历史帧引用的绝对安全！
    std::deque<KeyFrame> keyframes;

    // 仅由 timer_callback 读写，记录上一帧状态（单线程安全）
    KeyFrame last_keyframe;
    pcl::VoxelGrid<PointType> downSizeFilterKeyFrame; //关键帧稠密点云滤波器
    bool is_first_keyframe = true;
    double keyframe_dist_threshold = 1.0;             
    double keyframe_angle_threshold = 10.0 * M_PI/180.0; 

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubKeyframes_;

    // 👇 新增：用于 PGO 优化后批量刷新历史关键帧位置的 MarkerArray 发布器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubKeyframesArray_;
        
    // --- 回环相关 ---
    bool loop_closure_enable = false;    
    double sc_dist_thres = 0.2;          
    SCManager scManager;    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopClosureLines_;

    // ================== 【可视化升级：带质量信息的回环对】起始 ==================
    struct LoopPairViz {
        int from_id;
        int to_id;
        double fitness_score;   // ICP 匹配质量
        double corr_trans;      // 修正平移量
    };
    std::mutex mtx_loop_pairs_;
    std::vector<LoopPairViz> loop_closure_pairs_;
    // ================== 【可视化升级：带质量信息的回环对】结束 ==================


        
    // ================== 🔥【2026.3.27 修改：升级为 Deque 并增加连线锁】结束 ==================




// ================== 🔥【2026.3.26 修改：后台回环线程相关变量 (工业级安全版)】起始 ==================
        
        // 独立的后台回环线程，负责在后台慢慢算 Scan Context 和 ICP
        std::thread loop_thread_;                

        // 互斥锁：防止主线程(写)和回环线程(读)同时访问关键帧数组导致程序崩溃
        std::mutex mtx_keyframes_;               

        // 任务队列(篮子)：主线程提取了新的关键帧后，把它的 ID(数组下标) 丢进这里，回环线程去取
        std::queue<int> new_keyframes_queue_;    

        // 原子布尔标志位：默认为 true。当节点准备析构退出时设为 false，安全切断后台 while 循环
        std::atomic<bool> loop_thread_running_{true}; 

        // 条件变量：让后台线程在没有新关键帧时“彻底沉睡(0% CPU)”，有新关键帧时“瞬间唤醒(0ms延迟)”
        std::condition_variable cv_keyframes_; 

        // ================== 🔥【2026.3.26 修改：后台回环线程相关变量 (工业级安全版)】结束 ==================



        // ================== 🔥【2026.3.26 新增：图优化线程(线程3)相关变量】起始 ==================
        
        // 线程3：专门负责在后台慢慢执行极其耗时的 ICP 精匹配和位姿图优化
        std::thread pgo_thread_;                                    
        
        // 互斥锁：专门保护“回环边队列”，防止线程2（写）和线程3（读）发生冲突
        std::mutex mtx_loop_edges_;                                 
        
        // 回环边队列（红区的篮子）：存放线程2刚刚检测到的回环对 <当前帧ID, 历史帧ID>
        std::queue<LoopCandidate> loop_edges_queue_;     
        
        // 条件变量：当没有回环任务时，让线程3彻底进入 0% CPU 的深度休眠状态
        std::condition_variable cv_loop_edges_;                     
        
        // ================== 🔥【2026.3.26 新增：图优化线程(线程3)相关变量】结束 ==================

        // ================== 🔥【2026.3.27 新增：ICP 精匹配相关成员变量】起始 ==================

        double icp_fitness_score_threshold_ = 0.3; // 存储 fitness score 阈值
            /*越小越好：代表两帧点云贴合得越紧密。
            完美对齐（同一地点、几乎无噪声） → 通常 < 0.05
            良好回环 → 0.05 ~ 0.25
            勉强可接受 → 0.25 ~ 0.4
            假回环或完全不匹配 → > 0.5~1.0 */

        int    icp_max_iterations_ = 100;          // 存储最大迭代次数
        double icp_max_corr_dist_ = 3.0;           // 存储最大搜索距离
        double overlap_ratio_threshold_ = 0.2;   // 重叠率阈值
        double overlap_search_radius_ = 1.0;      // 近邻搜索半径（米）

        std::mutex mtx_loop_edges_history_;        // 保护诊断历史容器的互斥锁
        std::vector<LoopEdge> loop_edges_history_; // 诊断历史：记录每一次 ICP 的完整信息，供论文实验使用
        // ================== 🔥【2026.3.27 新增：ICP 精匹配相关成员变量】结束 ==================

        // ================== 【scan-to-submap：成员变量】起始 ==================
        int    submap_search_num_ = 12;      // 历史帧前后各取多少帧拼 submap
        double submap_voxel_size_ = 0.2;     // submap 降采样体素大小（米）
        // ================== 【scan-to-submap：成员变量】结束 ==================


        // ================== 【KD-Tree 半径回环检测：成员变量】起始 ==================
        double radius_search_dist_ = 10.0;      // 半径搜索距离（米）
        double radius_search_time_diff_ = 30.0;  // 最小时间间隔（秒），防止匹配到时序相邻帧
        int    radius_min_frame_gap_ = 50;       // 最小帧间距
        std::set<int> radius_loop_submitted_;     // 已经提交过的历史帧 ID，防止重复提交
        // ================== 【KD-Tree 半径回环检测：成员变量】结束 ==================
        
        // ================== 🔥 g2o 图优化相关成员变量 ==================
        int pgo_max_iterations_ = 30;



        // ================== 🔥【2026.3.30 新增：PGO→前端校正增量机制】起始 ==================
        // 当 PGO 优化完成后，线程3 会把最新帧的"优化前后位姿差"存到这里
        // 前端 timer_callback 在每次 EKF 更新后检查并应用
        std::mutex mtx_correction_;
        bool has_loop_correction_ = false;            // 是否有待应用的校正
        Eigen::Vector3d correction_pos_;               // 优化后的绝对位置
        Eigen::Matrix3d correction_rot_;               // 优化后的绝对旋转
        // ================== 🔥【2026.3.30 新增：PGO→前端校正增量机制】结束 ==================

        // ================== 🔥【重影修复：累积 PGO 校正量追踪】 ==================
        Eigen::Isometry3d accumulated_loop_correction_ = Eigen::Isometry3d::Identity();

        // ================== 🔥【彻底修复：独立的纯净 odom 链】起始 ==================
        // 关键设计：维护一条与 PGO 完全分离的 odom 位姿链
        // 推进规则：每帧 EKF 更新完成后，用"EKF state 的相对增量"推进 pure_odom
        // PGO 触发时：刷新 last_state_for_pure_odom_ 缓存，吸收 PGO jump，使下一帧增量计算干净
        // 这样：pure_odom 永远等于"如果没有 PGO 校正，FAST-LIO2 自己会算出的位姿"
        Eigen::Vector3d pure_odom_state_pos_ = Eigen::Vector3d::Zero();   // IMU 系下的纯 odom 位置
        Eigen::Quaterniond pure_odom_state_rot_ = Eigen::Quaterniond::Identity();  // IMU 系下的纯 odom 旋转
        Eigen::Vector3d last_state_pos_for_pure_odom_ = Eigen::Vector3d::Zero();   // 上一帧 EKF state 缓存
        Eigen::Quaterniond last_state_rot_for_pure_odom_ = Eigen::Quaterniond::Identity();
        bool pure_odom_initialized_ = false;
        // ================== 🔥【彻底修复：独立的纯净 odom 链】结束 ==================

        // ================== 🔥【P0修复1：PGO优化的实际目标帧ID】 ==================
        // 解决竞态条件：PGO期间前端新增keyframe导致keyframes.back()错位
        int correction_target_kf_id_ = -1;
        


        // ================== 🔥【Patchwork++ 地面分割：成员变量】起始 ==================
        // 真实 API 是 patchwork::PatchWorkpp，不是 PatchWorkpp<PointType> 模板类
        std::unique_ptr<patchwork::PatchWorkpp> patchworkpp_;
        bool ground_seg_enable_ = false;

        // 诊断统计
        int total_ground_points_removed_ = 0;
        int total_gs_frames_processed_ = 0;
        // ================== 🔥【Patchwork++ 地面分割：成员变量】结束 ==================




        
        // ================== 🔥【校正前端地图：逐帧存储结构体 + 容器】起始 ==================
        // 运行时，每一帧 EKF 更新完成后，将该帧的 body 系点云和 LiDAR 世界位姿存入此容器。
        // 退出时，save_corrected_frontend_map() 会用 PGO 优化后的关键帧位姿
        // 对每一帧执行"增量嫁接"校正，然后重新变换拼接成全局一致的前端地图。
        struct FrameForCorrectedMap
        {
            PointCloudXYZI::Ptr cloud_body;   // 该帧去畸变点云（LiDAR body 系）
            Eigen::Vector3d lidar_pos;         // 该帧 LiDAR 在世界系的位置（原始 EKF，含外参）
            Eigen::Matrix3d lidar_rot;         // 该帧 LiDAR 在世界系的旋转（原始 EKF，含外参）
            double time;                       // 时间戳
        };
        std::vector<FrameForCorrectedMap> all_frames_for_map_;
        double corrected_map_voxel_size_ = 0.1;  // 逐帧存储时的降采样体素（米），越小越密、越占内存
        // ================== 🔥【校正前端地图：逐帧存储结构体 + 容器】结束 ==================


    // =======================================================



};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}