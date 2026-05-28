// ================== RTK 真值节点（对齐 FAST-LIO2 评估专用版 v2）==================
// 用途：输出 RTK 轨迹，配合 evo_ape --align 评估 FAST-LIO2 精度
//
// 重要说明：
//   1. 本节点输出的是 ENU 系下的 GNSS 天线位置轨迹
//   2. FAST-LIO2 输出的是 IMU 初始系下的 IMU 中心位置轨迹
//   3. 两者存在的差异：
//        - 原点不同（GPS 第一次定位点 vs IMU 初始化点）
//        - 姿态不同（ENU 固定朝向 vs IMU 初始朝向）
//        - 杆臂差（GNSS 天线 vs IMU 中心，通常几十 cm）
//        - 时间起点不同（GPS 和 IMU 初始化不同步）
//   4. 这些差异都通过 evo_ape --align（SE(3) 对齐）解决，无需代码处理
//   5. z 轴一致（都朝上），evo 只需估计 xy 平面的旋转 + 三维平移
//
// 评估命令：
//   # TUM 格式（推荐，evo 原生支持）
//   evo_ape tum gnss_tum.txt fastlio_tum.txt --align -vp
//   evo_traj tum gnss_tum.txt fastlio_tum.txt --align -p
//
//   # KITTI 格式
//   evo_ape kitti gnss_path.kitti fastlio_path.kitti --align -vp
//
// 输出文件：
//   ~/fastlio2/src/fast_lio/Lio_pose/gnss_path.kitti    <- KITTI 格式（主要交付）
//   ~/fastlio2/src/fast_lio/Lio_pose/gnss_tum.txt       <- TUM 格式（推荐评估用）
//   ~/fastlio2/src/fast_lio/Lio_pose/gnss_info.txt      <- 元信息（原点、采样数等）

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <deque>
#include <mutex>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <cstdlib>  // getenv

#include <rclcpp/rclcpp.hpp>
#include "gpsTools.hpp"
#include "utility.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

class GNSSOdom : public ParamServer {
 public:
  GNSSOdom(const rclcpp::NodeOptions & options)
  : ParamServer("lio_sam_gnss_odom", options) {

    // ============================================================
    // 参数读取
    // ============================================================
    this->get_parameter_or("gpsTopic",             gpsTopic_,             std::string("/gps/fix"));
    this->get_parameter_or("imu_topic",            imuTopic_,             std::string("/imu/data_raw"));

    // GPS 质量门槛：
    //   0 = STATUS_NO_FIX    → 拒收
    //   1 = STATUS_FIX       → 普通单点定位（精度 2~10m）
    //   2 = STATUS_SBAS_FIX  → SBAS 增强（亚米级）
    //   4 = STATUS_GBAS_FIX  → 地基增强/RTK 浮点（分米级）
    //   推荐：高精度建图评估用 ≥2；普通评估用 ≥1
    this->get_parameter_or("gps_status_threshold", gps_status_threshold_, 1);

    this->get_parameter_or("time_sync_tolerance",  time_sync_tolerance_,  0.05);

    // Yaw 来源: "imu" = IMU 自带姿态（需 9 轴 IMU 或有融合算法）
    //          "gps" = GPS 位移推算（低速不可靠，但适合无磁力计的 IMU）
    //          "none" = 不填 yaw（只看 XYZ 平移，也能评估平移精度）
    this->get_parameter_or("yaw_source",           yaw_source_,           std::string("gps"));

    this->get_parameter_or("gps_yaw_min_distance", gps_yaw_min_dist_,     0.5);

    // Path 长度限制（防止 RViz 卡顿）
    this->get_parameter_or("path_max_size",        path_max_size_,        10000);

    // 输出文件目录（按需求改为 Lio_pose）
    std::string default_save_dir = "/home/und/fastlio2/src/fast_lio/Lio_pose";
    this->get_parameter_or("save_dir", save_dir_, default_save_dir);

    auto reliable_qos = rclcpp::QoS(10).reliable();

    gpsSub = create_subscription<sensor_msgs::msg::NavSatFix>(
        gpsTopic_, reliable_qos,
        std::bind(&GNSSOdom::GNSSCB, this, std::placeholders::_1));

    imuSub = create_subscription<sensor_msgs::msg::Imu>(
        imuTopic_, reliable_qos,
        std::bind(&GNSSOdom::IMUCB, this, std::placeholders::_1));

    gpsOdomPub   = create_publisher<nav_msgs::msg::Odometry>("/gps_odom", 100);
    fusedPathPub = create_publisher<nav_msgs::msg::Path>("/gps_path", 100);

    // ============================================================
    // 输出文件初始化
    // ============================================================
    std::filesystem::create_directories(save_dir_);

    // KITTI 格式 (3x4 变换矩阵)
    kitti_file_.open(save_dir_ + "/gnss_path.kitti", std::ios::out);
    if (kitti_file_.is_open()) {
      kitti_file_ << std::fixed << std::setprecision(9);
      RCLCPP_INFO(this->get_logger(), "KITTI: %s/gnss_path.kitti", save_dir_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open KITTI file at %s", save_dir_.c_str());
    }

    // TUM 格式 (timestamp tx ty tz qx qy qz qw) —— evo 最佳
    tum_file_.open(save_dir_ + "/gnss_path.tum", std::ios::out);
    if (tum_file_.is_open()) {
      tum_file_ << std::fixed << std::setprecision(9);
      RCLCPP_INFO(this->get_logger(), "TUM:   %s/gnss_pathtum", save_dir_.c_str());
    }

    // 元信息文件（记录原点、参数、统计）
    info_file_.open(save_dir_ + "/gnss_info.txt", std::ios::out);

    RCLCPP_INFO(this->get_logger(),
      "=============================================");
    RCLCPP_INFO(this->get_logger(),
      "RTK Ground Truth Node Started");
    RCLCPP_INFO(this->get_logger(),
      "  GPS topic        : %s", gpsTopic_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "  IMU topic        : %s", imuTopic_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "  GPS status >=    : %d", gps_status_threshold_);
    RCLCPP_INFO(this->get_logger(),
      "  Yaw source       : %s", yaw_source_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "  Save dir         : %s", save_dir_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "=============================================");
  }

  ~GNSSOdom() {
    // 析构时写入元信息，方便后续分析
    if (info_file_.is_open() && initXyz_) {
      info_file_ << std::fixed << std::setprecision(9);
      info_file_ << "# GNSS RTK Ground Truth Info\n";
      info_file_ << "origin_lat: " << gtools.lla_origin_(0) << "\n";
      info_file_ << "origin_lon: " << gtools.lla_origin_(1) << "\n";
      info_file_ << "origin_alt: " << gtools.lla_origin_(2) << "\n";
      info_file_ << "yaw_source: " << yaw_source_ << "\n";
      info_file_ << "total_samples: " << total_samples_ << "\n";
      info_file_ << "rejected_low_status: " << rejected_low_status_ << "\n";
      info_file_ << "rejected_nan: " << rejected_nan_ << "\n";
      info_file_ << "start_time: " << start_time_ << "\n";
      info_file_ << "end_time: "   << last_time_ << "\n";
      info_file_ << "duration_s: " << (last_time_ - start_time_) << "\n";
      info_file_.close();
    }
    if (kitti_file_.is_open()) kitti_file_.close();
    if (tum_file_.is_open())   tum_file_.close();
    RCLCPP_INFO(this->get_logger(),
      "RTK files saved | samples=%d, duration=%.1fs",
      total_samples_, last_time_ - start_time_);
  }

 private:
  // ============================================================
  // IMU 回调（仅缓存，用于 GPS 时刻姿态查询）
  // ============================================================
  void IMUCB(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    std::lock_guard<std::mutex> lock(mutexLock_);
    imuBuf_.push_back(msg);
    double now = rclcpp::Time(msg->header.stamp).seconds();
    while (!imuBuf_.empty() &&
           rclcpp::Time(imuBuf_.front()->header.stamp).seconds() < now - 2.0) {
      imuBuf_.pop_front();
    }
  }

  sensor_msgs::msg::Imu::ConstSharedPtr findClosestIMU(double target_time) {
    std::lock_guard<std::mutex> lock(mutexLock_);
    sensor_msgs::msg::Imu::ConstSharedPtr best = nullptr;
    double min_diff = 1e9;
    for (const auto &imu : imuBuf_) {
      double dt = std::abs(rclcpp::Time(imu->header.stamp).seconds() - target_time);
      if (dt < min_diff) {
        min_diff = dt;
        best = imu;
      }
    }
    return (best && min_diff <= time_sync_tolerance_) ? best : nullptr;
  }

  // 检查 IMU orientation 是否有效（全零 quaternion = 没填）
  bool isImuOrientationValid(const sensor_msgs::msg::Imu::ConstSharedPtr &imu) {
    if (!imu) return false;
    const auto &q = imu->orientation;
    // 全零 = 没填；(0,0,0,1) 是单位四元数，但如果 IMU 一直给这个值也说明没计算
    double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (norm < 0.9 || norm > 1.1) return false;
    // 协方差第一项 < 0 表示未知（ROS 约定）
    if (imu->orientation_covariance[0] < 0) return false;
    return true;
  }

  // ============================================================
  // GPS 回调（核心）
  // ============================================================
  void GNSSCB(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg) {
    // ---- 1. 有效性检查 ----
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude)) {
      rejected_nan_++;
      return;
    }

    if (msg->status.status < gps_status_threshold_) {
      rejected_low_status_++;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "GPS status too low (%d < %d), rejected=%d",
        msg->status.status, gps_status_threshold_, rejected_low_status_);
      return;
    }

    Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
    double gps_time = rclcpp::Time(msg->header.stamp).seconds();

    // ---- 2. 原点初始化 ----
    if (!initXyz_) {
      gtools.lla_origin_ = lla;
      initXyz_ = true;
      start_time_ = gps_time;
      RCLCPP_INFO(this->get_logger(),
        "=== GPS Origin set: (%.8f, %.8f, %.3f) at t=%.3f ===",
        lla(0), lla(1), lla(2), gps_time);
      // 原点那一刻的位置就是 (0,0,0)，要记录下来
      // 否则第一个 GPS 样本不会进入评估轨迹
      Eigen::Vector3d origin_enu(0, 0, 0);
      prevPos_enu_ = origin_enu;
      WriteSample(gps_time, origin_enu, 0.0, 0.0, 0.0, msg);
      return;
    }

    // ---- 3. LLA → ENU ----
    Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
    Eigen::Vector3d enu  = gtools.ECEF2ENU(ecef);

    // 跳变保护：相邻两帧移动 > 50m 视为异常（假设 1Hz GPS + 50m/s = 180km/h 明显跳变）
    if (total_samples_ > 0) {
      double jump = (enu - prevPos_enu_).norm();
      if (jump > 50.0) {
        RCLCPP_WARN(this->get_logger(),
          "GPS jump detected: %.2fm, skipping this sample", jump);
        return;
      }
    }

    // ---- 4. Yaw 计算 ----
    double roll = 0.0, pitch = 0.0, yaw_out = 0.0;

    auto synced_imu = findClosestIMU(gps_time);
    bool imu_rpy_valid = isImuOrientationValid(synced_imu);

    // 从 IMU 取 roll/pitch（无论 yaw_source 是什么，RP 通常可信）
    if (imu_rpy_valid) {
      tf2::Quaternion q_imu;
      tf2::fromMsg(synced_imu->orientation, q_imu);
      double tmp_yaw;
      tf2::Matrix3x3(q_imu).getRPY(roll, pitch, tmp_yaw);

      if (yaw_source_ == "imu") {
        yaw_out = tmp_yaw;
      }
    }

    if (yaw_source_ == "gps") {
      // GPS 位移推算 yaw
      double dE = enu.x() - prevPos_enu_.x();
      double dN = enu.y() - prevPos_enu_.y();
      double distance = std::sqrt(dE * dE + dN * dN);

      if (distance > gps_yaw_min_dist_) {
        yaw_gps_ = std::atan2(dE, dN);  // ENU heading: 0=North, East=+90°
        yaw_initialized_ = true;
      }

      yaw_out = yaw_initialized_ ? yaw_gps_ : 0.0;
    } else if (yaw_source_ == "none") {
      yaw_out = 0.0;
      roll = 0.0;
      pitch = 0.0;
    }
    // yaw_source_ == "imu" 已在上面 imu_rpy_valid 分支处理

    // ---- 5. 保存 + 发布 ----
    WriteSample(gps_time, enu, roll, pitch, yaw_out, msg);
    prevPos_enu_ = enu;
  }

  // ============================================================
  // 统一的轨迹保存 + 发布逻辑
  // ============================================================
  void WriteSample(double gps_time,
                   const Eigen::Vector3d &enu,
                   double roll, double pitch, double yaw,
                   const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg)
  {
    total_samples_++;
    last_time_ = gps_time;

    // 构造四元数
    tf2::Quaternion q_final;
    q_final.setRPY(roll, pitch, yaw);

    // ---- 保存到文件 ----
    SaveKitti(enu, roll, pitch, yaw);
    SaveTUM(gps_time, enu, q_final);

    // ---- 发布 Odometry ----
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = msg->header.stamp;
    odom_msg.header.frame_id = "gps_odom";
    odom_msg.child_frame_id  = "gps_base";
    odom_msg.pose.pose.position.x = enu(0);
    odom_msg.pose.pose.position.y = enu(1);
    odom_msg.pose.pose.position.z = enu(2);
    odom_msg.pose.pose.orientation = tf2::toMsg(q_final);

    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      odom_msg.pose.covariance[0]  = msg->position_covariance[0];
      odom_msg.pose.covariance[7]  = msg->position_covariance[4];
      odom_msg.pose.covariance[14] = msg->position_covariance[8];
      odom_msg.pose.covariance[21] = 0.1;
      odom_msg.pose.covariance[28] = 0.1;
      odom_msg.pose.covariance[35] = 1.0;
    }
    gpsOdomPub->publish(odom_msg);

    // ---- 发布 Path（带长度限制）----
    rospath_.header.frame_id = "gps_odom";
    rospath_.header.stamp    = msg->header.stamp;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = rospath_.header;
    pose_stamped.pose   = odom_msg.pose.pose;
    rospath_.poses.push_back(pose_stamped);
    if ((int)rospath_.poses.size() > path_max_size_) {
      // 保留最后 path_max_size_ 个
      rospath_.poses.erase(
        rospath_.poses.begin(),
        rospath_.poses.begin() + (rospath_.poses.size() - path_max_size_));
    }
    fusedPathPub->publish(rospath_);

    // ---- 节流日志 ----
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "#%d | LLA=(%.7f,%.7f,%.2f) | ENU=(%.2f,%.2f,%.2f) | yaw=%.1f° | status=%d",
      total_samples_,
      msg->latitude, msg->longitude, msg->altitude,
      enu(0), enu(1), enu(2),
      yaw * 180.0 / M_PI, msg->status.status);
  }

  // ---- KITTI 格式 (3x4 旋转矩阵 + 平移，单行) ----
  void SaveKitti(const Eigen::Vector3d &pos, double roll, double pitch, double yaw) {
    if (!kitti_file_.is_open()) return;

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());

    kitti_file_ << R(0,0) << " " << R(0,1) << " " << R(0,2) << " " << pos.x() << " "
                << R(1,0) << " " << R(1,1) << " " << R(1,2) << " " << pos.y() << " "
                << R(2,0) << " " << R(2,1) << " " << R(2,2) << " " << pos.z() << "\n";
    kitti_file_.flush();  // 防止程序异常退出丢数据
  }

  // ---- TUM 格式 (timestamp tx ty tz qx qy qz qw) ----
  void SaveTUM(double timestamp, const Eigen::Vector3d &pos, const tf2::Quaternion &q) {
    if (!tum_file_.is_open()) return;
    tum_file_ << timestamp << " "
              << pos.x() << " " << pos.y() << " " << pos.z() << " "
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << "\n";
    tum_file_.flush();
  }

  // ---- 成员变量 ----
  GpsTools gtools;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gpsOdomPub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fusedPathPub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;

  std::ofstream kitti_file_;
  std::ofstream tum_file_;
  std::ofstream info_file_;
  std::mutex mutexLock_;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf_;

  std::string gpsTopic_, imuTopic_;
  std::string yaw_source_;
  std::string save_dir_;
  int gps_status_threshold_;
  double time_sync_tolerance_;
  double gps_yaw_min_dist_;
  int path_max_size_;

  bool initXyz_ = false;
  bool yaw_initialized_ = false;

  Eigen::Vector3d prevPos_enu_ = Eigen::Vector3d::Zero();
  double yaw_gps_ = 0.0;

  // 统计
  int total_samples_ = 0;
  int rejected_nan_ = 0;
  int rejected_low_status_ = 0;
  double start_time_ = 0.0;
  double last_time_ = 0.0;

  nav_msgs::msg::Path rospath_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<GNSSOdom>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
