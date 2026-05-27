#!/usr/bin/env python3
"""
EGO-Planner ↔ jie_3d_nav 桥接节点

1. A* 全局路径 → waypoint 目标点 → EGO-Planner
2. OctoMap → 采样点云 → /grid_map/cloud (EGO 感知输入)
3. TF 里程计 → /grid_map/odom (EGO 定位输入)
4. EGO B-spline 轨迹 → nav_msgs/Path → ackermann_tracker
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import struct


class EGOBridge(Node):
    def __init__(self):
        super().__init__("ego_bridge")

        # ---- 参数 ----
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_sim_cloud", True)        # True=OctoMap模拟, False=雷达
        self.declare_parameter("lidar_topic", "/livox/lidar")  # 实机雷达话题
        self.declare_parameter("cloud_publish_rate", 5.0)
        self.declare_parameter("cloud_sample_count", 5000)
        self.declare_parameter("cloud_range", 30.0)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- 从 Occupied Markers 获取 OctoMap 点云 ----
        from visualization_msgs.msg import Marker
        self.occupied_points = []  # (x, y, z) tuples
        self.marker_sub = self.create_subscription(
            Marker, "/octomap_occupied_markers", self.on_marker,
            rclpy.qos.QoSProfile(depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

        # ---- A* 全局路径 → waypoint ----
        self.global_path = []
        # VOLATILE: 不接受缓存的旧路径
        self.path_sub = self.create_subscription(
            Path, "/planned_path", self.on_global_path,
            rclpy.qos.QoSProfile(depth=1,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.waypoint_pub = self.create_publisher(
            PoseStamped, "/waypoint", 10)
        # 自动触发导航启动
        from std_msgs.msg import Bool
        self.start_nav_pub = self.create_publisher(
            Bool, "/start_navigation", 10)
        self._init_time = self.get_clock().now()

        # ---- 点云输入: 仿真用 OctoMap，实机用雷达 ----
        self.use_sim = self.get_parameter("use_sim_cloud").value
        if not self.use_sim:
            lidar_topic = self.get_parameter("lidar_topic").value
            self.lidar_sub = self.create_subscription(
                PointCloud2, lidar_topic, self.on_lidar_cloud, 10)
            self.get_logger().info(f"Using real LiDAR: {lidar_topic} → /grid_map/cloud")
        else:
            self.get_logger().info("Using simulated cloud from OctoMap markers")

        self.cloud_pub = self.create_publisher(
            PointCloud2, "/grid_map/cloud", 10)

        # 实时雷达点云缓存
        self.latest_lidar_cloud = None

        # ---- TF → Odometry ----
        self.odom_pub = self.create_publisher(
            Odometry, "/grid_map/odom", 10)

        # ---- EGO B-spline 轨迹订阅 ----
        # EGO publishes custom Bspline msg — we subscribe to its viz path instead
        from nav_msgs.msg import Path as NavPath
        self.ego_path_pub = self.create_publisher(
            NavPath, "/ego_local_path",
            rclpy.qos.QoSProfile(depth=1,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

        # Also try subscribing to EGO's Bspline directly
        try:
            from traj_utils.msg import Bspline
            self.bspline_sub = self.create_subscription(
                Bspline, "/planning/bspline", self.on_bspline, 10)
            self.get_logger().info("Subscribed to EGO Bspline topic")
        except Exception:
            self.get_logger().warn("Cannot import traj_utils.msg.Bspline, will use other methods")

        # 定时器
        dt = 1.0 / self.get_parameter("cloud_publish_rate").value
        self.cloud_timer = self.create_timer(dt, self.publish_cloud)
        self.odom_timer = self.create_timer(0.05, self.publish_odom)  # 20Hz

        self.get_logger().info("EGO Bridge started")

    # ---- 从 Marker 提取 OctoMap 体素 ----
    def on_marker(self, msg):
        pts = [(p.x, p.y, p.z) for p in msg.points]
        if pts:
            self.occupied_points = pts

    # ---- 实机雷达点云 ----
    def on_lidar_cloud(self, msg):
        self.latest_lidar_cloud = msg

    # ---- 全局路径 → waypoint ----
    def on_global_path(self, msg: Path):
        # 启动后 2 秒内忽略（安全守卫）
        if (self.get_clock().now() - self._init_time).nanoseconds < 2e9:
            return
        if msg.poses:
            last = msg.poses[-1]
            wp = PoseStamped()
            wp.header.frame_id = self.get_parameter("map_frame").value
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.pose = last.pose
            self.waypoint_pub.publish(wp)

            # 触发 AckermannTracker 开始导航
            from std_msgs.msg import Bool
            self.start_nav_pub.publish(Bool(data=True))

            self.get_logger().info(
                f"Waypoint sent to EGO: ({wp.pose.position.x:.1f}, "
                f"{wp.pose.position.y:.1f}, {wp.pose.position.z:.1f})"
                f" + start_navigation triggered",
                throttle_duration_sec=3.0
            )

    # ---- 点云发布 (仿真:OctoMap采样 / 实机:雷达直通) ----
    def publish_cloud(self):
        if not self.use_sim:
            # 实机模式: 转发雷达点云
            if self.latest_lidar_cloud is not None:
                self.latest_lidar_cloud.header.stamp = self.get_clock().now().to_msg()
                self.cloud_pub.publish(self.latest_lidar_cloud)
            return

        # 仿真模式: OctoMap 采样
        if not self.occupied_points:
            return

        try:
            pose = self._get_robot_pose()
            if pose is None:
                return
            rx, ry, rz, ryaw = pose
        except Exception:
            return

        rng = self.get_parameter("cloud_range").value
        max_n = self.get_parameter("cloud_sample_count").value

        # 从全局已占用体素中截取机器人周围的子集
        local = []
        for (x, y, z) in self.occupied_points:
            if abs(x - rx) < rng and abs(y - ry) < rng and abs(z - rz) < rng:
                local.append((x, y, z))
            if len(local) >= max_n:
                break

        if not local:
            return

        cloud = PointCloud2()
        cloud.header.frame_id = self.get_parameter("map_frame").value
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = len(local)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_bigendian = False
        cloud.is_dense = True

        buf = b""
        for (x, y, z) in local:
            buf += struct.pack("fff", x, y, z)
        cloud.data = buf
        self.cloud_pub.publish(cloud)

    # ---- TF → Odometry (for EGO /grid_map/odom) ----
    def publish_odom(self):
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry, rz, ryaw = pose
        odom = Odometry()
        odom.header.frame_id = self.get_parameter("map_frame").value
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = self.get_parameter("base_frame").value
        odom.pose.pose.position.x = rx
        odom.pose.pose.position.y = ry
        odom.pose.pose.position.z = rz
        odom.pose.pose.orientation.z = math.sin(ryaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(ryaw / 2.0)
        self.odom_pub.publish(odom)

    # ---- EGO B-spline → nav_msgs/Path ----
    def on_bspline(self, msg):
        """Convert EGO Bspline control points to nav_msgs/Path for ackermann_tracker"""
        path = Path()
        path.header.frame_id = self.get_parameter("map_frame").value
        path.header.stamp = self.get_clock().now().to_msg()

        for pt in msg.pos_pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.position.z = pt.z
            path.poses.append(ps)

        self.ego_path_pub.publish(path)
        self.get_logger().debug(
            f"EGO path published: {len(path.poses)} ctrl pts",
            throttle_duration_sec=2.0
        )

    def _eval_bspline(self, msg, t):
        """评估 B-spline 在某时刻的位置"""
        # Uniform Bspline evaluation (simplified 3rd-order)
        cp = msg.control_points
        if len(cp) < 4:
            return Point()
        order = min(msg.order, len(cp) - 1)
        knot = msg.knots
        interval = t * (len(knot) - 2 * order - 1) + order
        i = max(order, min(int(interval), len(knot) - order - 1))

        # De Boor algorithm simplified
        if i - order >= 0 and i + 1 < len(cp):
            ratio = interval - i
            a = cp[i - 1] if i > 0 else cp[0]
            b = cp[i]
            c = cp[min(i + 1, len(cp) - 1)]
            # Quadratic blend
            x = (1 - ratio) * ((1 - ratio) * a.x + ratio * b.x) + \
                ratio * ((1 - ratio) * b.x + ratio * c.x)
            y = (1 - ratio) * ((1 - ratio) * a.y + ratio * b.y) + \
                ratio * ((1 - ratio) * b.y + ratio * c.y)
            z = (1 - ratio) * ((1 - ratio) * a.z + ratio * b.z) + \
                ratio * ((1 - ratio) * b.z + ratio * c.z)
            return Point(x=float(x), y=float(y), z=float(z))

        p0 = cp[max(0, min(i - 1, len(cp) - 1))]
        p1 = cp[max(0, min(i, len(cp) - 1))]
        ratio = max(0.0, min(1.0, interval - i))
        return Point(
            x=float(p0.x + (p1.x - p0.x) * ratio),
            y=float(p0.y + (p1.y - p0.y) * ratio),
            z=float(p0.z + (p1.z - p0.z) * ratio)
        )

    def _get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value,
                self.get_parameter("base_frame").value,
                rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return (x, y, z, math.atan2(siny, cosy))
        except Exception:
            return None


def main():
    rclpy.init()
    node = EGOBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
