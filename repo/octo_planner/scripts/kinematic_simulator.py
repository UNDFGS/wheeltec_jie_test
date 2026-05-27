#!/usr/bin/env python3
"""
Ackermann 轮式机器人运动学模拟器 (自行车模型)
- 订阅 /cmd_vel (geometry_msgs/Twist): linear.x = 前进速度, angular.z = 转向角速度
- 前轮转向, 后轮驱动
- 有最小转弯半径, 不能原地旋转, 不能横向移动
- 发布 TF: odom → base_link
- 发布 nav_msgs/Odometry
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class KinematicSimulator(Node):
    def __init__(self):
        super().__init__("kinematic_simulator")

        self.declare_parameter("wheel_base", 0.527)          # 前后轴距 (m)
        self.declare_parameter("track_width", 0.40)          # 左右轮距 (m)
        self.declare_parameter("max_steering_angle", 0.785)  # 最大转向角 (rad, ~45deg)
        self.declare_parameter("min_creep_speed", 0.12)      # 阿克曼转向最小蠕行速度 (m/s)
        self.declare_parameter("publish_rate", 50.0)         # Hz
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.wheel_base = self.get_parameter("wheel_base").value
        self.track_width = self.get_parameter("track_width").value
        self.max_steering = self.get_parameter("max_steering_angle").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # 初始位姿: x, y, yaw (可通过参数设定)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw_deg", 0.0)
        self.x = self.get_parameter("initial_x").value
        self.y = self.get_parameter("initial_y").value
        self.yaw = math.radians(self.get_parameter("initial_yaw_deg").value)
        self.steering_angle = 0.0  # 当前转向角

        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.on_cmd_vel, 10)

        from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, PoseStamped
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

        # 订阅 /initialpose 以支持 RViz "2D Pose Estimate"
        self.init_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initial_pose, 10)

        # QoS 匹配 jie_path_node 的 TRANSIENT_LOCAL + RELIABLE
        planner_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        # 发布导航起点（与初始位姿联动）
        self.start_pub = self.create_publisher(
            PointStamped, "/start_point", planner_qos)

        # QoS 桥接：RViz /goal_pose → jie_path_node 兼容的 /goal_point
        # RViz 2D Goal Pose 用 BEST_EFFORT QoS，必须匹配才能收到
        rviz_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.on_goal_pose, rviz_qos)
        self.goal_pub = self.create_publisher(
            PointStamped, "/goal_point", planner_qos)

        self.odom_pub = self.create_publisher(Odometry, "/odometry", 10)

        self.last_time = self.get_clock().now()
        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info(
            f"Ackermann kinematic_simulator started. "
            f"wheel_base={self.wheel_base} max_steering={self.max_steering:.2f}rad "
            f"rate={self.publish_rate}Hz "
            f"initial=(x={self.x:.1f}, y={self.y:.1f}, yaw={math.degrees(self.yaw):.0f}deg)"
        )

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def on_goal_pose(self, msg):
        """RViz '2D Goal Pose' → 先更新起点为机器人当前位置，再转发目标"""
        from geometry_msgs.msg import PointStamped

        # 1. 先用机器人当前位置更新起点
        start = PointStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.point.x = self.x
        start.point.y = self.y
        start.point.z = 0.0
        self.start_pub.publish(start)

        # 2. 转发目标
        pt = PointStamped()
        pt.header = msg.header
        pt.point.x = msg.pose.position.x
        pt.point.y = msg.pose.position.y
        pt.point.z = msg.pose.position.z
        self.goal_pub.publish(pt)

        self.get_logger().info(
            f"Start→({start.point.x:.1f},{start.point.y:.1f}) "
            f"Goal→({pt.point.x:.1f},{pt.point.y:.1f})"
        )

    def on_initial_pose(self, msg):
        """RViz '2D Pose Estimate' 回调 — 设置机器人位姿 + 同步设为导航起点"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # 从 quaternion 提取 yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.steering_angle = 0.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.get_logger().info(
            f"Initial pose set: x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}deg"
        )
        # 同时发布为导航起点
        from geometry_msgs.msg import PointStamped
        start = PointStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.point.x = self.x
        start.point.y = self.y
        start.point.z = 0.0
        self.start_pub.publish(start)

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / self.publish_rate
        self.last_time = now

        vx = self.cmd_vx
        wz = self.cmd_wz

        if dt > 0.0:
            # ----- 阿克曼转向模型 (支持倒车) -----
            # 阿克曼不能原地转向 — 必须有纵向速度才能转弯
            # 倒车时转向同样有效（自行车模型在 vx<0 时照样工作）
            min_speed = 0.15  # 最小纵向速度 (m/s)
            if abs(vx) < 0.01 and abs(wz) > 0.01:
                # 原地转向 → 注入最小前进速度
                vx = min_speed * (-1.0 if wz < 0 else 1.0)
            elif 0 < abs(vx) < min_speed and abs(wz) > 0.01:
                # 太慢但需要转向 → 保持方向，提升速度
                vx = min_speed * (1.0 if vx > 0 else -1.0)
            elif abs(vx) < 0.001:
                vx = 0.0

            if abs(vx) > 0.001:
                desired_steering = math.atan2(wz * self.wheel_base, vx)
                self.steering_angle = max(-self.max_steering,
                                          min(self.max_steering, desired_steering))
                actual_wz = vx * math.tan(self.steering_angle) / self.wheel_base
            else:
                actual_wz = 0.0
                self.steering_angle = 0.0

            # 积分位姿
            self.x += vx * math.cos(self.yaw) * dt
            self.y += vx * math.sin(self.yaw) * dt
            self.yaw += actual_wz * dt
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # 发布 TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = self._yaw_to_quat(self.yaw)
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0  # Ackermann 无横向速度
        odom.twist.twist.angular.z = self.cmd_wz  # 记录原始指令角速度
        self.odom_pub.publish(odom)

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        half = yaw * 0.5
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q


def main():
    rclpy.init()
    node = KinematicSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
