#!/usr/bin/env python3
"""
3D 阿克曼路径跟踪器 (Stanley Controller)
替换 d1_controller，专为阿克曼底盘设计

输入: /planned_path (nav_msgs/Path)
输出: /cmd_vel (geometry_msgs/Twist)

Stanley 控制律:
  delta = heading_error + atan2(k_cross * cross_track_error, speed)
  speed = min(max_speed, lookahead_speed)  弯道自动减速
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
import numpy as np


class AckermannTracker(Node):
    def __init__(self):
        super().__init__("ackermann_tracker")

        # ---- 阿克曼车辆参数 ----
        self.declare_parameter("wheel_base", 0.527)
        self.declare_parameter("max_steering_angle", 0.785)  # 45deg
        self.declare_parameter("min_turning_radius", 0.53)    # 最小转弯半径
        self.declare_parameter("max_speed", 0.60)
        self.declare_parameter("min_speed", 0.10)
        self.declare_parameter("k_cross_track", 0.8)          # 横向误差增益
        self.declare_parameter("k_heading", 1.0)               # 航向误差增益
        self.declare_parameter("lookahead_dist", 1.5)          # 前瞻距离
        self.declare_parameter("goal_tolerance", 0.3)          # 终点容差
        self.declare_parameter("replan_deviation", 1.5)        # 偏离路径多远触发重规划
        self.declare_parameter("control_rate", 20.0)           # Hz
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 状态 ----
        self.path: list[Point] = []
        self.path_index = 0
        self.navigation_active = False

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 路径订阅: VOLATILE — 不接受缓存
        path_qos = rclpy.qos.QoSProfile(depth=1,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.path_sub = self.create_subscription(Path, "/ego_local_path", self.on_path, path_qos)
        self.global_path_sub = self.create_subscription(Path, "/planned_path", self.on_path, path_qos)

        # 起点发布器 (QoS 匹配 jie_path_node 的 TRANSIENT_LOCAL)
        from geometry_msgs.msg import PointStamped
        start_qos = rclpy.qos.QoSProfile(depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.start_pub = self.create_publisher(PointStamped, "/start_point", start_qos)

        # 导航控制: 仅通过 /start_navigation 和 /stop_navigation 控制
        self.start_sub = self.create_subscription(
            Bool, "/start_navigation", self.on_start_navigation, 10)
        self.stop_sub = self.create_subscription(
            Bool, "/stop_navigation", self.on_stop_navigation, 10)

        dt = 1.0 / self.get_parameter("control_rate").value
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info(
            f"AckermannTracker started. wheel_base={self.get_parameter('wheel_base').value:.3f} "
            f"max_steering={self.get_parameter('max_steering_angle').value:.2f}rad "
            f"lookahead={self.get_parameter('lookahead_dist').value:.1f}m"
        )

    def on_path(self, msg: Path):
        """存储路径，不自动启动——等待显式 /start_navigation"""
        if not msg.poses:
            return
        self.path = [p.pose.position for p in msg.poses]
        self.path_index = 0
        self.get_logger().info(
            f"Path stored: {len(self.path)} waypoints. "
            f"Send /start_navigation to begin."
        )

    def on_start_navigation(self, msg: Bool):
        """显式启动导航 — 用机器人当前位置更新全局起点"""
        if not msg.data:
            return
        if not self.path:
            self.get_logger().warn("No path available. Set start/goal first.")
            return
        if self.navigation_active:
            return

        # 用机器人当前位置更新 A* 起点
        pose = self.get_robot_pose()
        if pose:
            from geometry_msgs.msg import PointStamped
            pt = PointStamped()
            pt.header.frame_id = self.get_parameter("map_frame").value
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = pose[0], pose[1], pose[2]
            self.start_pub.publish(pt)

        self.navigation_active = True
        self.path_index = 0
        self.get_logger().info(f"Navigation started: {len(self.path)} waypoints.")

    def on_stop_navigation(self, msg: Bool):
        if msg.data:
            self.stop_navigation("User requested stop.")

    def stop_navigation(self, reason: str):
        self.navigation_active = False
        self.path.clear()
        self.publish_cmd(0.0, 0.0)
        self.get_logger().info(f"Navigation stopped: {reason}")

    def publish_cmd(self, vx: float, wz: float):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def get_robot_pose(self):
        """返回 (x, y, yaw) 或 None"""
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value,
                self.get_parameter("base_frame").value,
                rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            # quat → yaw
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            return (x, y, yaw)
        except Exception:
            return None

    def find_closest_index(self, x: float, y: float):
        """找到路径上最近点的索引"""
        if not self.path:
            return 0
        best_i = self.path_index
        best_d = float("inf")
        # 从当前索引往后搜索，允许回退 5 个点
        start = max(0, self.path_index - 5)
        for i in range(start, len(self.path)):
            dx = self.path[i].x - x
            dy = self.path[i].y - y
            d = dx * dx + dy * dy
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def find_lookahead_point(self, x: float, y: float, lookahead: float):
        """找到路径上前瞻距离处的目标点"""
        # 从最近点开始，累积距离直到超过 lookahead
        start = self.find_closest_index(x, y)
        cum_dist = 0.0
        for i in range(start, len(self.path) - 1):
            dx = self.path[i + 1].x - self.path[i].x
            dy = self.path[i + 1].y - self.path[i].y
            seg_len = math.sqrt(dx * dx + dy * dy)
            if cum_dist + seg_len >= lookahead:
                # 插值
                ratio = (lookahead - cum_dist) / seg_len
                px = self.path[i].x + dx * ratio
                py = self.path[i].y + dy * ratio
                pz = self.path[i].z + (self.path[i + 1].z - self.path[i].z) * ratio
                return Point(x=px, y=py, z=pz), i
            cum_dist += seg_len
        # 不够长 — 返回终点
        last = self.path[-1]
        return Point(x=last.x, y=last.y, z=last.z), len(self.path) - 1

    def compute_path_yaw(self, idx: int):
        """计算路径上某点的朝向"""
        if idx >= len(self.path) - 1:
            idx = len(self.path) - 2
        if idx < 0:
            return 0.0
        dx = self.path[idx + 1].x - self.path[idx].x
        dy = self.path[idx + 1].y - self.path[idx].y
        return math.atan2(dy, dx)

    def control_loop(self):
        if not self.navigation_active or not self.path:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # ---- 偏离检测 ----
        dev = self.get_parameter("replan_deviation").value
        closest_idx = self.find_closest_index(rx, ry)
        closest = self.path[closest_idx]
        dist_to_path = math.sqrt((rx - closest.x)**2 + (ry - closest.y)**2)
        self.path_index = closest_idx
        if dist_to_path > dev * 2.0:
            self.stop_navigation(f"Too far off path: {dist_to_path:.2f}m. Re-set start/goal.")
            return

        # ---- 找到目标终点 ----
        goal = self.path[-1]
        dist_to_goal = math.sqrt((rx - goal.x)**2 + (ry - goal.y)**2)
        if dist_to_goal < self.get_parameter("goal_tolerance").value:
            self.get_logger().info(
                f"Goal reached! dist={dist_to_goal:.3f}m "
                f"< tol={self.get_parameter('goal_tolerance').value:.3f}m"
            )
            self.navigation_active = False
            self.publish_cmd(0.0, 0.0)
            return

        # ---- Stanley 控制 (阿克曼修正版) ----
        lookahead = self.get_parameter("lookahead_dist").value
        target, target_idx = self.find_lookahead_point(rx, ry, lookahead)
        self.path_index = target_idx

        # 目标在 MAP 坐标系中的偏移
        dx_map = target.x - rx
        dy_map = target.y - ry

        # 旋转到机器人坐标系 (body frame: x=前, y=左)
        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)
        target_body_x = cos_yaw * dx_map + sin_yaw * dy_map
        target_body_y = -sin_yaw * dx_map + cos_yaw * dy_map

        # 目标方位角 (= heading error)
        heading_error = math.atan2(target_body_y, target_body_x)

        # 横向误差 (= cross-track, 机器人坐标系下的 y 分量)
        path_yaw = self.compute_path_yaw(target_idx)
        cross_track = target_body_y

        target_dist = math.sqrt(target_body_x**2 + target_body_y**2)

        # ---- 180° U-turn / 倒车处理 ----
        # 目标在后方时阿克曼可后退+转向，比纯前进 U-turn 更高效
        k_cross = self.get_parameter("k_cross_track").value
        k_head = self.get_parameter("k_heading").value
        max_speed = self.get_parameter("max_speed").value
        min_speed = self.get_parameter("min_speed").value

        reverse_mode = False
        if target_body_x < -0.5:
            # 目标明显在后方 → 倒车+转向
            reverse_mode = True
            # 翻转 target_body 让控制器"以为"目标在前方
            target_body_x = -target_body_x
            target_body_y = -target_body_y
            heading_error = math.atan2(target_body_y, target_body_x)
            # 倒车时用较低速度
            speed = min_speed * 1.5  # ~0.15 m/s
            k_head = 2.0
        elif target_body_x < 0.0:
            # 目标略偏后 → 最低速前进+大转向绕过去
            turn_factor = 0.12
            speed = max_speed * turn_factor
            speed = max(min_speed, speed)
            k_head = 2.0
        else:
            # 正常前进
            turn_factor = max(0.3, 1.0 - abs(heading_error) / 1.5)
            speed = max_speed * turn_factor
            speed = max(min_speed, speed)

        # Stanley 转向角
        steering = k_head * heading_error
        if speed > 0.01:
            steering += math.atan2(k_cross * cross_track, speed)

        # 倒车时翻转速度
        if reverse_mode:
            speed = -speed

        # 限幅
        max_steer = self.get_parameter("max_steering_angle").value
        steering = max(-max_steer, min(max_steer, steering))

        # 转向角 → 角速度 (自行车模型)
        if abs(speed) > 0.001:
            wz = speed * math.tan(steering) / self.get_parameter("wheel_base").value
        else:
            wz = 0.0

        self.publish_cmd(speed, wz)

        # 日志（每秒一次）
        self.get_logger().info(
            f"Track: pos=({rx:.2f},{ry:.2f}) target=({target.x:.2f},{target.y:.2f}) "
            f"d={target_dist:.2f}m h_err={math.degrees(heading_error):.0f}deg "
            f"ct_err={cross_track:.2f}m steer={math.degrees(steering):.0f}deg "
            f"cmd=(v={speed:.2f}, wz={wz:.2f})",
            throttle_duration_sec=1.0
        )


def main():
    rclpy.init()
    node = AckermannTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
