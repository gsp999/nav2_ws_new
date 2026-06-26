"""单目标导航节点：接收目标坐标，导航到位并全程锁 yaw
ramp 区域的悬挂/限速由 ramp_zone_manager 独立处理，本节点不干预。

两种发送方式：
  1. /nav_goal (Float32MultiArray): data = [x, y, yaw]    ← 推荐
  2. /single_nav_goal (PoseStamped): yaw 嵌在 orientation 中
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Float32MultiArray
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    """yaw → orientation (z, w)"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(x, y, yaw):
    """(x, y, yaw) → PoseStamped"""
    p = PoseStamped()
    p.header.frame_id = "map"
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    qz, qw = yaw_to_quat(yaw)
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


class ThirdAreaSingle(Node):
    def __init__(self):
        super().__init__("third_area_single")
        self.declare_parameter("team", "red")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.yaw_pub = self.create_publisher(Float32, "/desired_yaw", 10)

        # 简洁版：(x, y, yaw) 直接输角度
        self.goal_sub = self.create_subscription(
            Float32MultiArray, "/nav_goal", self.nav_goal_cb, 10)

        # PoseStamped 版：兼容 RViz / move_base_simple
        self.pose_sub = self.create_subscription(
            PoseStamped, "/single_nav_goal", self.pose_goal_cb, 10)

        self.nav_ready = False

        # 用 timer 非阻塞等待 Nav2，不堵 spin
        self._wait_timer = self.create_timer(1.0, self._check_nav_ready)
        self.get_logger().info("ThirdAreaSingle: waiting for Nav2 (non-blocking)...")

    def _check_nav_ready(self):
        if self.nav_client.wait_for_server(timeout_sec=0.1):
            if not self.nav_ready:
                self.nav_ready = True
                self.get_logger().info("ThirdAreaSingle: Nav2 ready, listening on /nav_goal  [x, y, yaw]")
                self._wait_timer.cancel()

    # ---- /nav_goal: [x, y, yaw] ----
    def nav_goal_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().error(f"/nav_goal needs [x, y, yaw], got {len(msg.data)} values")
            return

        x, y, yaw = msg.data[0], msg.data[1], msg.data[2]
        self.get_logger().info(
            f"Goal: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}deg"
        )
        self._start_nav(x, y, yaw)

    # ---- /single_nav_goal: PoseStamped ----
    def pose_goal_cb(self, msg: PoseStamped):
        siny = 2.0 * (msg.pose.orientation.w * msg.pose.orientation.z)
        cosy = 1.0 - 2.0 * (msg.pose.orientation.z ** 2)
        yaw = math.atan2(siny, cosy)
        self.get_logger().info(
            f"Goal (Pose): ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), "
            f"yaw={math.degrees(yaw):.1f}deg"
        )
        self._start_nav(msg.pose.position.x, msg.pose.position.y, yaw)

    def _start_nav(self, x, y, yaw):
        if not self.nav_ready:
            self.get_logger().warn("Nav2 not ready yet — ignoring goal")
            return
        if self.nav_busy:
            self.get_logger().warn("Still navigating — ignoring new goal")
            return

        # 全程锁 yaw
        self.yaw_pub.publish(Float32(data=float(yaw)))
        self.get_logger().info(f"Yaw hold active: {math.degrees(yaw):.1f}deg")

        self.nav_busy = True
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.response_cb)

    def response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.yaw_pub.publish(Float32(data=999.0))
            self.nav_busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Navigation failed! status={result.status}")
        else:
            self.get_logger().info("Goal reached!")

        self.yaw_pub.publish(Float32(data=999.0))
        self.nav_busy = False
        self.get_logger().info("Yaw hold released — ready for next goal")


def main(args=None):
    rclpy.init(args=args)
    node = ThirdAreaSingle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
