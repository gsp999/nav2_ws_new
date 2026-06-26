"""监测位置，坡面升悬挂+最低速，yaw 覆盖控制"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, Float32, Int32


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RampZoneManager(Node):
    def __init__(self):
        super().__init__("ramp_zone_manager")

        # team 必须最先声明，影响 ramp Y 默认值
        self.declare_parameter("team", "red")
        team = self.get_parameter("team").value

        # Ramp Y 边界：根据队伍自动选默认值，也可通过 launch 参数覆盖
        if team == "blue":
            self.declare_parameter("ramp_y_min", 3.1)
            self.declare_parameter("ramp_y_max", 4.6)
        else:
            self.declare_parameter("ramp_y_min", -4.6)
            self.declare_parameter("ramp_y_max", -3.1)

        self.declare_parameter("ramp_x_min", 8.9)
        self.declare_parameter("ramp_x_max", 10.4)
        self.declare_parameter("min_ramp_speed", 0.25)
        self.declare_parameter("suspension_ramp", 75.0)
        self.declare_parameter("suspension_flat", 30.0)
        self.declare_parameter("yaw_kp", 2.0)
        self.declare_parameter("yaw_max_vel", 2.0)

        self.team = team
        self.ramp_x_min = self.get_parameter("ramp_x_min").value
        self.ramp_x_max = self.get_parameter("ramp_x_max").value
        self.ramp_y_min = self.get_parameter("ramp_y_min").value
        self.ramp_y_max = self.get_parameter("ramp_y_max").value
        self.min_ramp_speed = self.get_parameter("min_ramp_speed").value
        self.suspension_ramp = self.get_parameter("suspension_ramp").value
        self.suspension_flat = self.get_parameter("suspension_flat").value
        self.yaw_kp = self.get_parameter("yaw_kp").value
        self.yaw_max_vel = self.get_parameter("yaw_max_vel").value

        self.in_ramp = False
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.desired_yaw = None  # None = 不覆盖，让 Nav2 控制

        # 订阅
        self.pose_sub = self.create_subscription(
            PoseStamped, "/odin1/relocation", self.pose_cb, 10)
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_cb, 10)
        self.yaw_sub = self.create_subscription(
            Float32, "/desired_yaw", self.yaw_target_cb, 10)

        # 发布
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_adjusted", 10)
        self.suspension_pub = self.create_publisher(
            Float32MultiArray, "/t0x0102_action", 10)
        self.target_state_pub = self.create_publisher(Int32, "/targetstate", 10)

        self.get_logger().info(
            f"RampZoneManager [{self.team}]: yaw_kp={self.yaw_kp}, "
            f"ramp y=[{self.ramp_y_min},{self.ramp_y_max}]")

    def yaw_target_cb(self, msg: Float32):
        """设置期望 yaw（rad）。发 NaN 或 >900 禁用覆盖"""
        if math.isnan(msg.data) or msg.data > 900:
            if self.desired_yaw is not None:
                self.get_logger().info("Yaw override disabled")
            self.desired_yaw = None
        else:
            self.desired_yaw = msg.data
            self.get_logger().info(
                f"Yaw override: {math.degrees(self.desired_yaw):.0f} deg")

    def pose_cb(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_yaw = quat_to_yaw(msg.pose.orientation)

        was_in_ramp = self.in_ramp
        self.in_ramp = (
            self.ramp_x_min <= self.current_x <= self.ramp_x_max and
            self.ramp_y_min <= self.current_y <= self.ramp_y_max
        )
        if self.in_ramp and not was_in_ramp:
            self.get_logger().info("Enter ramp -> raise suspension")
            h = self.suspension_ramp
            self.suspension_pub.publish(Float32MultiArray(data=[h, h, h, h]))
            self.target_state_pub.publish(Int32(data=-1))
        elif not self.in_ramp and was_in_ramp:
            self.get_logger().info("Exit ramp -> lower suspension")
            h = self.suspension_flat
            self.suspension_pub.publish(Float32MultiArray(data=[h, h, h, h]))
            self.target_state_pub.publish(Int32(data=-1))

    def cmd_cb(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.angular.z = msg.angular.z

        # Yaw 覆盖：只在 ramp 区域内锁 yaw，正常路段交给 Nav2
        if self.desired_yaw is not None and self.in_ramp:
            yaw_error = normalize_angle(self.desired_yaw - self.current_yaw)
            wz = self.yaw_kp * yaw_error
            wz = max(-self.yaw_max_vel, min(self.yaw_max_vel, wz))
            out.angular.z = wz

        # Ramp 最低速
        if self.in_ramp:
            speed = math.sqrt(out.linear.x**2 + out.linear.y**2)
            if 0.01 < speed < self.min_ramp_speed:
                scale = self.min_ramp_speed / speed
                out.linear.x *= scale
                out.linear.y *= scale

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RampZoneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
