import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped


class CenterCalculator(Node):

    def __init__(self):
        super().__init__('door_goal_calculator')

        # ---------------------------------
        # Subscriber
        # ---------------------------------

        self.door_point1_sub = self.create_subscription(
            PointStamped,
            '/mpc/door_point1',
            self.door_point1_callback,
            10
        )

        self.door_point2_sub = self.create_subscription(
            PointStamped,
            '/mpc/door_point2',
            self.door_point2_callback,
            10
        )

        # ---------------------------------
        # Publisher
        # ---------------------------------

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/mpc/goal',
            10
        )

        # ---------------------------------
        # 出入口端点
        # ---------------------------------

        self.door_point1 = None
        self.door_point2 = None

        self.get_logger().info(
            'door_goal_calculator started.'
        )

    # =====================================
    # Point 1 callback
    # =====================================

    def door_point1_callback(self, msg):

        self.door_point1 = (
            msg.point.x,
            msg.point.y
        )

        self.calculate_goal()

    # =====================================
    # Point 2 callback
    # =====================================

    def door_point2_callback(self, msg):

        self.door_point2 = (
            msg.point.x,
            msg.point.y
        )

        self.calculate_goal()

    # =====================================
    # Goal calculation
    # =====================================

    def calculate_goal(self):

        # 2点が揃っていなければ計算しない
        if self.door_point1 is None:
            return

        if self.door_point2 is None:
            return

        # ---------------------------------
        # 出入口両端
        # ---------------------------------

        x1, y1 = self.door_point1
        x2, y2 = self.door_point2

        # ---------------------------------
        # 出入口中心
        # ---------------------------------

        goal_x = (x1 + x2) / 2.0
        goal_y = (y1 + y2) / 2.0

        # ---------------------------------
        # 出入口方向ベクトル
        # ---------------------------------

        dx = x2 - x1
        dy = y2 - y1

        # ---------------------------------
        # 法線ベクトル
        #
        # (-dy, dx)
        # ---------------------------------

        nx = -dy
        ny = dx

        # ---------------------------------
        # 原点 → 出入口中心
        # ---------------------------------

        cx = goal_x
        cy = goal_y

        # ---------------------------------
        # 法線方向を
        # 「原点 → 出入口中心」
        # に合わせる
        # ---------------------------------

        dot = nx * cx + ny * cy

        if dot < 0:
            nx = -nx
            ny = -ny

        # ---------------------------------
        # 目的角度
        # ---------------------------------

        goal_theta = math.atan2(ny, nx)

        # ---------------------------------
        # Publish
        # ---------------------------------

        goal_msg = PoseStamped()

        goal_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.header.frame_id = 'rslidar'

        goal_msg.pose.position.x = float(goal_x)
        goal_msg.pose.position.y = float(goal_y)
        goal_msg.pose.position.z = 0.0

        # yaw → quaternion
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(
            goal_theta / 2.0
        )
        goal_msg.pose.orientation.w = math.cos(
            goal_theta / 2.0
        )

        self.goal_pub.publish(goal_msg)

        self.get_logger().info(
            f'Goal: '
            f'x={goal_x:.1f}, '
            f'y={goal_y:.1f}, '
            f'theta={math.degrees(goal_theta):.1f} deg'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CenterCalculator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()