import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathGenerator(Node):

    def __init__(self):
        super().__init__('path_generator')

        # ---------------------------------
        # Subscriber
        # ---------------------------------

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/mpc/goal',
            self.goal_callback,
            10
        )

        # ---------------------------------
        # Publisher
        # ---------------------------------

        self.path_pub = self.create_publisher(
            Path,
            '/mpc/path',
            10
        )

        # ---------------------------------
        # パラメータ
        # ---------------------------------

        # 経路点の間隔 [m]
        # 0.1 m = 100 mm
        self.path_interval = 0.1

        self.get_logger().info(
            'path_generator started.'
        )

    # =====================================
    # Goal callback
    # =====================================

    def goal_callback(self, msg):

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        goal_theta = 2.0 * math.atan2(qz, qw)

        self.get_logger().info(
            f'Received Goal: '
            f'x={goal_x:.2f} m, '
            f'y={goal_y:.2f} m, '
            f'theta={math.degrees(goal_theta):.1f} deg'
        )

        # ---------------------------------
        # 経路生成
        # ---------------------------------

        path = self.generate_path(
            goal_x,
            goal_y,
            goal_theta
        )

        # ---------------------------------
        # Publish
        # ---------------------------------

        self.path_pub.publish(path)

        self.get_logger().info(
            f'Published path: {len(path.poses)} points'
        )

    # =====================================
    # Path generation
    # =====================================

    def generate_path(
        self,
        goal_x,
        goal_y,
        goal_theta
    ):

        path = Path()

        path.header.stamp = (
            self.get_clock().now().to_msg()
        )

        path.header.frame_id = 'rslidar'

        # ---------------------------------
        # 原点 → Goalまでの距離 [m]
        # ---------------------------------

        distance = math.sqrt(
            goal_x ** 2 +
            goal_y ** 2
        )

        # ---------------------------------
        # 経路点数
        # ---------------------------------

        num_points = max(
            2,
            int(distance / self.path_interval) + 1
        )

        # ---------------------------------
        # 経路生成
        # ---------------------------------

        for i in range(num_points):

            ratio = i / (num_points - 1)

            x = goal_x * ratio
            y = goal_y * ratio

            pose = PoseStamped()

            pose.header.stamp = path.header.stamp
            pose.header.frame_id = 'rslidar'

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # ---------------------------------
            # yaw → quaternion
            # ---------------------------------

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0

            pose.pose.orientation.z = math.sin(
                goal_theta / 2.0
            )

            pose.pose.orientation.w = math.cos(
                goal_theta / 2.0
            )

            path.poses.append(pose)

        return path


def main(args=None):

    rclpy.init(args=args)

    node = PathGenerator()

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
