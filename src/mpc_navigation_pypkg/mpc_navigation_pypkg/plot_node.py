import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped


class PlotSaver(Node):

    def __init__(self):
        super().__init__('plot_saver')

        # ==========================================
        # 保存先
        # ==========================================

        self.save_dir = os.path.expanduser(
            '/home/ubuntu/humble_ws/src/mpc_navigation_pypkg/mpc_navigation_pypkg/output'
        )

        os.makedirs(self.save_dir, exist_ok=True)

        # ==========================================
        # 最新データ
        # ==========================================

        self.filtered_points = None
        self.door_point1 = None
        self.door_point2 = None
        self.goal = None

        # ==========================================
        # Subscriber
        # ==========================================

        self.filtered_sub = self.create_subscription(
            PointCloud2,
            '/mpc/filtered_points',
            self.filtered_points_callback,
            10
        )

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

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/mpc/goal',
            self.goal_callback,
            10
        )

    # ==========================================
    # Callback
    # ==========================================

    def filtered_points_callback(self, msg):

        self.filtered_points = self.pointcloud_to_numpy(msg)

        self.save_plot()

    # ------------------------------------------

    def door_point1_callback(self, msg):

        self.door_point1 = (
            msg.point.x,
            msg.point.y
        )

        self.save_plot()

    # ------------------------------------------

    def door_point2_callback(self, msg):

        self.door_point2 = (
            msg.point.x,
            msg.point.y
        )

        self.save_plot()

    # ------------------------------------------

    def goal_callback(self, msg):

        self.goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.save_plot()

    # ==========================================
    # PointCloud2 → numpy
    # ==========================================

    def pointcloud_to_numpy(self, msg):

        points = []

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            points.append([
                p[0],
                p[1],
                p[2]
            ])

        if len(points) == 0:
            return np.empty((0, 3))

        return np.asarray(points)

    # ==========================================
    # PNG保存
    # ==========================================

    def save_plot(self):

        if (
            self.filtered_points is None
            and self.door_point1 is None
            and self.door_point2 is None
            and self.goal is None
        ):
            return

        fig, ax = plt.subplots(figsize=(8, 8))

        # ======================================
        # Filtered PointCloud
        # ======================================

        if (
            self.filtered_points is not None
            and len(self.filtered_points) > 0
        ):

            points = self.filtered_points

            ax.scatter(
                points[:, 1],
                points[:, 0],
                s=8,
                label='Filtered points'
            )

        # ======================================
        # Door point 1
        # ======================================

        if self.door_point1 is not None:

            x, y = self.door_point1

            ax.scatter(
                x,
                y,
                s=100,
                marker='o',
                label='Door point 1'
            )

        # ======================================
        # Door point 2
        # ======================================

        if self.door_point2 is not None:

            x, y = self.door_point2

            ax.scatter(
                x,
                y,
                s=100,
                marker='o',
                label='Door point 2'
            )

        # ======================================
        # Goal
        # ======================================

        if self.goal is not None:

            x, y = self.goal

            ax.scatter(
                x,
                y,
                s=150,
                marker='*',
                label='Goal'
            )

        # ======================================
        # 車椅子位置
        # ======================================

        ax.scatter(
            0,
            0,
            s=100,
            marker='x',
            label='Wheelchair'
        )

        # ======================================
        # 軸
        # ======================================

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')

        ax.set_title('MPC Navigation')

        ax.set_aspect('equal')

        ax.grid(True)

        ax.legend()

        # ======================================
        # 保存
        # ======================================

        timestamp = self.get_clock().now().nanoseconds

        filename = os.path.join(
            self.save_dir,
            f'plot_{timestamp}.png'
        )

        plt.savefig(
            filename,
            dpi=150,
            bbox_inches='tight'
        )

        plt.close(fig)

        self.get_logger().info(
            f'Saved: {filename}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PlotSaver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()