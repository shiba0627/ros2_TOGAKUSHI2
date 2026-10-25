import os
from datetime import datetime

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudSaver(Node):

    def __init__(self):
        super().__init__('pointcloud_saver')

        # ---------------------------------
        # 保存先
        # ---------------------------------

        self.output_dir = (
            '/home/ubuntu/humble_ws/src/'
            'mpc_navigation_pypkg/mpc_navigation_pypkg/output'
        )

        self.pointcloud_dir = os.path.join(
            self.output_dir,
            'pointclouds'
        )

        self.plot_dir = os.path.join(
            self.output_dir,
            'plots'
        )

        os.makedirs(self.pointcloud_dir, exist_ok=True)
        os.makedirs(self.plot_dir, exist_ok=True)

        # ---------------------------------
        # Subscriber
        # ---------------------------------

        self.filtered_sub = self.create_subscription(
            PointCloud2,
            '/mpc/filtered_points',
            self.filtered_points_callback,
            10
        )

        # ---------------------------------
        # 保存間隔
        # ---------------------------------

        self.save_interval = 1.0
        self.last_save_time = self.get_clock().now()

        self.get_logger().info(
            'PointCloudSaver started.'
        )

        self.get_logger().info(
            'Subscribed to /mpc/filtered_points'
        )

    # =================================
    # PointCloud callback
    # =================================

    def filtered_points_callback(self, msg):

        # ---------------------------------
        # 保存間隔チェック
        # ---------------------------------

        now = self.get_clock().now()

        elapsed = (
            now - self.last_save_time
        ).nanoseconds / 1e9

        if elapsed < self.save_interval:
            return

        self.last_save_time = now

        # ---------------------------------
        # PointCloud2 → NumPy
        # ---------------------------------

        points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        if len(points) == 0:
            self.get_logger().warn(
                'No points received.'
            )
            return

        # 構造化配列から各座標を取得
        x = np.asarray(points['x'], dtype=np.float32)
        y = np.asarray(points['y'], dtype=np.float32)
        z = np.asarray(points['z'], dtype=np.float32)

        # ---------------------------------
        # ファイル名
        # ---------------------------------

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S_%f'
        )[:-3]

        # ---------------------------------
        # 点群保存
        # ---------------------------------

        npz_path = os.path.join(
            self.pointcloud_dir,
            f'pointcloud_{timestamp}.npz'
        )

        np.savez_compressed(
            npz_path,
            x=x,
            y=y,
            z=z
        )

        # ---------------------------------
        # XYプロット
        # ---------------------------------

        png_path = os.path.join(
            self.plot_dir,
            f'pointcloud_{timestamp}.png'
        )

        plt.figure(figsize=(8, 8))

        plt.scatter(
            x,
            y,
            s=3
        )

        # 車いす位置
        plt.scatter(
            0,
            0,
            s=50,
            marker='x'
        )

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Filtered PointCloud XY')

        plt.axis('equal')
        plt.grid(True)

        plt.tight_layout()

        plt.savefig(
            png_path,
            dpi=150
        )

        plt.close()

        # ---------------------------------
        # ログ
        # ---------------------------------

        self.get_logger().info(
            f'Saved {len(points)} points'
        )

        self.get_logger().info(
            f'NPZ: {npz_path}'
        )

        self.get_logger().info(
            f'PNG: {png_path}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PointCloudSaver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()