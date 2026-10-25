import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudFilter(Node):

    def __init__(self):
        super().__init__('pointcloud_filter')

        # ---------------------------------
        # パラメータ
        # ---------------------------------

        self.z_min = -0.2
        self.z_max = -0.1

        # X, Yの最大値 [m]
        self.x_max = 3.0
        self.y_max = 3.0

        # 最新のPointCloud
        self.latest_filtered_points = []

        # ---------------------------------
        # Subscriber
        # ---------------------------------

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.pointcloud_callback,
            10
        )

        # ---------------------------------
        # Publisher
        # ---------------------------------

        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/mpc/filtered_points',
            10
        )

        # ---------------------------------
        # キーボード確認用Timer
        # ---------------------------------

        self.timer = self.create_timer(
            0.05,   # 20 Hz
            self.keyboard_callback
        )

        # ---------------------------------
        # キーボード入力をraw modeにする
        # ---------------------------------

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(
            'PointCloud filter started.'
        )

        self.get_logger().info(
            'Press SPACE to publish.'
        )

        self.get_logger().info(
            'Press q to quit.'
        )

    # ==================================================
    # PointCloud callback
    # ==================================================

    def pointcloud_callback(self, msg):

        filtered_points = []

        for point in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):

            x, y, z = point

            # ---------------------------------
            # X, Yフィルタ
            # ±3 mを超える点を削除
            # ---------------------------------

            if abs(x) > self.x_max or abs(y) > self.y_max:
                continue

            # ---------------------------------
            # 高さフィルタ
            # ---------------------------------

            if z < self.z_min:
                continue

            if z > self.z_max:
                continue

            # ---------------------------------
            # フィルタ後
            # Zを0にする
            # ---------------------------------

            filtered_points.append(
                (x, y, 0.0)
            )

        self.latest_filtered_points = filtered_points

    # ==================================================
    # キーボード確認
    # ==================================================

    def keyboard_callback(self):

        if select.select(
            [sys.stdin],
            [],
            [],
            0
        )[0]:

            key = sys.stdin.read(1)

            # Space
            if key == ' ':

                self.publish_filtered_points()

            # q
            elif key == 'q':

                self.get_logger().info(
                    'Quit.'
                )

                rclpy.shutdown()

    # ==================================================
    # Publish
    # ==================================================

    def publish_filtered_points(self):

        # ---------------------------------
        # PointCloudがまだ届いていない場合
        # ---------------------------------

        if len(self.latest_filtered_points) == 0:

            self.get_logger().warn(
                'No filtered PointCloud data available.'
            )

            return

        # ---------------------------------
        # PointCloud2を作成
        # ---------------------------------

        msg = point_cloud2.create_cloud_xyz32(
            PointCloud2().header,
            self.latest_filtered_points
        )

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'rslidar'

        # ---------------------------------
        # Publish
        # ---------------------------------

        self.filtered_pub.publish(msg)

        # ---------------------------------
        # ログ
        # ---------------------------------

        self.get_logger().info(
            f'Filtered points: '
            f'{len(self.latest_filtered_points)}'
        )

        self.get_logger().info(
            'Published: /mpc/filtered_points'
        )

    # ==================================================
    # 終了処理
    # ==================================================

    def destroy_node(self):

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.old_settings
        )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PointCloudFilter()

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