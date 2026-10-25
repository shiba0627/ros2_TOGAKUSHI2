import math
from itertools import combinations

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from geometry_msgs.msg import PointStamped


class GoalCalculator(Node):

    def __init__(self):
        super().__init__('goal_calculator')

        # ==========================================
        # Subscriber
        # ==========================================
        self.filtered_sub = self.create_subscription(
            PointCloud2,
            '/mpc/filtered_points',
            self.filtered_points_callback,
            10
        )

        # ==========================================
        # Publisher
        # ==========================================
        self.door_point1_pub = self.create_publisher(
            PointStamped,
            '/mpc/door_point1',
            10
        )

        self.door_point2_pub = self.create_publisher(
            PointStamped,
            '/mpc/door_point2',
            10
        )

        # ==========================================
        # DBSCAN parameters
        # ==========================================
        self.eps = 0.3#[m]
        self.min_samples = 4

        # 出入口として許容するクラスタ間距離 [m]
        self.door_min_distance = 0.55
        self.door_max_distance = 1

        # ==========================================
        # Variables
        # ==========================================
        self.xy = None
        self.door_data = None

        self.get_logger().info(
            'goal_calculator started.'
        )

    # ==========================================================
    # PointCloud2 callback
    # ==========================================================
    def filtered_points_callback(self, msg):

        # ------------------------------------------
        # PointCloud2 → NumPy
        # ------------------------------------------
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=['x', 'y']
        )

        if points is None or len(points) == 0:
            self.get_logger().warn(
                'No points received.'
            )
            return

        self.xy = points

        # ------------------------------------------
        # DBSCAN
        # ------------------------------------------
        db = DBSCAN(
            eps=self.eps,
            min_samples=self.min_samples
        )

        db.fit(self.xy)

        self.labels = db.labels_
        self.cluster_num = max(self.labels)+1
        self.xyID = np.column_stack((self.xy[:,0], self.xy[:,1], self.labels))

        self.split_data()

        # ------------------------------------------
        # 出入口両端をPublish
        # ------------------------------------------
        self.publish_door_points()

    # ==========================================================
    # クラスタの組み合わせから出入口を探索
    # ==========================================================
    def split_data(self):
        elements = np.arange(0, self.cluster_num)
        combination_list = np.array(list(combinations(elements, 2)))
        now_list = []#現在考えているlistひとつめ
        next_list = []#listふたつめ
        shortest_list = []
        door_data = np.array([0,0,0,0,999999])#x1,y1,x2,y2,キョリ
        for i in range(len(combination_list)):#i:組み合わせ数だけ繰り返す
            now_index = np.where(self.xyID[:, 2] == combination_list[i][0])
            for k in now_index[0]:#k:クラスタ番号iのインデックス
                now_list.append(self.xyID[k])
            next_index = np.where(self.xyID[:, 2] == combination_list[i][1])
            for j in next_index[0]:
                next_list.append(self.xyID[j])
            next_list_np = np.array(next_list)
            now_list_np = np.array(now_list)
            if len(now_list) > 20 and len(next_list) > 20:
                shortest_xy, dis = self.cluster_range(next_list_np, now_list_np)#shotest_xy.shape=(2,2)
                if dis < door_data[4] and dis > 0.55 and dis < 1:
                    door_data[0] = shortest_xy[0][0]
                    door_data[1] = shortest_xy[0][1]
                    door_data[2] = shortest_xy[1][0]
                    door_data[3] = shortest_xy[1][1]
                    door_data[4] = dis
                #self.plot_data_Nx2(shortest_xy)
                shortest_list.append(shortest_xy)
                now_list = []
                next_list = []
        self.door_data = door_data

        if self.door_data is not None:

            self.get_logger().info(
                'Door detected: '
                f'P1=({self.door_data[0]:.1f}, '
                f'{self.door_data[1]:.1f}), '
                f'P2=({self.door_data[2]:.1f}, '
                f'{self.door_data[3]:.1f}), '
                f'distance={self.door_data[4]:.1f}'
            )

    # ==========================================================
    # 2クラスタ間の最短距離
    # ==========================================================
    def cluster_range(self, cls_1, cls_2):

        shortest_distance = float('inf')
        shortest_xy = None

        for p1 in cls_1:

            for p2 in cls_2:

                distance = self.range_calculate(
                    p1[0],
                    p2[0],
                    p1[1],
                    p2[1]
                )

                if distance < shortest_distance:

                    shortest_distance = distance

                    shortest_xy = np.array([
                        [p1[0], p1[1]],
                        [p2[0], p2[1]]
                    ])

        return shortest_xy, shortest_distance

    # ==========================================================
    # 2点間距離
    # ==========================================================
    def range_calculate(self, x1, x2, y1, y2):

        return math.sqrt(
            (x1 - x2) ** 2
            +
            (y1 - y2) ** 2
        )

    # ==========================================================
    # 出入口両端をPublish
    # ==========================================================
    def publish_door_points(self):

        # ------------------------------------------
        # 出入口両端
        # ------------------------------------------
        x1 = self.door_data[0]
        y1 = self.door_data[1]

        x2 = self.door_data[2]
        y2 = self.door_data[3]

        # ------------------------------------------
        # PointStamped
        # ------------------------------------------
        point1_msg = PointStamped()
        point2_msg = PointStamped()

        # Header
        now = self.get_clock().now().to_msg()

        point1_msg.header.stamp = now
        point2_msg.header.stamp = now

        point1_msg.header.frame_id = 'rslidar'
        point2_msg.header.frame_id = 'rslidar'

        # ------------------------------------------
        # P1
        # ------------------------------------------
        point1_msg.point.x = float(x1)
        point1_msg.point.y = float(y1)
        point1_msg.point.z = 0.0

        # ------------------------------------------
        # P2
        # ------------------------------------------
        point2_msg.point.x = float(x2)
        point2_msg.point.y = float(y2)
        point2_msg.point.z = 0.0

        # ------------------------------------------
        # Publish
        # ------------------------------------------
        self.door_point1_pub.publish(point1_msg)
        self.door_point2_pub.publish(point2_msg)

        self.get_logger().info(
            f'Published door points: '
            f'P1=({x1:.1f}, {y1:.1f}), '
            f'P2=({x2:.1f}, {y2:.1f})'
        )


# ==========================================================
# main
# ==========================================================
def main(args=None):

    rclpy.init(args=args)

    node = GoalCalculator()

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