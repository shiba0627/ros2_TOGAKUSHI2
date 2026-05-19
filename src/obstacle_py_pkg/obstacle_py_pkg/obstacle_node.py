#!/usr/bin/env python3
# 障害物検知ノード + 直交座標点群配信
# 単位は[m]

import json
import rclpy
from rclpy.node import Node

from togakushi2_interfaces.msg import Obstacle
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        self.sub = self.create_subscription(PointCloud2, 'rslidar_points', self.callback, 10)
        
        # 近距離検知出力
        self.pub = self.create_publisher(Obstacle, 'obstacle_info', 10)
        
        # 直交座標点群出力
        self.pub_points = self.create_publisher(String, 'obstacle_points', 10)
        
        self.detection_result = Obstacle()  # 検知結果
        
        # 近距離検知パラメータ
        self.DETECT_RANGE_FRONT = 1.4  # 前方向[m]
        self.DETECT_RANGE_BACK = 0.8   # 後方向[m]
        self.WIDTH_HALF = 0.3           # 前後範囲の半幅
        self.HEIGHT_MIN = 0.1           # 地面のノイズ除去[m]
        self.HEIGHT_MAX = 1.7           # 上限高さ[m]
        self.THRESHOLD = 10             # 障害物判定閾値
        self.R_SIDE = 1.0               # 左右検知範囲、半径
        self.LIDAR_HEIGHT = 1.55        # LiDAR設置高さ[m]
        
        # 点群用パラメータ
        self.POINTS_MAX_RANGE = 1.5     # 3x3m = ±1.5m

    def callback(self, lidar_msg):
        xyz_points = point_cloud2.read_points(
            lidar_msg, 
            field_names=('x', 'y', 'z', 'intensity'), 
            skip_nans=True
        )
        
        # xyz_points はジェネレータなので、リストに変換（複数回使用するため）
        xyz_points_list = list(xyz_points)
        
        # 近距離検知
        self.Filter_Points_Near(xyz_points_list)
        self.pub.publish(self.detection_result)
        
        # 直交座標点群生成
        points_msg = self.Generate_Points(xyz_points_list)
        self.pub_points.publish(points_msg)

    def Filter_Points_Near(self, xyz_points):
        """既存の近距離検知ロジック"""
        count = {1: 0, 3: 0, 6: 0, 9: 0}  # 1:前, 3:右, 6:後ろ, 9:左
        self.detection_result.data = [False] * 10
        
        for point in xyz_points:
            x, y, z = point[0], point[1], point[2]
            z = z + self.LIDAR_HEIGHT  # LiDAR設置高さのオフセット
            
            # 高さ成分が範囲外なら何もしない
            if not (self.HEIGHT_MIN <= z <= self.HEIGHT_MAX):
                continue
            
            # 近距離検知
            # 前方
            if -self.WIDTH_HALF <= y <= self.WIDTH_HALF and 1 <= x <= self.DETECT_RANGE_FRONT:
                count[1] += 0.1
            # 後方
            elif -self.WIDTH_HALF <= y <= self.WIDTH_HALF and -self.DETECT_RANGE_BACK <= x <= 0:
                count[6] += 1
            # 右
            elif y < -self.WIDTH_HALF and x > 0 and x**2 + y**2 <= self.R_SIDE**2:
                count[3] += 1
            # 左
            elif y > self.WIDTH_HALF and x > 0 and x**2 + y**2 <= self.R_SIDE**2:
                count[9] += 1
        
        # 点数が閾値より多かったら障害物
        for idx, cnt in count.items():
            self.detection_result.data[idx] = cnt >= self.THRESHOLD

    def Generate_Points(self, xyz_points):
        """3x3m範囲内の点を直交座標で抽出し、JSON化して返す"""
        points = []
        
        for point in xyz_points:
            x, y, z = point[0], point[1], point[2]
            z = z + self.LIDAR_HEIGHT  # LiDAR設置高さのオフセット
            
            # 高さフィルタ
            if not (self.HEIGHT_MIN <= z <= self.HEIGHT_MAX):
                continue
            
            # 3x3m範囲フィルタ（-1.5m ～ +1.5m）
            if not (-self.POINTS_MAX_RANGE <= x <= self.POINTS_MAX_RANGE and
                    -self.POINTS_MAX_RANGE <= y <= self.POINTS_MAX_RANGE):
                continue
            
            # 直交座標をそのまま記録
            points.append({
                "x": float(round(x, 3)),
                "y": float(round(y, 3))
            })
        
        # JSON化
        json_data = json.dumps({"points": points})
        
        # String型メッセージで返す
        msg = String()
        msg.data = json_data
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()