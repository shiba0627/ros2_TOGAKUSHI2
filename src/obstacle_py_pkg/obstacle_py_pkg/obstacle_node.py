# 障害物検知のノード
# 単位は[m]
import rclpy
from rclpy.node import Node

from togakushi2_interfaces.msg import Obstacle
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        self.sub = self.create_subscription(PointCloud2, 'rslidar_points', self.callback, 10)
        self.pub = self.create_publisher(Obstacle, 'obstacle_info', 10)
        self.pub_far = self.create_publisher(Obstacle, 'obstacle_info_far', 10)#遠方障害物用
        self.detection_result = Obstacle() # 検知結果
        self.detection_result_far = Obstacle()# 遠方検知結果
        
        #近距離検知パラメータ
        self.DETECT_RANGE_FRONT = 1.0# 前方向[m]
        self.DETECT_RANGE_BACK = 0.8# 後方向[m]
        self.WIDTH_HALF = 0.3#前後範囲の半幅
        self.HEIGHT_MIN = 0.1# 地面のノイズ除去[m]
        self.HEIGHT_MAX = 1.7# 上限高さ[m]
        self.THRESHOLD = 10# 障害物判定閾値
        self.R_SIDE = 0.8# 左右検知範囲、半径
        
        #遠距離検知パラメータ
        self.GRID_SIZE_FAR = 2.0#[m]遠距離検知のグリッド長
        self.GRID_MAX = 3.0#[3]グリッド座標の最大値
        self.THRESHOLD_FAR = 5
        
    def callback(self, lidar_msg):
        xyz_points = point_cloud2.read_points(lidar_msg,field_names=('x','y','z','intensity'), skip_nans=True)
        self.detection_result.data = [False] * 10#[*,前方,*,右,*,*,後ろ,*,*,左]
        self.detection_result_far.data = [False] * 8#[左前, 前, 右前, 右, 右後, 後, 左後, 左]
        
        self.Filter_Points(xyz_points)

        self.pub.publish(self.detection_result)
        self.pub_far.publish(self.detection_result_far)
        
    def Filter_Points(self, xyz_points):
        #点群を四方向に分割，検知結果を記入
        count = {1: 0, 3: 0, 6: 0, 9: 0}#1:前, 3:右, 6:後ろ, 9:左
        count_far = [0] * 8#0:左前, 1:前, 2:右前, 3:右, 4:右後, 5:後, 6:左後, 7:左

        for point in xyz_points:
            x, y, z = point[0], point[1], point[2]
            z = z + 1.55#[m]LiDAR設置高さのオフセット
            
            #高さ成分が範囲外なら何もしない
            if not (self.HEIGHT_MIN <= z <= self.HEIGHT_MAX):
                continue
            
            # 近距離検知
            # 前方
            if   -self.WIDTH_HALF <= y <= self.WIDTH_HALF and 0 <= x <= self.DETECT_RANGE_FRONT:
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
            
            # 遠方検知
            # 前3方向
            if self.GRID_MAX-self.GRID_SIZE_FAR <= x <= self.GRID_MAX:
                if self.GRID_MAX - self.GRID_SIZE_FAR <= y <= self.GRID_MAX:
                    count_far[0] += 1#左前
                elif -(self.GRID_MAX - self.GRID_SIZE_FAR) <= y <= self.GRID_MAX - self.GRID_SIZE_FAR:
                    count_far[1] += 1#前
                elif -self.GRID_MAX <= y <= -(self.GRID_MAX - self.GRID_SIZE_FAR):
                    count_far[2] += 1#右前
            # 右左
            elif  -(self.GRID_MAX - self.GRID_SIZE_FAR) <= x <= self.GRID_MAX - self.GRID_SIZE_FAR:
                if -self.GRID_MAX <= y <= -(self.GRID_MAX - self.GRID_SIZE_FAR):
                    count_far[3] += 1#右
                elif self.GRID_MAX - self.GRID_SIZE_FAR <= y <= self.GRID_MAX:
                    count_far[7] += 1#左
            # 後3方向
            elif -self.GRID_MAX <= x <= -(self.GRID_MAX - self.GRID_SIZE_FAR) :
                if -self.GRID_MAX <= y <= -(self.GRID_MAX - self.GRID_SIZE_FAR):
                    count_far[4] += 1#右後
                elif -(self.GRID_MAX - self.GRID_SIZE_FAR) <= y <= self.GRID_MAX - self.GRID_SIZE_FAR:
                    count_far[5] += 1#後
                elif self.GRID_MAX - self.GRID_SIZE_FAR <= y <= self.GRID_MAX:
                    count_far[6] += 1#左後

        # 点数が閾値より多かったら障害物
        for idx, cnt in count.items():
            self.detection_result.data[idx] = cnt >= self.THRESHOLD
        for i ,cnt in enumerate(count_far):
            self.detection_result_far.data[i] = cnt >= self.THRESHOLD_FAR

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()