#20250209
#python3 test_pkg_python/test_pkg_python/real_DBSCAN.py 
#リアルタイムで出入り口を検出しパブリッシュ
from matplotlib import pyplot as plt
import csv
import numpy as np
import math
from sklearn.cluster import DBSCAN
from itertools import combinations
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import threading
import readchar
import time

class ee_detection(Node):
    def __init__(self):
        #変数定義
        self.e = 90
        self.min = 4
        self.lim = 3000
        super().__init__('ee_detection_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription#エラー対策
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'door_data',
            10)
        #self.get_logger().info('__init__')#ターミナルに出力
        self.latest_scan = None

        self.timer = self.create_timer(0.01,self.timer_callback)#パブリッシュ用のタイマー

    def timer_callback(self):
        #self.get_logger().info('Timer callback running')
        if hasattr(self, 'door_data'):
            if np.array_equal(self.door_data, np.array([0, 0, 0, 0, 999999])):
                self.get_logger().info("ドアなし")
            else:
                msg = Float32MultiArray()
                msg.data = [float(x) for x in self.door_data]  # NumPy配列をリストに変換
                self.publisher.publish(msg)
                self.get_logger().info(f'Published door_data: {msg.data}')

    def scan_callback(self, msg):
        """ LIDARのスキャンデータを受信 """
        self.latest_scan = msg  # 最新のデータを保存
        if self.latest_scan is not None:
            time1 = time.time()
            self.process_scan(self.latest_scan)
            time2 = time.time()
            #self.get_logger().info(f'処理時間{time2-time1}')
        else:
            self.get_logger().info(f'scan data is no')
    
    

            
    def process_scan(self, msg):
        """ スキャンデータを処理 """
        #angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        data = np.array(msg.ranges)
        ID = np.arange(0, len(data))
        left = np.column_stack((ID,data))
        self.xy_left = left_data(left)
        self.xy = self.xy_left
        #self.get_logger().info(f'data.shape={data.shape}')
        #self.get_logger().info(f'left.shape={left.shape}')
        #self.get_logger().info(f'xy_left.shape={self.xy_left.shape}')
        self.analyze()
    
    def analyze(self):
        db = DBSCAN(eps = self.e, min_samples=self.min)
        db.fit(self.xy)
        self.labels = db.labels_
        self.cluster_num = max(self.labels) + 1#クラスタ総数, 外れ値は除く
        self.xyID = np.column_stack((self.xy[:,0], self.xy[:,1], self.labels))
        self.split_data()
        #self.get_logger().info(f'doot_data = {self.door_data}')
        


    def split_data(self):
        combination_list = self.combination()#クラスタの組み合わせ
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
                if dis < door_data[4] and dis > 550 and dis < 1000:
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
        
    def cluster_range(self, cls_1, cls_2):#クラスタ間の距離
        tate = len(cls_1)
        yoko = len(cls_2)
        shortest_dis = 99999
        for i in range(tate):
            for j in range(yoko):
                a = self.range_calculate(cls_1[i,0], cls_2[j,0], cls_1[i,1], cls_2[j,1])
                if a < shortest_dis:
                    shortest_dis = a
                    shortest_xy = np.array([[cls_1[i,0], cls_1[i,1]],[cls_2[j,0], cls_2[j,1]]])
        return shortest_xy, shortest_dis
    def range_calculate(self,x1,x2,y1,y2):#二点間の距離
        x=x1-x2
        y=y1-y2
        a=x*x+y*y
        a=math.sqrt(a)
        return a
    
    def combination(self):
        elements = np.arange(0,self.cluster_num)
        combination_list = np.array(list(combinations(elements, 2)))
        return combination_list
    
    def plot_data_Nx2_red(self, data):
        plt.scatter(data[:,1], data[:,0], c = 'red', s=10)
        plt.xlim(-self.lim, self.lim) # x軸の範囲
        plt.ylim(-self.lim, self.lim) # y軸の範囲
        
def left_data(data):
    #点群を回転
    for i in range(len(data)):
        data[i,0]=data[i,0]+0
        data[i,1]=data[i,1]*1000
    #絶対値が3000以上, 100以下を除外
    filter_data = []
    for i in range(len(data)):
        if int(data[i,1]) < 3000 and int(data[i,1]) > 20:
            filter_data.append(data[i,:])
    #極座標を直行座標に変換
    list_x = []
    list_y = []
    xy = []
    p=math.pi
    for i in range(len(filter_data)):
        if 360 < data[i,0]:
            x=-int(filter_data[i][1]) * math.cos((315-int(filter_data[i][0])/4)*p/180)
            y=-int(filter_data[i][1]) * math.sin((315-int(filter_data[i][0])/4)*p/180)-230
            list_x.append(x)
            list_y.append(y)
    xy = np.column_stack((list_x, list_y))
    return xy

def right_data(data):
    #絶対値が3000以上, 10以下を除外
    filter_data = []
    for i in range(len(data)):
        if int(data[i][1]) < 3000 and int(data[i][1]) > 500:
            filter_data.append(data[i][:])
    #極座標を直行座標に変換
    list_x = []
    list_y = []
    xy = []
    p=math.pi
    for i in range(len(filter_data)):
        if 720 >int(data[i][0]):
            x=-int(filter_data[i][1]) * math.cos((315-int(filter_data[i][0])/4)*p/180)
            y=-int(filter_data[i][1]) * math.sin((315-int(filter_data[i][0])/4)*p/180)+230
            list_x.append(x)
            list_y.append(y)
    xy = np.column_stack((list_x, list_y))
    return xy
def main():
    rclpy.init()
    node = ee_detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()