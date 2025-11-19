# 20250206
# python 2d_lidar_2.8.py
# 2D-LiDAR取得データのCSVファイルを読み込み、クラスタリングを用いて出入口を検出
# LiDAR二つ, 原点の補正を行う, 2月のあけぼの訪問時用
# 外れ値クラスタは計算しない
# 処理時間を計測
import time
from matplotlib import pyplot as plt
import csv
import numpy as np
import math
from sklearn.cluster import DBSCAN
from itertools import combinations
class read_lidardata():
    def __init__(self):
        #パラメータ
        self.e=90#DBSCAN
        self.min=4#DBSCAN
        self.lim = 3000#プロットするときの軸上限値
        filename = f'./data/csv/20250128/2_left.csv'
        filename2 = './data/csv/20250128/2_right.csv'
        #csv読み込み(ID, absolute)
        with open(filename, encoding='utf8', newline='') as f:
            csvreader = csv.reader(f)
            next(csvreader)#1行目を無視
            self.data_left = [row for row in csvreader]
        with open(filename2, encoding='utf8', newline='') as f:
            csvreader2 = csv.reader(f)
            next(csvreader2)#1行目を無視
            self.data_right = [row for row in csvreader2]
        np_left = np.array(self.data_left, dtype=float)
        self.xy_left = left_data(np_left)
        self.xy_right = right_data(self.data_right)
        #self.plot_data_Nx2_blue(self.xy_left)
        #self.plot_data_Nx2_red(self.xy_right)
        self.xy = np.vstack((self.xy_left,self.xy_right))
        self.plot_data_Nx2_blue(self.xy)
        #クラスタリング
        db = DBSCAN(eps=self.e, min_samples=self.min)
        db.fit(self.xy)
        self.labels = db.labels_
        self.cluster_num = max(self.labels) - min(self.labels)#クラスタ総数, 外れ値は除く
        self.xyID = np.column_stack((self.xy[:,0], self.xy[:,1], self.labels))
        print(self.xyID)
    
    def split_data(self):
        combination_list = self.combination()#クラスタの組み合わせ
        now_list = []#現在考えているlistひとつめ
        next_list = []#listふたつめ
        shortest_list = []
        door_data = np.array([0,0,0,0,99999])#x1,y1,x2,y2,キョリ
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
        plt.plot([door_data[1],door_data[3]],[door_data[0],door_data[2]],color = 'red')

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
    def plot_data_Nx2_blue(self, data):
        #plt.figure()
        #print(data[:,2])
        plt.scatter(data[:,1], data[:,0], c = 'blue', s=10)
        #plt.scatter(data[:][1], data[:][0], c = 'red', s=20)
        #plt.plot([data[0,1],data[1,1]],[data[0,0],data[1,0]],color = 'red')
        plt.xlim(-self.lim, self.lim) # x軸の範囲
        plt.ylim(-self.lim, self.lim) # y軸の範囲

def left_data(data):
    #点群を回転
    for i in range(len(data)):
        data[i,0]=data[i,0]+40
    #絶対値が3000以上, 100以下を除外
    filter_data = []
    for i in range(len(data)):
        if int(data[i,1]) < 3000 and int(data[i,1]) > 500:
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
    lidar_data = read_lidardata()
    lidar_data.split_data()
    plt.axvline(x=0, color='black')
    plt.axhline(y=0, color='black')
    plt.show()

if __name__ == '__main__':
    main()

