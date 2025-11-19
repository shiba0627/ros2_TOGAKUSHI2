#3D-LiDARの値を購読、点群データをcsv保存s
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class lidar_subscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.sub = self.sreate_
def main():
    rclpy.init()
    lidar_sub_node = lidar_subscriber()

if __name__ == '__main__':
    main()