import rclpy
from rclpy.node import Node
import os
from config import CAM_ID, SAVE_PATH
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import datetime

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.sub_flag = self.create_subscription(Bool, '/key_flag', self.flag_callback, 10)
        self.sub_lidar = self.create_subscription(PointCloud2,'rslidar_points', self.lidar_callback, 10)
        self.save_flag = False

    def flag_callback(self, msg):
        self.save_flag = True
        self.get_logger().info('key_flagがTrue')

    def lidar_callback(self, msg):
        if self.save_flag:
            self.get_logger().info('lidarcallback')
            self.save_flag = False
            xyz = self.read_lidar(msg)
            points = list(xyz)
            self.get_logger().info(f'点群を保存完了')
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(SAVE_PATH,f'lidar_{timestamp}.txt')
            with open(filename,'w') as f:
                for point in points:
                    f.write(','.join(map(str,point))+'\n')
    def read_lidar(self, msg):
        xyz_points = point_cloud2.read_points(msg, field_names=('x','y','z','intensity'))
        return xyz_points
def main():
    rclpy.init()
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('ctrl + c で終了')
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()