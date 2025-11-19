# カメラ数:1
# 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import select
import sys
import cv2
import datetime
import os
from config import CAM_ID, SAVE_PATH

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        self.sub = self.create_subscription(Bool,'/key_flag', self.image_callback,10)
        self.cap = cv2.VideoCapture(CAM_ID)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 5)
        os.makedirs(SAVE_PATH, exist_ok=True)
        self.get_logger().info('__init__完了')
    def image_callback(self,msg):
        for _ in range(5):
            self.cap.read()#バッファクリア 5フレーム分読み捨て
            ret, frame = self.cap.read()
        ret, frame = self.cap.read()
        if ret:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(SAVE_PATH,f'image_{timestamp}.jpg')
            cv2.imwrite(filename, frame)
            self.get_logger().info('カメラ画像保存完了')
        else:
            self.get_logger().warn('カメラ画像取得失敗')

def main():
    rclpy.init()
    node = CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('ctrl + c で終了')
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()