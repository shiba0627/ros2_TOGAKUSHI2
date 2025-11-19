# カメラ数:2
# 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2
import datetime
import os
from config import CAM_ID, CAM_ID_2, SAVE_PATH

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        self.sub = self.create_subscription(Bool,'/key_flag', self.image_callback,10)
        self.cap1 = cv2.VideoCapture(CAM_ID)
        self.cap1.set(cv2.CAP_PROP_BUFFERSIZE, 5)
        self.cap2 = cv2.VideoCapture(CAM_ID_2)
        self.cap2.set(cv2.CAP_PROP_BUFFERSIZE, 5)
        if not self.cap1.isOpened():
            self.get_logger().error('カメラ1が開けません')
        if not self.cap2.isOpened():
            self.get_logger().error('カメラ2が開けません')

        os.makedirs(SAVE_PATH, exist_ok=True)
        self.get_logger().info('__init__完了')
    def image_callback(self,msg):
        for _ in range(5):
            self.cap1.read()#バッファクリア 5フレーム分読み捨て
            self.cap2.read()#バッファクリア 5フレーム分読み捨て
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

        if ret1:
            filename1 = os.path.join(SAVE_PATH,f'image1_{timestamp}.jpg')
            cv2.imwrite(filename1, frame1)
            self.get_logger().info('カメラ1画像保存完了')
        else:
            self.get_logger().warn('カメラ1画像取得失敗')
        
        if ret2:
            filename2 = os.path.join(SAVE_PATH,f'image2_{timestamp}.jpg')
            cv2.imwrite(filename2, frame2)
            self.get_logger().info('カメラ2画像保存完了')
        else:
            self.get_logger().warn('カメラ2画像取得失敗')

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