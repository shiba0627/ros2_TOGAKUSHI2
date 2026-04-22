#使ってないノード
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # 購読メッセージ型
#from cv_bridge import CvBridge
#import cv2
import os

day = 20260113

class HWencode_node(Node):
    def __init__(self):
        super().__init__('HWencode_node')

        #動画保存先
        self.save_directory = f'/home/ubuntu/Desktop/togakushi2_log/{day}/image_logs'
        os.makedirs(self.save_directory, exist_ok=True)

        #画像トピックを購読
        self.img_subscription = self.create_subscription(
            Image,
            '/camera/gameimage',
            self.image_callback,
            10)
    def image_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    image_saver = HWencode_node()
    
    # rclpy.spin()の前にチェック（コンストラクタでエラー終了した場合）
    if rclpy.ok():
        try:
            rclpy.spin(image_saver)
        except KeyboardInterrupt:
            pass # Ctrl+Cが押された場合はスピンを停止
        finally:
            # ノード終了処理
            image_saver.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()