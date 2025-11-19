#!/usr/bin/env python3
# 画像を購読，圧縮して配信
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('logger_node')

        # CvBridgeのインスタンスを作成
        self.bridge = CvBridge()

        # 圧縮品質 (0-100, 高いほど高品質・高データ量)
        self.declare_parameter('jpeg_quality', 50)
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value

        # 元のRaw画像トピックを購読
        self.subscription = self.create_subscription(
            Image,
            '/camera/gameimage',
            self.image_callback,
            10) # QoSプロファイル

        # 圧縮済み画像トピックを配信
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/camera/gameimage/compressed',
            10) # QoSプロファイル

        self.get_logger().info(f"Image Compressor Node started. Subscribing to /camera/gameimage, publishing to /camera/gameimage/compressed with JPEG quality {self.jpeg_quality}.")

    def image_callback(self, msg):
        try:
            # 1. ROSのImageメッセージをOpenCVの画像(Numpy配列)に変換
            #    OpenCVはBGR形式を標準とするため 'bgr8' を指定
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # 2. 圧縮後のCompressedImageメッセージを作成
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header  # タイムスタンプとframe_idを必ずコピーする
        compressed_msg.format = "jpeg"

        # 3. OpenCVを使って画像をJPEGにエンコード(圧縮)
        #    encode_param: [フラグ, 値] のリスト
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        result, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)

        if not result:
            self.get_logger().warn('Failed to encode image to JPEG')
            return

        # 4. 圧縮データをメッセージに格納
        #    encoded_image は (N, 1) のNumpy配列なので、.tobytes()でbytesに変換
        compressed_msg.data = encoded_image.tobytes()

        # 5. 圧縮済み画像を配信
        self.publisher_.publish(compressed_msg)

def main(args=None):
    rclpy.init(args=args)
    image_compressor = ImageCompressor()
    rclpy.spin(image_compressor)
    
    # ノード終了処理
    image_compressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()