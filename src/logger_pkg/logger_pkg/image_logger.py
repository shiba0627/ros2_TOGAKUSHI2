#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage # 購読するメッセージ型
#from cv_bridge import CvBridge
#import cv2
import os
import rclpy.duration

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_log_node')

        # CvBridgeのインスタンスを作成
        #self.bridge = CvBridge()

        # 保存先ディレクトリ（存在しなければ作成）
        self.save_directory = '/home/ubuntu/Desktop/togakushi2_log/image_logs'
        os.makedirs(self.save_directory, exist_ok=True)

        # 圧縮済み画像トピックを購読
        self.subscription = self.create_subscription(
            CompressedImage, # ★CompressedImage型を購読
            '/camera/gameimage/compressed', # ★圧縮側の配信トピック
            self.image_callback,
            10) # QoSプロファイル

        #self.get_logger().info(f"Image Saver Node started. Subscribing to /camera/gameimage/compressed.")
        self.get_logger().info(f"Saving images to: {self.save_directory}")

        # 最後に保存した時刻を記録する変数を初期化
        self.last_save_time = self.get_clock().now()
        # 何秒おきに写真を保存するか定義
        self.one_second_duration = rclpy.duration.Duration(seconds=0.5)

    def image_callback(self, msg):
        current_time = self.get_clock().now()#現在時刻
        duration_since_last_save = current_time - self.last_save_time#差を計算
        # 経過時間が1秒未満なら、何もせずに関数を終了
        if duration_since_last_save < self.one_second_duration:
            # self.get_logger().debug('Skipping save (less than 1 sec)') # デバッグ用
            return
        # 1秒以上経過していたら、最終保存時刻を現在時刻で更新
        self.last_save_time = current_time
        # ファイル名を決定 (タイムスタンプを使用)
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        filename = f"image_{sec}_{nanosec}.jpg"
        filepath = os.path.join(self.save_directory, filename)

        try:
            # 2. msg.data には圧縮済みのJPEGデータ(bytes)がそのまま入っている
            #    これを 'wb' (write binary) モードで書き出すだけ
            with open(filepath, 'wb') as f:
                f.write(msg.data)
            
            self.get_logger().info(f'Successfully saved raw compressed image: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to save raw image {filepath}: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    
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