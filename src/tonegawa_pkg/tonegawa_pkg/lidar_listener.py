import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import readchar
import numpy as np
import csv

class ScanSubscriber(Node):

    def __init__(self):
        super().__init__('scan_subscriber')  # スーパーコンストラクタの実行 ノード名を送る
        self.subscription = self.create_subscription(
            LaserScan,  # メッセージ型
            '/scan',  # トピック名
            self.scan_callback,  # コールバック関数
            10  # キューサイズ
        )
        self.scan_data = None
        self.h_pressed = False  # Hキーが押されたかのフラグscript
    
    def save_scan_to_csv(self):
        """スキャンデータをCSVファイルに保存する"""
        if self.scan_data is None:
            self.get_logger().warn("保存するスキャンデータがありません。")
            return

        filename = "src/test_pkg_python/data/scan_data.csv"
        try:
            with open(filename, mode='w', encoding='utf-8',newline='') as file:
                writer = csv.writer(file)
                # ヘッダーがない場合、最初に書く
                if file.tell() == 0:
                    writer.writerow(["ID", "Distance (mm)"])

                # データを書き込む
                #angles = np.linspace(-np.pi, np.pi, len(self.scan_data))
                angles = np.arange(0,len(self.scan_data))
                for angle, distance in zip(angles, self.scan_data):
                    writer.writerow([angle, distance*100])

            self.get_logger().info(f"Scan data saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save data to CSV: {e}")

    def wait_for_keypress(self):
        """Hキーを押すまで待機する"""
        self.get_logger().info("hを押して保存")
        while True:
            char = readchar.readchar()  # キー入力を取得
            if char.lower() == 'h':  # Hキーが押されたら
                self.h_pressed = True
                break
            elif char.lower() == 'q':  # qキーで終了
                self.get_logger().info("プログラムを終了します。")
                exit()


    def scan_callback(self, msg):
        self.scan_data = msg.ranges  # 最新のスキャンデータを保存
    
def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)  # 0.1秒ごとにノードを処理
        if node.h_pressed:  # Hキーが押されたらスキャンデータを保存
            node.h_pressed = False
            node.save_scan_to_csv()

        node.wait_for_keypress()  # Hキーを待機

    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
