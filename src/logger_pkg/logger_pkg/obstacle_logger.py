import rclpy
from rclpy.node import Node
from ros2_whill.msg import BoolMultiArray
from datetime import datetime
import os

class ObstacleSaver(Node):
    def __init__(self):
        super().__init__('obstacle_saver')

        self.subscription = self.create_subscription(
            BoolMultiArray,
            '/obstacle_info',
            self.listener_callback,
            10)

        # 保存先ディレクトリ（存在しなければ作成）
        save_dir = 'obstacle_logs'
        os.makedirs(save_dir, exist_ok=True)

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file_path = os.path.join(save_dir, f'obstacle_log_{now_str}.txt')
        self.file = open(self.file_path, 'w')

        self.get_logger().info(f"保存開始: {self.file_path}")

    def listener_callback(self, msg):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        bool_str = ' '.join(['1' if b else '0' for b in msg.data])
        line = f"{now}, {bool_str}\n"
        self.file.write(line)
        self.file.flush()  # 万が一落ちたときも途中まで残るように
        self.get_logger().debug(f'Saved: {line.strip()}')

    def destroy_node(self):
        self.file.close()
        self.get_logger().info("保存終了")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


