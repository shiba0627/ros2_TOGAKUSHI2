import rclpy
from rclpy.node import Node
from datetime import datetime
import os
from sensor_msgs.msg import Joy


class JoySaver(Node):
    def __init__(self):
        super().__init__('Joy_saver')

        self.subscription = self.create_subscription(
            Joy,
            '/whill/states/joy',
            self.listener_callback,
            10)

        # 保存先ディレクトリ（存在しなければ作成）
        save_dir = '/home/ubuntu/Desktop/togakushi2_log/Joy_logs'
        os.makedirs(save_dir, exist_ok=True)

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file_path = os.path.join(save_dir, f'joy_log_{now_str}.txt')
        self.file = open(self.file_path, 'w')

        self.get_logger().info(f"保存開始: {self.file_path}")

    def listener_callback(self, msg):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        axes_str = ' '.join([f"{a:.3f}" for a in msg.axes])
        buttons_str = ' '.join(map(str, msg.buttons))
        line = f"{now}, axes: {axes_str}, buttons: {buttons_str}\n"
        self.file.write(line)
        self.file.flush()
        self.get_logger().debug(f'Saved: {line.strip()}')


    def destroy_node(self):
        self.file.close()
        self.get_logger().info("保存終了")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoySaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


