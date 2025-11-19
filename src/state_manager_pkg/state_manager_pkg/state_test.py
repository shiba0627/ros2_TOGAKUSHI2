"""
StateManagerノードの動作確認用ノード
キーボード入力で /state_transition に状態変更要求をPublishする
また、/robot_state を購読して現在状態を表示する
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StateTester(Node):
    def __init__(self):
        super().__init__('state_tester')

        # Publisher: 状態変更要求
        self.cmd_pub = self.create_publisher(String, '/state_transition', 10)

        # Subscriber: 現在状態を購読
        self.state_sub = self.create_subscription(String, '/robot_state', self.on_state_update, 10)

        self.get_logger().info("=== StateTester 起動 ===")
        self.get_logger().info("コマンド入力で状態変更をテストします。")
        self.get_logger().info("利用可能な状態: USER, HELPER, OUTO_STOP, ERROR")
        self.get_logger().info("終了するには Ctrl+C を押してください。")

        # 入力受付を別スレッドで実行
        import threading
        threading.Thread(target=self.input_loop, daemon=True).start()

    def input_loop(self):
        """キーボード入力を監視して状態変更要求を送信"""
        while rclpy.ok():
            try:
                next_state = input("次の状態を入力 > ").strip().upper()
                if not next_state:
                    continue
                msg = String()
                msg.data = next_state
                self.cmd_pub.publish(msg)
                self.get_logger().info(f"要求を送信: {next_state}")
            except (EOFError, KeyboardInterrupt):
                break

    def on_state_update(self, msg):
        """現在の状態を受信して表示"""
        self.get_logger().info(f"[状態更新] 現在: {msg.data}")


def main():
    rclpy.init()
    node = StateTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
