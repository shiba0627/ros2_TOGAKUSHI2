# キーボード入力を監視し、キー入力があったらflagをTrueに
# /home/ubuntu/tone_ws2/src/lidar_save_pkg/lidar_save_pkg/key_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import select
import sys

class KeyNode(Node):
    def __init__(self):
        super().__init__('key_node')
        self.publisher = self.create_publisher(Bool,'/key_flag', 10)
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.get_logger().info('sキーで写真を保存')
    def timer_callback(self):
        if select.select([sys.stdin],[],[], 0)[0]:
            key = sys.stdin.read(1)
            if key.lower() == 's':
                msg = Bool()
                msg.data = True
                self.publisher.publish(msg)
                self.get_logger().info('sが押下。写真と点群を保存')
        

def main():
    rclpy.init()
    node = KeyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('ctrl + c で終了')
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()