import rclpy
from rclpy.node import Node
from datetime import datetime
import os
from nav_msgs.msg import Odometry

class OdomSaver(Node):
    def __init__(self):
        super().__init__('odom_saver')

        self.subscription = self.create_subscription(
            Odometry,
            '/whill/odom',
            self.listener_callback,
            10)

        # 保存先ディレクトリ
        save_dir = '/home/ubuntu/Desktop/togakushi2_log/Odom_logs'
        os.makedirs(save_dir, exist_ok=True)

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file_path = os.path.join(save_dir, f'odom_log_{now_str}.txt')
        self.file = open(self.file_path, 'w')

        self.get_logger().info(f"保存開始: {self.file_path}")

    def listener_callback(self, msg):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        # 位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # 姿勢（クォータニオン）
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # 速度
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        wx = msg.twist.twist.angular.x
        wy = msg.twist.twist.angular.y
        wz = msg.twist.twist.angular.z

        line = (f"{now}, pos: {x:.3f} {y:.3f} {z:.3f}, "
                f"ori: {qx:.3f} {qy:.3f} {qz:.3f} {qw:.3f}, "
                f"lin_vel: {vx:.3f} {vy:.3f} {vz:.3f}, "
                f"ang_vel: {wx:.3f} {wy:.3f} {wz:.3f}\n")
        
        self.file.write(line)
        self.file.flush()
        self.get_logger().debug(f'Saved: {line.strip()}')

    def destroy_node(self):
        self.file.close()
        self.get_logger().info("保存終了")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
