import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
from ros2_whill.msg import BoolMultiArray
from sensor_msgs.msg import Joy


# 受信側のIPアドレスとポート番号
HOST = '0.0.0.0'#すべてのリクエストを受け入れ
PORT = 12345

class UdpServerNode(Node):
    def __init__(self):
        super().__init__('udp_server_node')
        
        # UDPソケットの作成
        self.udp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server_socket.bind((HOST, PORT))
        self.get_logger().info(f"UDPサーバーノードが {HOST}:{PORT} で待機中です。")
        
        #---パブリッシャー--
        # ロボット状態の変更要求
        self.state_pub = self.create_publisher(String, '/state_transition', 10)
        #走行制御
        self.pub_joy = self.create_publisher(Joy,'/whill/controller/joy', 10)

        #---サブスクライバー--
        #障害物情報を購読
        self.sub_obstacle_info = self.create_subscription(
            BoolMultiArray, 'obstacle_info', self.callback_lidar, 10)
        #ロボット状態を購読
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_update, 10)
        
        #状態変数
        self.obstacle_front = 0#前方障害物の初期値
        self.obstacle_right = 0#右障害物の初期値
        self.obstacle_left = 0#左障害物の初期値
        self.obstacle_back = 0#後方障害物の初期値
        self.now_state = "USER"#ロボット状態の初期値

        #--- 初期化処理 ---
        transition_msg = String()#状態を格納する型
        transition_msg.data = self.now_state
        self.state_pub.publish(transition_msg)

        # タイマーコールバックを設定して、定期的にソケットを確認
        self.timer = self.create_timer(0.02, self.receive_data)

        
    def state_update(self, msg: String):
        """/robot_stateトピックに基づいて内部の状態を更新する"""
        if self.now_state != msg.data:
            #self.get_logger().info(f"ロボットの状態が '{self.now_state}' から '{msg.data}' に更新されました。")
            self.now_state = msg.data

    def send_joy(self,axis_0_rotate, axis_1_foward):
        msg = Joy()
        axis_value = []
        axis_value.append(float(axis_0_rotate))#旋回
        axis_value.append(float(axis_1_foward))#前進
        msg.axes = axis_value
        self.pub_joy.publish(msg)
        #self.get_logger().info(f"pub_joy{axis_0_rotate},{axis_1_foward}")


    def callback_lidar(self, msg):
        """LiDARからの障害物情報に基づいて状態を更新する"""
        # センサーデータはインデックスで指定
        # 前方: index 1, 右: index 3, 左: index 9, 後方: index 6
        self.obstacle_front = 1 if msg.data[1] else 0
        self.obstacle_right = 1 if msg.data[3] else 0
        self.obstacle_left  = 1 if msg.data[9] else 0
        self.obstacle_back  = 1 if msg.data[6] else 0

    def receive_data(self):
        """UDPソケット受信"""
        try:
            # データを受信
            data, address = self.udp_server_socket.recvfrom(1024)
            message = data.decode('utf-8')
            #self.get_logger().info(f"[{address[0]}:{address[1]}] から受信: {message}")
            # 受信メッセージをJoy指令として解析
            try:
                parts = message.split(',')
                if len(parts) == 2:
                    angular = float(parts[0]) # 旋回
                    linear = float(parts[1])  # 前後進
                    self.send_joy(angular, linear)
            except (ValueError, IndexError):
                # 解析できないメッセージは無視する（状態要求など）
                pass
            #障害物情報と，現在のロボット状態を送信
            response_data = {
                "obstacle_front": self.obstacle_front,
                "obstacle_right": self.obstacle_right,
                "obstacle_left": self.obstacle_left,
                "obstacle_back": self.obstacle_back,
                "robot_state": self.now_state
            }
            response_message = str(response_data)
            self.udp_server_socket.sendto(response_message.encode('utf-8'), address)
            
        except (ValueError, socket.error) as e:
            self.get_logger().error(f"データの処理または通信中にエラーが発生しました: {e}")
            
    def destroy_node(self):
        self.get_logger().info("UDPサーバーノードを終了します。")
        self.udp_server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    udp_server_node = UdpServerNode()
    
    try:
        rclpy.spin(udp_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        udp_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()