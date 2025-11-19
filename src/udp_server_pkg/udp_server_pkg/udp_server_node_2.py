import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
from ros2_whill.msg import BoolMultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image  # 追記: 画像メッセージ
from cv_bridge import CvBridge, CvBridgeError # 追記: ROS/CV変換
import cv2 # 追記: OpenCV
import numpy as np # 追記: Numpy

# --- ポート設定 ---
HOST = '0.0.0.0'#すべてのリクエストを受け入れ
CONTROL_PORT = 12345 # Joy指令受信、障害物/状態 送信
IMAGE_PORT = 12346   # 画像送信用ポート

class UdpServerNode(Node):
    def __init__(self):
        super().__init__('udp_server_node_2')
        
        # --- 制御用ソケット (ポート 12345) ---
        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.bind((HOST, CONTROL_PORT))
        # ノンブロッキングに設定（タイマーでポーリングするため）
        self.control_sock.setblocking(False) 
        self.get_logger().info(f"UDP制御サーバーが {HOST}:{CONTROL_PORT} で待機中です。")

        # --- 画像送信用ソケット (ポート 12346) ---
        # 送信専用なので bind は不要
        self.image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"UDP画像送信ソケットを {IMAGE_PORT} で準備します。")

        # 最後に通信したクライアントのアドレス
        self.last_client_address = None 
        # 画像送信先のアドレス（IP, 画像受信ポート）
        self.image_target_address = None 
        
        # JPEG圧縮品質 (0-100)
        self.declare_parameter('jpeg_quality', 50)
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value

        #---パブリッシャー--
        self.state_pub = self.create_publisher(String, '/state_transition', 10)
        self.pub_joy = self.create_publisher(Joy,'/whill/controller/joy', 10)

        #---サブスクライバー--
        # 障害物情報
        self.sub_obstacle_info = self.create_subscription(
            BoolMultiArray, 'obstacle_info', self.callback_lidar, 10)
        # ロボット状態
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_update, 10)
        
        # (追記) ゲーム画像
        self.bridge = CvBridge()
        self.sub_game_image = self.create_subscription(
            Image,
            '/camera/gameimage', # 購読するトピック名
            self.game_image_callback,
            10) # キューサイズは1など小さくても良い

        #---状態変数--
        self.obstacle_front = 0
        self.obstacle_right = 0
        self.obstacle_left = 0
        self.obstacle_back = 0
        self.now_state = "USER"

        #--- 初期化処理 ---
        transition_msg = String()
        transition_msg.data = self.now_state
        self.state_pub.publish(transition_msg)

        # タイマーコールバックで制御ポートの受信処理
        self.timer = self.create_timer(0.02, self.receive_control_data) # 50Hz

        
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

    def callback_lidar(self, msg):
        """LiDARからの障害物情報に基づいて状態を更新する"""
        self.obstacle_front = 1 if msg.data[1] else 0
        self.obstacle_right = 1 if msg.data[3] else 0
        self.obstacle_left  = 1 if msg.data[9] else 0
        self.obstacle_back  = 1 if msg.data[6] else 0

    def receive_control_data(self):
        """UDP制御ソケット(12345)の受信と応答"""
        try:
            # 制御データを受信 (ノンブロッキング)
            data, address = self.control_sock.recvfrom(1024)
            
            # 最後に通信したクライアントのアドレスを更新
            # これにより、画像送信先が自動的に決まる
            if self.last_client_address != address:
                self.last_client_address = address
                # 画像送信先のポートも設定 (クライアント側は IMAGE_PORT で待ち受ける必要がある)
                self.image_target_address = (address[0], IMAGE_PORT)
                self.get_logger().info(f"クライアント {address[0]} に接続しました。")
                self.get_logger().info(f"画像送信先を {self.image_target_address[0]}:{self.image_target_address[1]} に設定しました。")

            message = data.decode('utf-8')
            
            # 受信メッセージをJoy指令として解析
            try:
                if self.now_state == 'HELPER':
                    pass
                else:
                    parts = message.split(',')
                    if len(parts) == 2:
                        angular = float(parts[0]) # 旋回
                        linear = float(parts[1])  # 前後進
                        self.send_joy(angular, linear)
            except (ValueError, IndexError):
                # 解析できないメッセージは無視する
                pass

            #障害物情報と，現在のロボット状態を送信 (制御ポートに応答)
            response_data = {
                "obstacle_front": self.obstacle_front,
                "obstacle_right": self.obstacle_right,
                "obstacle_left": self.obstacle_left,
                "obstacle_back": self.obstacle_back,
                "robot_state": self.now_state
            }
            # 文字列に変換して送信 (例: "{'obstacle_front': 0, ...}")
            response_message = str(response_data)
            self.control_sock.sendto(response_message.encode('utf-8'), address)
            
        except BlockingIOError:
            # データが来ていない場合は何もしない (正常)
            pass
        except (ValueError, socket.error) as e:
            self.get_logger().error(f"制御データの処理または通信中にエラーが発生しました: {e}")

    def game_image_callback(self, msg: Image):
        """(追記) /image/gameimage を受信したらUDP(12346)で送信する"""
        
        # まだクライアント(Windows)と一度も通信していない場合は何もしない
        if not self.image_target_address:
            return

        try:
            # ROS Image -> OpenCV 形式 (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # OpenCV画像をJPEGにエンコード
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        result, frame_encoded = cv2.imencode('.jpg', cv_image, encode_param)

        if not result:
            self.get_logger().warn('JPEG encoding failed')
            return

        # 圧縮データをバイト列に変換
        data_bytes = frame_encoded.tobytes()

        # UDPパケットの最大サイズ（約64KB）をチェック
        if len(data_bytes) > 65507:
            self.get_logger().warn(f'画像サイズが大きすぎます ({len(data_bytes)} bytes)。JPEG品質を下げるか、解像度を下げてください。')
            return

        # UDPで送信 (画像用ソケットを使用)
        try:
            self.image_sock.sendto(data_bytes, self.image_target_address)
        except Exception as e:
            # 送信エラーは頻発する可能性があるため、エラーレベルを調整しても良い
            self.get_logger().warn(f'画像UDPパケットの送信に失敗しました: {e}')
            
    def destroy_node(self):
        self.get_logger().info("UDPサーバーノードを終了します。")
        self.control_sock.close()
        self.image_sock.close() # 画像用ソケットも閉じる
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