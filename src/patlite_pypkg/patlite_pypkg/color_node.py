"""
パトライトの表示制御
パトライトのタッチセンサを監視し，押されたら状態遷移
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
from enum import Enum
import rclpy.time
#import ne_usb_example as patlite
from . import ne_usb_example as patlite

class Mode(str, Enum):
    USER = 'USER'
    HELPER = 'HELPER'
    GAME = 'GAME'
    ERROR = 'ERROR'

class Engage(str, Enum):
    FALL = 'FALL'
    RISE = 'RISE'

class PatLite(Node):
    def __init__(self):
        super().__init__('color_node')
        #パトライトとの接続
        self.dev = patlite.usb_open()
        if self.dev is None:
            self.get_logger().error("Failed to connect to Patlite.")
            sys.exit(1)
        else:
            self.get_logger().info("Patlite device connected.")
            patlite.set_light(self.dev, 2, 1)#緑点灯

        #ロボット状態を購読
        self.state_sub = self.create_subscription(String, '/robot_state', self.on_state_update, 10)
        
        #状態変更要求
        self.cmd_pub = self.create_publisher(String, '/state_transition', 10)

        #タッチセンサの押下時間検出用
        self.is_pressed = False
        self.press_start_time = None
        self.pub_flag = False#配信済みフラグ．Falseならまだ配信していない
        self.sensor_timer = self.create_timer(0.05, self.check_sensor_ex)#タッチセンサ監視タイマ 50ミリ秒周期

    def on_state_update(self, msg: String):
        """ロボット状態の更新を受け取り、対応するパトライト制御を行う"""
        state = msg.data
        self.get_logger().info(f"Received state: {state}")

        # 状態に応じたパトライト制御
        if state == Mode.USER:
            patlite.set_light(self.dev, 2, 1)#緑
        elif state == Mode.HELPER:
            patlite.set_light(self.dev, 4, 1)#青
        elif state == Mode.GAME:
            patlite.set_light(self.dev, 5, 1)#紫
        elif state == Mode.ERROR:
            patlite.set_light(self.dev, 1, 5)#赤, 点滅
        else:
            self.get_logger().warn(f"Unknown state: {state}")

    def check_sensor(self):
        """タッチセンサの状態を監視し、押下時間に応じてメッセージをパブリッシュ"""
        try:
            current_is_pressed = patlite.GetTouchSensorState(self.dev)#センサーの値を取得
            current_time = self.get_clock().now()

            # ボタンが押された瞬間を検出
            if current_is_pressed and not self.is_pressed:#初回の押下なら
                self.is_pressed = 1#押下フラグをTrue
                self.press_start_time = current_time#押下開始時間
                self.get_logger().info("Touch sensor pressed.")
            
            # ボタンが離された瞬間を検出
            elif not current_is_pressed and self.is_pressed:#現在押されていないかつ，押下フラグがTrueなら
                self.is_pressed = 0#押下フラグをFalse
                
                if self.press_start_time is not None:#押下開始時刻が存在するなら
                    duration_ns = (current_time - self.press_start_time).nanoseconds#押下継続時間を計算[ナノ秒]
                    duration_s = duration_ns / 1e9#秒に変換

                    transition_msg = String()
                    if duration_s >= 2.0:
                        transition_msg.data = Mode.USER
                        self.get_logger().info(f"1s Long press detected ({duration_s:.2f} s). Transitioning to USER.")
                    else:
                        transition_msg.data = Mode.HELPER
                        self.get_logger().info(f"Short press detected ({duration_s:.2f} s). Transitioning to HELPER.")
                    
                    #状態変更要求
                    self.cmd_pub.publish(transition_msg)
                
                self.press_start_time = None

        except Exception as e:
            self.get_logger().error(f"Error checking sensor: {e}")

    def check_sensor_ex(self):
        try:
            transition_msg = String()

            current_is_pressed = patlite.GetTouchSensorState(self.dev)#センサー値を取得
            current_time = self.get_clock().now()#現在時刻を取得

            if current_is_pressed and not self.is_pressed:#現在押されているかつ，押下フラグがFalse(初回の押下)なら
                self.is_pressed = 1#押下フラグをTrue
                self.press_start_time = current_time#押下開始時間
                self.pub_flag = False
                #self.get_logger().info("Touch sensor pressed.")
            elif current_is_pressed and self.is_pressed:#現在押されているかつ，押下フラグがTrueなら
                if not self.pub_flag and self.press_start_time is not None:#配信済フラグがFalseかつ，押下開始時刻が存在するなら
                    duration_ns = (current_time - self.press_start_time).nanoseconds#押下継続時間を計算[ナノ秒]
                    duration_s = duration_ns / 1e9#秒に変換
                    if duration_s >= 1.0:#2秒以上経過しているなら
                        #transition_msg.data = Mode.USER
                        transition_msg.data = Engage.FALL
                        self.pub_flag = True
                        self.cmd_pub.publish(transition_msg)
                #self.press_start_time = None
            elif not current_is_pressed and self.is_pressed:#現在押されていないかつ，押下フラグがTrueなら
                if not self.pub_flag:#配信済みフラグがFalseなら
                    if self.press_start_time is not None:#押下開始時刻が存在するなら
                        #transition_msg.data = Mode.HELPER
                        transition_msg.data = Engage.RISE
                        self.pub_flag = True
                        self.cmd_pub.publish(transition_msg)
                self.press_start_time = None
                self.pub_flag = False
                self.is_pressed = False

                
                        

        except Exception as e:
            self.get_logger().error(f"Error checking sensor: {e}")

    def destroy_node(self):
        patlite.set_light(self.dev, 0, 0)#消灯
        #USBのクローズ処理など
        self.get_logger().info("Resetting and closing Patlite device...")
        try:
            self.dev.reset()
            patlite.usb_close()
        except Exception as e:
            self.get_logger().error(f"Error closing device: {e}")
        super().destroy_node()

def main(args = None):
    rclpy.init()
    node = PatLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
