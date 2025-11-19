"""
ロボットの操作モード遷移を管理するノード
定義する状態:
  USER      : ユーザ操作モード
  HELPER    : 介助者操作モード
  GAME      : ゲームモード
  ERROR     : エラー状態
他ノードは /robot_state を購読し、/state_transition に要求を送信して状態変更を行う
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum

#strEnum(文字列列挙型)
#メンバーを文字列として直接使用できる
#Mode.USERのように呼ぶ
class Mode(str, Enum):
    USER = 'USER'
    HELPER = 'HELPER'
    GAME = 'GAME'
    ERROR = 'ERROR'
class Engage(str, Enum):
    FALL = 'FALL'
    RISE = 'RISE'
class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager_node')

        self.state = Mode.USER#初期状態

        # Publisher: 現在の状態を配信
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        # Subscriber: 状態変更要求を受信
        self.cmd_sub = self.create_subscription(String, '/state_transition', self.on_transition, 10)

        # 初回配信
        self.publish_state()

    def publish_state(self):
        """現在の状態を /robot_state に配信"""
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)
        self.get_logger().info(f'現在状態: {self.state.value}')

    def on_transition(self, msg):
        """次の状態を受信し、遷移可能かチェックして配信"""
        next_state_str = msg.data.strip().upper()  # 受信文字列を正規化

        try:
            #next_state = Mode(next_state_str)#受信値が未定義の場合，エラー
            engage = Engage(next_state_str)#上昇or下降
            if self.state.value == Mode.HELPER:
                if engage == Engage.FALL:
                    next_state = Mode.USER
                elif engage == Engage.RISE:
                    next_state = Mode.HELPER
            if self.state.value == Mode.USER:
                if engage == Engage.FALL:
                    next_state = Mode.GAME
                elif engage == Engage.RISE:
                    next_state = Mode.HELPER
            if self.state.value == Mode.GAME:
                if engage == Engage.FALL:
                    next_state = Mode.GAME
                elif engage == Engage.RISE:
                    next_state = Mode.HELPER
            self.state = next_state
            self.publish_state()
            
        except ValueError:
            self.get_logger().warn(f'不明な状態要求を受信: {next_state_str}')
            return

        # 遷移チェック
        """
        if self.validate_transition(next_state):
            self.get_logger().info(f'状態遷移: {self.state.value} → {next_state.value}')
            self.state = next_state
            self.publish_state()
        else:
            self.get_logger().warn(f'不正な状態遷移要求: {self.state.value} → {next_state.value}')
        """
    def validate_transition(self, next_state: Mode) -> bool:
        """状態遷移が許されるかチェック"""
        """とりあえず今はすべての遷移が許される"""
        allowed = {
            Mode.USER: [Mode.USER, Mode.HELPER, Mode.GAME, Mode.ERROR],
            Mode.HELPER: [Mode.HELPER, Mode.USER, Mode.ERROR, Mode.GAME],
            Mode.GAME: [Mode.GAME, Mode.USER, Mode.HELPER, Mode.ERROR],
            Mode.ERROR: [Mode.USER, Mode.HELPER, Mode.ERROR, Mode.GAME],
        }
        return next_state in allowed.get(self.state, [])


def main():
    rclpy.init()
    node = StateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
