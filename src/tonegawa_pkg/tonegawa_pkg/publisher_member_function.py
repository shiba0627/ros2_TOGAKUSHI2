# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy#ros2のpythonライブラリ
from rclpy.node import Node#Nodeクラスを継承
from std_msgs.msg import String#ros2のメッセージ型, 文字列

#ノードクラス
class MinimalPublisher(Node):#Nodeクラスを継承
    def __init__(self):
        super().__init__('minimal_publisher')#ノード名定義
        self.publisher_ = self.create_publisher(String, 'topic', 10)#トピック名, メッセージ型, キューサイズ(保持できる最大メッセージ数)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)#0.5秒ごと動作
        self.i = 0#メッセージのカウント用
    def timer_callback(self):
        msg = String()#stting型のメッセージオブジェクト作成
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)#topicにメッセージ送信
        self.get_logger().info('Publishing: "%s"' % msg.data)#ros2ログに出力
        self.i += 1#カウントアップ


def main(args=None):
    rclpy.init(args=args)#ros2を初期化

    minimal_publisher = MinimalPublisher()#インスタンス化

    rclpy.spin(minimal_publisher)#ctrl+cされるまで継続実行

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()#ノード削除(なくても最悪良い)
    rclpy.shutdown()#ros2シャットダウン


if __name__ == '__main__':
    main()
