#humble_ws/src/camera_pkg/camera_pkg/camera_sub.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
import playsound
import time
from enum import Enum

class Mode(str, Enum):
    USER = 'USER'
    HELPER = 'HELPER'
    GAME = 'GAME'
    ERROR = 'ERROR'

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('game_node')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        # ---- パラメータ設定 ----
        self.tag_size = 0.072  # [m] 実際のタグの一辺長
        self.fx = 520.0       # カメラの焦点距離 [pixel]（要調整）
        self.fy = 520.0

        self.play_flag = False
        self.last_detect_time = time.time()
        self.reset_delay = 3.0  # 秒数（検出なしでリセット）
        self.last_read_id = 0
        self.total_score = 0

        # ---- サブスクライブ ----
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.get_logger().info('CameraSubscriberNode started: subscribing to /camera/image')
        #ロボット状態を購読
        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_update, 10)

        self.now_state = Mode.USER
        # ---- タイマーで定期的にリセット判定 ----
        self.create_timer(0.5, self.check_detection_timeout)
        
        #パブリッシャー
        self.img_pub = self.create_publisher(Image, '/camera/gameimage', 10)


    def state_update(self, msg: String):
        """/robot_stateトピックに基づいて内部の状態を更新する"""
        self.now_state = msg.data

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#sensor_msgs/ImageをOpenCV形式に変換
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray)

            if results:
                self.last_detect_time = time.time()

            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners#角４点
                pts = np.array([ptA, ptB, ptC, ptD], dtype=int)
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)
                h, w = cv_image.shape[:2]
                cv2.rectangle(cv_image, (0, 0), (w, h), (0, 255, 0), thickness=20)

                tag_id = r.tag_id

                # ---- 音声再生 ----
                if self.now_state == Mode.GAME:
                    if not self.play_flag and tag_id == 1:
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「よろしく頼む」.mp3', block=False)
                        self.play_flag = True
                        if tag_id - self.last_read_id == 1:
                            self.total_score = self.total_score + 1
                            self.last_read_id = tag_id
                    elif not self.play_flag and tag_id == 2:
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/決定ボタンを押す13.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「さあ、いくぞ！」.mp3', block=False)
                        self.play_flag = True
                        if tag_id - self.last_read_id == 1:
                            self.total_score = self.total_score + 1
                            self.last_read_id = tag_id
                    elif not self.play_flag and tag_id == 3:
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/決定ボタンを押す17.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「これは強敵だな…！」.mp3', block=False)
                        self.play_flag = True
                        if tag_id - self.last_read_id == 1:
                            self.total_score = self.total_score + 1
                            self.last_read_id = tag_id
                    elif not self.play_flag and tag_id == 4:
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/和太鼓でドン.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「やるじゃないか！」.mp3', block=False)
                        self.play_flag = True
                        if tag_id - self.last_read_id == 1:
                            self.total_score = self.total_score + 1
                            self.last_read_id = tag_id
                    elif not self.play_flag and tag_id == 5:
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/決定ボタンを押す5.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「お相手しましょう」.mp3', block=False)
                        self.play_flag = True
                        if tag_id - self.last_read_id == 1:
                            self.total_score = self.total_score + 1
                            self.last_read_id = tag_id
                    elif not self.play_flag and tag_id == 6:
                        #playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/ニワトリの鳴き声1.mp3', block=False)
                        playsound.playsound('/home/ubuntu/humble_ws/src/game_pkg/game_pkg/data/「くらえ！」.mp3', block=False)
                        self.play_flag = True
                        if tag_id - self.last_read_id == 1:
                            self.total_score = self.total_score + 1
                            self.last_read_id = 0
            cv2.rectangle(img=cv_image, pt1=(15, 20), pt2=(180, 120), color=(0,0,0), thickness=-1)
            cv2.putText(cv_image, f'{self.now_state}', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.putText(cv_image, f'NEXT:{self.last_read_id+1}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.putText(cv_image, f'SCORE:{self.total_score}', (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            
            #加工済ゲーム画面を配信
            try:
                img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'cv2_to_imgmsg failed: {e}')
                return
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame_2'
            self.img_pub.publish(img_msg)
                
            #cv2.imshow('AprilTag Detection', cv_image)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def check_detection_timeout(self):
        """一定時間検出がなければフラグをリセット"""
        if self.play_flag and (time.time() - self.last_detect_time > self.reset_delay):
            self.play_flag = False
            self.get_logger().info('3秒経過, 音声再生フラグをリセット')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
