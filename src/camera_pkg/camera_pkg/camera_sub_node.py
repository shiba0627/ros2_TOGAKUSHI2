#humble_ws/src/camera_pkg/camera_pkg/camera_sub.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import apriltag
import numpy as np
import playsound
import time


class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_sub_node')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        # ---- パラメータ設定 ----
        self.tag_size = 0.114  # [m] 実際のタグの一辺長
        self.fx = 520.0       # カメラの焦点距離 [pixel]（要調整）
        self.fy = 520.0

        self.play_flag = False
        self.last_detect_time = time.time()
        self.reset_delay = 3.0  # 秒数（検出なしでリセット）

        # ---- サブスクライブ ----
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.get_logger().info('CameraSubscriberNode started: subscribing to /camera/image')

        # ---- タイマーで定期的にリセット判定 ----
        self.create_timer(0.5, self.check_detection_timeout)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            results = self.detector.detect(gray)

            if results:
                self.last_detect_time = time.time()

            for r in results:
                (ptA, ptB, ptC, ptD) = r.corners
                pts = np.array([ptA, ptB, ptC, ptD], dtype=int)
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)

                cX, cY = int(r.center[0]), int(r.center[1])
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

                # ---- 距離計算 ----
                tag_width_px = np.linalg.norm(ptA - ptB)
                distance = (self.tag_size * self.fx) / tag_width_px  # 単純なZ方向推定
                cv2.putText(cv_image, f"Dist: {distance:.2f} m",
                            (cX - 50, cY + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                tag_id = r.tag_id
                margin = r.decision_margin
                cv2.putText(cv_image, f"ID:{tag_id} M:{margin:.1f}",
                            (cX - 40, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # ---- 音声再生 ----
                if not self.play_flag:
                    playsound.playsound('/home/ubuntu/humble_ws/src/camera_pkg/camera_pkg/決定ボタンを押す4.mp3', block=False)

                    #self.get_logger().info(f'音声を再生: ID={tag_id}, 距離={distance:.2f}m')
                    self.play_flag = True

            cv2.imshow('AprilTag Detection', cv_image)
            cv2.waitKey(1)

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
